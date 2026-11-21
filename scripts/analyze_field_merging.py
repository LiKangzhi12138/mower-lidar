"""Offline R3.9 comparisons with the existing detector and separate experiment configs."""

import argparse
from collections import deque
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from offline_detector import run, _write_ply
from extract_obstacle_candidates import candidate_color, cluster_dbscan

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from convert_field_rosbag import BAGS, write_new_or_identical

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs/r3_2_ransac_normal_trial.json"
EPS = (0.20, 0.25, 0.30, 0.35)


def dump(path, document):
    data = (json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
    write_new_or_identical(path, data)


def configs(output):
    original = json.loads(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    paths = {}
    folder = output / "experiment_configs"
    folder.mkdir(exist_ok=True)
    for name, height, pitch, eps in [("old", 0.415, 5.0, 0.35)] + [
            (f"field_{eps:.2f}", 0.66, 7.5, eps) for eps in EPS]:
        cfg = deepcopy(original)
        cfg["coordinates"].update(mount_height=height, mount_pitch_degrees=pitch)
        cfg["clustering"]["params"]["eps"] = eps
        path = paths[name] = folder / f"{name}.json"
        dump(path, cfg)
    return paths


def analyze(source, config, output):
    context = {}
    result, _ = run(source, config, output, "unused", export_artifacts=False, context=context)
    xyz, rows, labels = (context[k] for k in ("analysis_xyz", "candidate_rows", "labels"))
    context["ground_rows"] = np.setdiff1d(np.arange(len(xyz)), rows)
    largest = max(result["candidates"], key=lambda item: item["point_count"], default=None)
    count = len(rows)
    params = result["parameters"]["ground"]["params"]
    for plane in result["rejected_plane_candidates"]:
        plane["reject_reasons"] = [
            reason for key, limit, reason in (
                ("normal_angle_degrees", "max_normal_angle_degrees", "normal_angle_exceeds_limit"),
                ("abs_vertical_intercept", "max_abs_ground_intercept", "abs_intercept_exceeds_limit"),
            ) if plane[key] > params[limit]]
    result["ground_diagnostic"] = ("accepted_candidate" if result["selected_plane_diagnostics"]
                                   else "no_accepted_plane; exact termination not exposed by detector")
    result["largest_cluster_id"] = largest["id"] if largest else None
    result["largest_cluster_points"] = largest["point_count"] if largest else 0
    result["largest_cluster_fraction_of_nonground"] = result["largest_cluster_points"] / max(count, 1)
    result["noise_fraction_of_nonground"] = result["noise_point_count"] / max(count, 1)
    result["largest_cluster_bbox_extent_xyz"] = largest["bbox_extent_xyz"] if largest else None
    result["nonground_index_sha256"] = hashlib.sha256(rows.tobytes()).hexdigest()
    # 前向窗口仅用于查图与统计，聚类输入仍为整个非地面点云。
    region = (xyz[:, 0] >= 0.4) & (xyz[:, 0] <= 3.0) & (np.abs(xyz[:, 1]) <= 2.0)
    ids, sizes = np.unique(labels[region[rows] & (labels >= 0)], return_counts=True)
    result["near_window_clusters"] = dict(zip(map(str, ids), map(int, sizes)))
    for candidate in result["candidates"]:
        candidate.pop("input_indices")
        candidate.pop("point_cloud")
    assert len(context["ground_rows"]) == result["removed_point_count"]
    assert sum(c["point_count"] for c in result["candidates"]) + result["noise_point_count"] == count
    return result, context


def compare_masks(old, field):
    a, b = old["ground_rows"], field["ground_rows"]
    both = len(np.intersect1d(a, b))
    return {"both": both, "old_only": len(a) - both, "field_only": len(b) - both,
            "intersection_over_union": both / max(len(a) + len(b) - both, 1)}


def write_layers(folder, context):
    folder.mkdir(parents=True, exist_ok=True)
    xyz, rows, labels = (context[k] for k in ("analysis_xyz", "candidate_rows", "labels"))
    ground = xyz[context["ground_rows"]]
    colors = np.array([candidate_color(int(label)) if label >= 0 else (.55, .55, .55)
                       for label in labels]).reshape(-1, 3)
    for name, pts, rgb in (("ground", ground, np.tile([.15, .8, .25], (len(ground), 1))),
                           ("nonground", xyz[rows], np.tile([.7, .7, .7], (len(rows), 1))),
                           ("obstacles_colored", xyz[rows], colors)):
        _write_ply(folder / f"{name}.ply", pts, rgb)


def chart(path, title, cases):
    fig, axes = plt.subplots(3, len(cases), figsize=(4.2 * len(cases), 11), squeeze=False,
                             layout="constrained")
    for col, (name, context) in enumerate(cases):
        xyz, rows, labels, ground = (context[k] for k in ("analysis_xyz", "candidate_rows", "labels", "ground_rows"))
        cluster_colors = {int(k): candidate_color(int(k)) for k in np.unique(labels) if k >= 0}
        for ax, (i, j, bounds) in zip(axes[:, col], ((0, 1, None),
                (0, 1, ((-0.3, 4), (-2.5, 2.5))), (0, 2, ((-0.3, 4), (-0.65, 2.4))))):
            ax.scatter(xyz[ground, i], xyz[ground, j], s=0.75, c="#63966b", linewidths=0, alpha=.42)
            noise = rows[labels < 0]
            ax.scatter(xyz[noise, i], xyz[noise, j], s=1, c="#808080", linewidths=0, alpha=.45)
            for k, color in cluster_colors.items():
                points = xyz[rows[labels == k]]
                ax.scatter(points[:, i], points[:, j], s=2.2, color=color, linewidths=0)
            if bounds:
                ax.set_xlim(*bounds[0])
                ax.set_ylim(*bounds[1])
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlabel("XYZ"[i] + " (coordinate units)")
            ax.set_ylabel("XYZ"[j] + " (coordinate units)")
            ax.grid(alpha=.15)
        axes[0, col].set_title(name)
    fig.suptitle(title + "\nGreen: ground candidate | gray: noise | colors: DBSCAN labels (per panel)")
    fig.savefig(path, dpi=125)
    plt.close(fig)


def overlap(baseline, context, largest_id):
    assert np.array_equal(baseline["candidate_rows"], context["candidate_rows"])
    selected = baseline["labels"] == largest_id
    ids, counts = np.unique(context["labels"][selected], return_counts=True)
    return dict(zip(map(str, ids), map(int, counts)))


def core_path(points, baseline_labels, source_mask, target_mask, eps):
    tree = cKDTree(points)
    core = tree.query_ball_point(points, eps, return_length=True) >= 12
    starts = np.flatnonzero(core & source_mask)
    targets = core & target_mask
    parent = np.full(len(points), -1, dtype=np.int64)
    parent[starts] = starts
    queue = deque(map(int, starts))
    while queue:
        node = queue.popleft()
        if targets[node]:
            path = [node]
            while parent[path[-1]] != path[-1]:
                path.append(int(parent[path[-1]]))
            return np.asarray(path[::-1])
        for other in tree.query_ball_point(points[node], eps):
            if core[other] and baseline_labels[other] == baseline_labels[node] and parent[other] < 0:
                parent[other] = node
                queue.append(other)
    raise ValueError("No core-to-core path for the selected endpoint groups")


def connectivity(folder, source, cfg, root):
    a, old = analyze(source, cfg["old"], root)
    b, field = analyze(source, cfg["field_0.35"], root)
    _, narrower = analyze(source, cfg["field_0.25"], root)
    assert np.array_equal(field["candidate_rows"], narrower["candidate_rows"])
    assert not len(old["ground_rows"])
    old_label = int(max(a["near_window_clusters"], key=a["near_window_clusters"].get))
    xyz = field["analysis_xyz"]
    removed = np.isin(np.arange(len(xyz)), field["ground_rows"])
    field_labels = np.full(len(xyz), -2, dtype=int)
    field_labels[field["candidate_rows"]] = field["labels"]
    nearby = ((xyz[:, 0] >= .4) & (xyz[:, 0] <= 3.0) & (np.abs(xyz[:, 1]) <= 2.0))
    ranked = sorted(b["near_window_clusters"], key=b["near_window_clusters"].get, reverse=True)
    groups = [int(x) for x in ranked[:2]]
    label_mask = old["labels"] == old_label
    path = core_path(xyz, old["labels"], label_mask & nearby & (field_labels == groups[0]),
                     label_mask & nearby & (field_labels == groups[1]), .35)
    # 固定现场坐标，仅切换地面点剔除，检验坐标刚体变换本身是否造成标签差异。
    field_no_removal = cluster_dbscan(xyz, .35, 12)
    values, counts = np.unique(field_labels[label_mask], return_counts=True)
    old_partition = dict(zip(map(str, values), map(int, counts)))
    rows = field["candidate_rows"]
    near = nearby[rows]
    label = groups[1]  # 取近场第二多点的簇，单独追踪 0.25 下的两个主要分支。
    base = field["labels"] == label
    values, counts = np.unique(narrower["labels"][base & near & (narrower["labels"] >= 0)], return_counts=True)
    subgroups = [int(values[i]) for i in np.argsort(-counts)[:2]]
    residual_path = core_path(xyz[rows], field["labels"], base & near & (narrower["labels"] == subgroups[0]),
                              base & near & (narrower["labels"] == subgroups[1]), .35)
    model = np.asarray(b["plane_model_ax_by_cz_d"])
    residual = (xyz[rows][residual_path] @ model[:3] + model[3]) / np.linalg.norm(model[:3])
    result = {
        "old_near_dominant_label": old_label, "old_near_dominant_size": int(label_mask.sum()),
        "old_cluster_partition_by_field_label_minus2_removed_minus1_noise": old_partition,
        "field_pose_without_ground_removal_labels_equal_old": bool(np.array_equal(field_no_removal, old["labels"])),
        "old_core_path": {"field_endpoint_labels": groups, "node_count": len(path),
                          "removed_as_field_ground": int(removed[path].sum()),
                          "xyz_in_field_analysis_frame": xyz[path].tolist()},
        "field_core_path": {"eps_035_label": label, "eps_025_endpoint_labels": subgroups,
                            "node_count": len(residual_path), "signed_plane_distance": residual.tolist(),
                            "xyz_in_field_analysis_frame": xyz[rows][residual_path].tolist()},
    }
    dump(folder / "connectivity.json", result)
    print(json.dumps({k: v for k, v in result.items() if not k.endswith("_path")}), flush=True)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output", type=Path, required=True)
    cli.add_argument("--phase", choices=("mounts", "sensitivity", "connectivity"), required=True)
    cli.add_argument("--case", action="append", nargs=2, metavar=("BAG", "MESSAGE_ID"))
    options = cli.parse_args()
    root = options.output
    manifest = json.loads((root / "converted/conversion_summary.json").read_text(encoding="utf-8"))
    if manifest["success_count"] != 111 or manifest["failures"]:
        raise ValueError("Expected successful conversion of all field messages before experiments")
    cfg = configs(root)
    if options.phase == "mounts":
        result = {}
        for bag, item in manifest["bags"].items():
            result[bag] = []
            for entry in item["messages"]:
                source = root / "converted" / entry["path"]
                a, old = analyze(source, cfg["old"], root)
                b, field = analyze(source, cfg["field_0.35"], root)
                clean = a["preprocess_stats"]["remaining_point_count"]
                assert clean == b["preprocess_stats"]["remaining_point_count"]
                result[bag].append({"id": entry["id"], "raw_points": entry["points"], "clean_points": clean,
                                    "ground_overlap": compare_masks(old, field), "old": a, "field": b})
            print(bag, len(result[bag]), flush=True)
        dump(root / "mount_comparison.json", result)
    elif options.phase == "sensitivity":
        cases = root / "cases"
        cases.mkdir(exist_ok=True)
        for bag, mid in options.case or ():
            if bag not in BAGS:
                raise ValueError(f"Unknown bag: {bag}")
            source = root / "converted" / bag / f"message_{int(mid):06d}.npz"
            folder = cases / f"{bag}__{int(mid):06d}"
            folder.mkdir()  # 不覆盖他人产物。
            a, old = analyze(source, cfg["old"], root)
            values = {f"{eps:.2f}": analyze(source, cfg[f"field_{eps:.2f}"], root) for eps in EPS}
            b, field = values["0.35"]
            write_layers(folder / "old", old)
            for key, (_, context) in values.items():
                write_layers(folder / f"field_eps_{key}", context)
            chart(folder / "mount_comparison.png", f"{bag}, message {mid}",
                  [("Old mount, eps .35", old), ("Field mount, eps .35", field)])
            chart(folder / "eps_comparison.png", f"{bag}, message {mid} · Field mount",
                  [(f"eps {key}", context) for key, (_, context) in values.items()])
            for result, context in values.values():
                assert b["plane_model_ax_by_cz_d"] == result["plane_model_ax_by_cz_d"]
                assert b["nonground_index_sha256"] == result["nonground_index_sha256"]
                result["point_overlap_from_eps_035_largest"] = overlap(field, context, b["largest_cluster_id"])
            np.savez_compressed(folder / "connectivity_arrays.npz",
                                points=field["analysis_xyz"][field["candidate_rows"]],
                                **{f"eps_{key.replace('.', '')}": context["labels"]
                                   for key, (_, context) in values.items()})
            dump(folder / "case.json", {"bag": bag, "id": int(mid), "old": a,
                                       "eps": {key: result for key, (result, _) in values.items()}})
            print(folder.name, flush=True)
    else:
        for bag, mid in options.case or ():
            source = root / "converted" / bag / f"message_{int(mid):06d}.npz"
            connectivity(root / "cases" / f"{bag}__{int(mid):06d}", source, cfg, root)


if __name__ == "__main__":
    main()
