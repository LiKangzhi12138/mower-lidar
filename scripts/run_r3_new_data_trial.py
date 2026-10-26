"""Repeatable R3.1 batch driver using the existing R3 offline detector."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from npz_to_ply import read_cloud_data
from offline_detector import ROOT, run
from prepare_cloud import filter_point_rows


DATA_ROOT = Path(r"E:\workspace\mower-data\data\newdata")
DEFAULT_OUTPUT = ROOT / "outputs" / "r3_new_data_trial"
SAMPLE_IDS = (115, 120, 125, 130, 132, 136, 139, 142, 144, 145, 147, 149)


def configs():
    coordinates = {
        "mode": "raw_sensor", "sensor_frame": "livox_frame",
        "coordinate_unit": None, "unit_confirmed": False,
        "mount_height": None, "mount_pitch_degrees": None,
    }
    base = {"schema_version": 1, "preprocess": {"zero_policy": "exclude"},
            "coordinates": coordinates, "visualization": {"preview_png": False}}
    return {
        "raw_dbscan": {**base, "ground": {"method": "none"},
                       "clustering": {"method": "dbscan", "params": {"eps": 0.35, "min_points": 12}}},
        "raw_euclidean": {**base, "ground": {"method": "none"},
                          "clustering": {"method": "euclidean", "params": {"radius": 0.35, "min_points": 12, "max_points": None}}},
        "ransac_plane": {**base, "ground": {"method": "ransac", "plane_policy": "retain",
                                            "params": {"distance_threshold": 0.15, "ransac_n": 3,
                                                       "num_iterations": 1000, "probability": 0.999,
                                                       "random_seed": 20261015}},
                         "clustering": {"method": "none"}},
    }


def cleaned_input(path):
    xyz, intensity, _ = read_cloud_data(path, verbose=False)
    keep, _ = filter_point_rows(xyz, intensity, "exclude")
    return np.asarray(xyz, dtype=np.float64)[keep], np.flatnonzero(keep)


def plot_comparison(path, sample, raw_xyz, xyz, plane_local, db_labels, eu_labels, limits, stats):
    projections = ((0, 1), (0, 2), (1, 2))
    names = ("Raw NPZ (orange: zero rows)", "RANSAC plane candidate", "Candidate region (cleaned)",
             "DBSCAN", "Euclidean")
    fig, axes = plt.subplots(3, 5, figsize=(22, 12), constrained_layout=True)
    for row, (i, j) in enumerate(projections):
        for col in range(5):
            ax = axes[row, col]
            ax.set_xlim(*limits[i])
            ax.set_ylim(*limits[j])
            # XY keeps geometric aspect; Z projections use vertical exaggeration
            # for legibility while retaining the same numeric limits everywhere.
            ax.set_aspect("equal" if row == 0 else "auto", adjustable="box")
            ax.grid(alpha=0.12)
            ax.set_xlabel("XYZ"[i] + " (unconfirmed coordinate values)")
            ax.set_ylabel("XYZ"[j] + " (unconfirmed coordinate values)")
            if row == 0:
                ax.set_title(names[col])
            if col in (0, 1, 2):
                cloud = raw_xyz if col == 0 else xyz
                finite = np.isfinite(cloud).all(axis=1)
                ax.scatter(cloud[finite, i], cloud[finite, j], s=0.4,
                           c="#666666" if col != 1 else "#c8c8c8", linewidths=0)
                if col == 0 and stats["zero_count"]:
                    ax.scatter([0], [0], s=34, c="#e67e22", marker="x", linewidths=1.2)
                if col == 1 and len(plane_local):
                    ax.scatter(xyz[plane_local, i], xyz[plane_local, j],
                               s=0.65, c="#145bdb", linewidths=0)
            else:
                labels = db_labels if col == 3 else eu_labels
                noise = labels < 0
                if noise.any():
                    ax.scatter(xyz[noise, i], xyz[noise, j], s=0.4, c="#888888", linewidths=0)
                for label in np.unique(labels[labels >= 0]):
                    mask = labels == label
                    hue = (0.11 + int(label) * 0.618033988749895) % 1.0
                    ax.scatter(xyz[mask, i], xyz[mask, j], s=0.7,
                               c=[plt.cm.hsv(hue)], linewidths=0)
    fig.suptitle(
        f"{sample}/lidar/cloud_01.npz | raw sensor frame, unit UNKNOWN | "
        f"clean={len(xyz)}, zero={stats['zero_count']}, plane={stats['plane_count']}, "
        f"DBSCAN={stats['dbscan_count']}, Euclidean={stats['euclidean_count']}\n"
        "Fixed axis ranges across 12 frames; XZ/YZ vertically exaggerated. Blue=geometric plane; gray=unclustered. "
        "Candidate region equals cleaned input; no plane removed.", fontsize=13,
    )
    fig.savefig(path, dpi=145)
    plt.close(fig)


def cluster_metrics(result, labels, clean_count):
    sizes = [c["point_count"] for c in result["candidates"]]
    return {
        "count": len(sizes), "small_clusters_le_30": sum(size <= 30 for size in sizes),
        "largest_cluster_points": max(sizes, default=0),
        "largest_cluster_fraction": max(sizes, default=0) / clean_count if clean_count else 0,
        "noise_points": int((labels < 0).sum()),
        "noise_fraction": float((labels < 0).mean()) if len(labels) else 0,
        "sizes": sizes,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=DATA_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--reuse-results", action="store_true", help="Regenerate comparisons and summary from existing R3 outputs")
    args = parser.parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    cfg_dir = out / "configs"
    cfg_dir.mkdir(exist_ok=True)
    paths = {}
    for name, cfg in configs().items():
        path = cfg_dir / f"{name}.json"
        path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
        paths[name] = path
    sources = [args.data_root / f"sample_{number:06d}" / "lidar" / "cloud_01.npz"
               for number in SAMPLE_IDS]
    if any(not path.is_file() for path in sources):
        raise FileNotFoundError("One or more selected newdata clouds are missing")
    cleaned = {path: cleaned_input(path) for path in sources}
    all_xyz = np.concatenate([value[0] for value in cleaned.values()])
    axis_limits = []
    for axis in range(3):
        lower, upper = np.quantile(all_xyz[:, axis], [0.005, 0.995])
        pad = max((upper - lower) * 0.05, 0.01)
        axis_limits.append([float(lower - pad), float(upper + pad)])
    (out / "selection.json").write_text(json.dumps({
        "source_root": str(args.data_root.resolve()),
        "sampling_rule": "12 snapshots across acquisition order; middle lidar/cloud_01.npz from each",
        "samples": [str(path.resolve()) for path in sources],
        "fixed_plot_axis_limits_raw_values": axis_limits,
        "axis_limit_rule": "0.5th to 99.5th percentile across all selected cleaned XYZ, plus 5% padding",
        "mount_pitch": "UNKNOWN", "mount_height": "UNKNOWN", "coordinate_unit": "UNKNOWN",
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    figures = out / "comparisons"
    figures.mkdir(exist_ok=True)
    summary = []
    for source in sources:
        sample = source.parent.parent.name
        xyz, clean_source = cleaned[source]
        results = {}
        labels = {}
        for method, cfg_path in paths.items():
            if args.reuse_results:
                folder = out / f"{sample}_{method}"
                result = json.loads((folder / "result.json").read_text(encoding="utf-8"))
            else:
                result, folder = run(source, cfg_path, out, f"{sample}_{method}")
            with np.load(folder / "indices_and_labels.npz") as indices:
                if not np.array_equal(indices["clean_source_indices"], clean_source):
                    raise AssertionError("R3 preprocess indices disagree across methods")
                if not np.array_equal(indices["candidate_source_indices"], clean_source):
                    raise AssertionError("Unexpected ground removal in raw/plane trial")
                labels[method] = indices["candidate_labels"].copy()
                if method == "ransac_plane":
                    plane_source = indices["plane_source_indices"].copy()
            results[method] = result
        if len({r["input_sha256"] for r in results.values()}) != 1:
            raise AssertionError("Method inputs differ")
        plane_local = np.searchsorted(clean_source, plane_source)
        if not np.array_equal(clean_source[plane_local], plane_source):
            raise AssertionError("Plane indices cannot be mapped to cleaned points")
        db = cluster_metrics(results["raw_dbscan"], labels["raw_dbscan"], len(xyz))
        eu = cluster_metrics(results["raw_euclidean"], labels["raw_euclidean"], len(xyz))
        # An Euclidean component touching several DBSCAN groups is a merge signal,
        # not evidence about the number of physical objects.
        eu_links = 0
        for eu_label in np.unique(labels["raw_euclidean"][labels["raw_euclidean"] >= 0]):
            subset = labels["raw_dbscan"][labels["raw_euclidean"] == eu_label]
            ids, counts = np.unique(subset[subset >= 0], return_counts=True)
            if (counts >= 20).sum() >= 2:
                eu_links += 1
        stats = {"plane_count": len(plane_local), "zero_count": results["raw_dbscan"]["preprocess_stats"]["all_zero_xyz_row_count"],
                 "dbscan_count": db["count"],
                 "euclidean_count": eu["count"]}
        figure_path = figures / f"{sample}_comparison.png"
        raw_xyz, _, _ = read_cloud_data(source, verbose=False)
        plot_comparison(figure_path, sample, np.asarray(raw_xyz), xyz, plane_local,
                        labels["raw_dbscan"], labels["raw_euclidean"], axis_limits, stats)
        metadata = json.loads((source.parent.parent / "metadata.json").read_text(encoding="utf-8"))
        cloud_info = next((item for item in metadata.get("clouds", []) if Path(item["file"]).name == source.name), {})
        entry = {
            "sample": sample, "input_file": str(source.resolve()),
            "input_sha256": results["raw_dbscan"]["input_sha256"],
            "input_points": results["raw_dbscan"]["preprocess_stats"]["input_point_count"],
            "clean_points": len(xyz),
            "all_zero_points": results["raw_dbscan"]["preprocess_stats"]["all_zero_xyz_row_count_removed"],
            "nonfinite_removed": results["raw_dbscan"]["preprocess_stats"]["nonfinite_row_count_removed"],
            "created_utc": metadata.get("created_utc"),
            "header_stamp_ns": cloud_info.get("header_stamp_ns"),
            "receive_time_ns": cloud_info.get("receive_time_ns"),
            "frame_id": cloud_info.get("frame_id"),
            "scene_hint": metadata.get("scene_hint") or "UNKNOWN",
            "label_hint": metadata.get("label_hint") or "UNKNOWN",
            "plane_count": len(plane_local),
            "plane_model": results["ransac_plane"]["plane_model_ax_by_cz_d"],
            "dbscan": db, "euclidean": eu,
            "euclidean_components_linking_multiple_dbscan_groups": eu_links,
            "times_seconds": {
                "dbscan_core": results["raw_dbscan"]["timings"]["clustering_seconds"],
                "euclidean_core": results["raw_euclidean"]["timings"]["clustering_seconds"],
                "ransac_core": results["ransac_plane"]["timings"]["ground_or_plane_seconds"],
            },
            "comparison_figure": str(figure_path.resolve()),
            "run_directories": {method: str((out / f"{sample}_{method}").resolve())
                                for method in paths},
        }
        summary.append(entry)
        print(json.dumps({"sample": sample, "clean": len(xyz), "plane": len(plane_local),
                          "dbscan": db["count"], "euclidean": eu["count"],
                          "dbscan_noise": db["noise_points"], "euclidean_noise": eu["noise_points"]},
                         ensure_ascii=False), flush=True)
    (out / "trial_summary.json").write_text(json.dumps({
        "result_type": "experimental_obstacle_candidates", "selected_frame_count": len(summary),
        "fixed_plot_axis_limits_raw_values": axis_limits, "samples": summary,
    }, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
