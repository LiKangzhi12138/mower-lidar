"""Run R3.10 Core regression through the existing entry point on verified R3.9 NPZs."""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from time import perf_counter

# Load the detector before Open3D so the original OpenMP setting is applied.
from run_perception import DEFAULT_CONFIG, run_pipeline

import numpy as np
from PIL import Image
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
FIELD_CONFIG = ROOT / "configs/field_66cm_7p5deg_ransac_dbscan.json"
HISTORY = ROOT / "outputs/r3_field_merging_diagnostic"
CASES = (("partial_overlap_20261119-165259", 92), ("front_1m_20261119-165004", 144))
DIAGNOSTICS = ("plane_model_ax_by_cz_d", "plane_candidate_point_count", "removed_point_count",
               "candidate_region_point_count", "selected_plane_diagnostics", "rejected_plane_candidates")


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                    encoding="utf-8")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_geometry(path, doc, coordinates):
    with np.load(path, allow_pickle=False) as data:
        raw = data["points"]
    pitch = np.deg2rad(coordinates["mount_pitch_degrees"])
    assert coordinates["pitch_axis_sensor"] == [0, 1, 0]
    assert coordinates["vertical_axis_analysis"] == [0, 0, 1]
    # Independently apply the fixed +Y rotation used by these experiment configs.
    sx, cx = np.sin(pitch), np.cos(pitch)
    for c in doc["candidates"]:
        sensor = raw[c["input_indices"], :3].astype(np.float64)
        x, y, z = sensor.T
        xyz = np.column_stack((cx*x + sx*z, y, -sx*x + cx*z + coordinates["mount_height"]))
        center = xyz.mean(axis=0)
        dist = np.linalg.norm(sensor, axis=1)
        nearest = int(dist.argmin())
        assert len(sensor) == c["point_count"]
        assert c["nearest_input_index"] == c["input_indices"][nearest]
        np.testing.assert_allclose(c["nearest_sensor_xyz"], sensor[nearest], atol=1e-12)
        np.testing.assert_allclose(c["nearest_sensor_distance"], dist[nearest], atol=1e-12)
        np.testing.assert_allclose(c["center_xyz"], center, atol=1e-12)
        np.testing.assert_allclose(c["bbox_min_xyz"], xyz.min(axis=0), atol=1e-12)
        np.testing.assert_allclose(c["bbox_max_xyz"], xyz.max(axis=0), atol=1e-12)
        np.testing.assert_allclose(c["bbox_extent_xyz"], np.ptp(xyz, axis=0), atol=1e-12)
        np.testing.assert_allclose(c["range_xy"], np.hypot(*center[:2]), atol=1e-12)
        np.testing.assert_allclose(c["nearest_range_xy"], np.hypot(xyz[:, 0], xyz[:, 1]).min(), atol=1e-12)
        np.testing.assert_allclose(c["azimuth_deg"], np.degrees(np.arctan2(center[1], center[0])),
                                   atol=1e-12)
        assert c["coordinate_frame"] == doc["coordinate_frame"] and c["point_cloud"] is None
    return len(doc["candidates"])


def geometry(doc):
    return [{k: v for k, v in c.items() if k != "point_cloud"} for c in doc["candidates"]]


def check_full(path, config, output, reference):
    result, doc = run_pipeline(path, output, config, mode="full")
    assert geometry(doc) == geometry(reference)
    for name, expected in (("ground", result["removed_point_count"]),
                           ("nonground", result["candidate_region_point_count"]),
                           ("obstacles_colored", result["candidate_region_point_count"])):
        cloud = o3d.io.read_point_cloud(str(output / f"{name}.ply"))
        assert len(cloud.points) == expected and cloud.has_colors()
    with Image.open(output / "radar_view.png") as picture:
        picture.verify()
    return doc


def verify_ground(result, doc, cfg):
    for key in DIAGNOSTICS:
        assert doc[key] == result[key]
    tolerance = cfg["ground"]["params"]
    selected = doc["selected_plane_diagnostics"]
    checks = (
        ("normal_angle_degrees", tolerance["max_normal_angle_degrees"]),
        ("abs_vertical_intercept", tolerance["max_abs_ground_intercept"]),
    )
    rejected = []
    for trial in doc["rejected_plane_candidates"]:
        reasons = [f"{key}: {trial[key]:.6g} > {limit}" for key, limit in checks if trial[key] > limit]
        assert reasons
        rejected.append({**trial, "reasons": reasons})
    if selected is None:
        assert doc["plane_model_ax_by_cz_d"] is None and doc["removed_point_count"] == 0
        reason = ("all configured trials rejected" if len(rejected) == tolerance["max_plane_trials"]
                  else "no accepted candidate; exact termination not exposed")
    else:
        assert doc["plane_model_ax_by_cz_d"] == selected["plane_model"]
        assert doc["plane_candidate_point_count"] == doc["removed_point_count"] == selected["point_count"] > 0
        assert all(selected[key] <= limit for key, limit in checks)
        reason = "; ".join(f"{key}: {selected[key]:.6g} <= {limit}" for key, limit in checks)
    return selected is not None, reason, rejected


def stats(values):
    return dict(zip(("p50", "p95", "max"), map(float, np.percentile(values, [50, 95, 100]))))


def main(output):
    baseline = load(DEFAULT_CONFIG)
    expected = deepcopy(baseline)
    expected["coordinates"].update(mount_height=0.66, mount_pitch_degrees=7.5)
    cfg = load(FIELD_CONFIG)
    assert cfg == expected
    old_diff = {DEFAULT_CONFIG: sha256(DEFAULT_CONFIG), FIELD_CONFIG: sha256(FIELD_CONFIG)}
    manifest = load(HISTORY / "converted/conversion_summary.json")
    assert manifest["success_count"] == 111 and not manifest["failures"]
    old_results = load(HISTORY / "mount_comparison.json")
    assert not output.exists() or not any(output.iterdir())
    output.mkdir(parents=True, exist_ok=True)
    entries, refs = [], {}
    point_count, candidate_count = 0, 0
    hashes = {}
    for bag, info in manifest["bags"].items():
        indexed = {old["id"]: old for old in old_results[bag]}
        for msg in info["messages"]:
            mid = msg["id"]
            source = HISTORY / "converted" / msg["path"]
            folder = output / "core" / bag / source.stem
            try:
                before = hashes[source] = sha256(source)
                started = perf_counter()
                result, doc = run_pipeline(source, folder, FIELD_CONFIG, mode="core")
                e2e = (perf_counter() - started)*1000
                assert Path(result["config_file"]) == FIELD_CONFIG.resolve()
                assert result["parameters"] == cfg and result["input_sha256"] == before
                assert doc["coordinate_parameters"] == cfg["coordinates"]
                assert doc["ground_parameters"] == cfg["ground"]
                assert doc["clustering_parameters"] == cfg["clustering"]
                assert {p.name for p in folder.iterdir()} == {"obstacles.json"}
                target = folder / "obstacles.json"
                assert load(target) == doc
                assert doc["source_result_json"] is None and doc["radar_plot"] is None
                assert doc["outputs"]["radar_view_png"] is None and doc["outputs"]["obstacles_colored_ply"] is None
                accepted, reason, rejected = verify_ground(result, doc, cfg)
                prep, timing = result["preprocess_stats"], result["timings"]
                ground, nonground = doc["removed_point_count"], doc["candidate_region_point_count"]
                assert prep["input_point_count"] == msg["points"]
                assert prep["remaining_point_count"] == msg["points"] - msg["zero_xyz"] - msg["nonfinite_rows"]
                assert ground + nonground == prep["remaining_point_count"]
                assert doc["candidate_count"] == len(doc["candidates"])
                noise = doc["unclustered_point_count"]
                assert sum(c["point_count"] for c in doc["candidates"]) + noise == nonground
                old = indexed[mid]["field"]
                for key in (*DIAGNOSTICS[:-1], "candidate_count"):
                    assert doc[key] == old[key], (bag, mid, key)
                assert noise == old["noise_point_count"]
                # R3.9 stores the same cluster geometry without per-point source indices.
                for actual, expected_candidate in zip(doc["candidates"], old["candidates"], strict=True):
                    assert all(actual[key] == value for key, value in expected_candidate.items())
                candidate_count += verify_geometry(source, doc, cfg["coordinates"])
                point_count += prep["input_point_count"]
                largest = max((c["point_count"] for c in doc["candidates"]), default=0)
                entries.append({"bag": bag, "id": mid, "success": True,
                                "input_points": prep["input_point_count"],
                                "valid_points": prep["remaining_point_count"], "accepted_ground": accepted,
                                "accept_or_failure_reason": reason, "rejected_trials": rejected,
                                "selected_plane": doc["selected_plane_diagnostics"],
                                "ground_points": ground, "nonground_points": nonground,
                                "clusters": doc["candidate_count"], "largest_cluster_points": largest,
                                "largest_cluster_ratio": largest/nonground if nonground else None,
                                "noise_points": noise, "noise_ratio": noise/nonground if nonground else None,
                                "ransac_ms": timing["ground_or_plane_seconds"]*1000,
                                "dbscan_ms": timing["clustering_seconds"]*1000,
                                "core_ransac_dbscan_ms": (timing["ground_or_plane_seconds"] +
                                                          timing["clustering_seconds"])*1000,
                                "detector_total_ms": timing["total_seconds"]*1000, "end_to_end_ms": e2e,
                                "obstacles_json": str(target.relative_to(output))})
                if (bag, mid) in CASES:
                    refs[bag, mid] = source, doc
            except Exception as error:
                entries.append({"bag": bag, "id": mid, "success": False,
                                "error": f"{type(error).__name__}: {error}"})
        print(bag, sum(x["success"] for x in entries if x["bag"] == bag), flush=True)
    dump(output / "per_message.json", entries)
    success = [item for item in entries if item["success"]]
    summary = {"total": len(entries), "processed_ok": len(success),
               "accepted_ground": sum(row["accepted_ground"] for row in success),
               "input_points": point_count, "candidate_geometries_checked": candidate_count,
               "configuration": cfg, "failures": [row for row in entries if not row["success"]]}
    if success:
        summary["timings_ms"] = {key: stats([x[key] for x in success]) for key in
                                ("ransac_ms", "dbscan_ms", "core_ransac_dbscan_ms",
                                 "detector_total_ms", "end_to_end_ms")}
    dump(output / "summary.json", summary)
    assert len(success) == 111 and summary["accepted_ground"] == 111

    for (bag, mid), (source, ref) in refs.items():
        check_full(source, FIELD_CONFIG, output / "full" / bag / source.stem, ref)

    history = load(ROOT / "outputs/r3_radar_view/sample_000142/obstacles.json")
    path = Path(history["input_file"])
    old, doc = run_pipeline(path, output / "legacy_142_core", mode="core")
    assert old["parameters"] == baseline
    assert doc["input_sha256"] == history["input_sha256"]
    assert geometry(doc) == geometry(history)
    assert check_full(path, DEFAULT_CONFIG, output / "legacy_142_full", doc)["candidates"] == history["candidates"]
    verify_geometry(path, doc, baseline["coordinates"])

    # Old settings on the same field input must expose a real no-plane result.
    source, _ = refs[CASES[0]]
    failure, doc = run_pipeline(source, output / "old_mount_case_92_core", mode="core")
    accepted, reason, rejected = verify_ground(failure, doc, baseline)
    assert not accepted and len(rejected) == 6
    assert doc["candidate_count"] > 0 and doc["candidate_region_point_count"] == failure["preprocess_stats"]["remaining_point_count"]
    assert all(sha256(path) == digest for path, digest in (hashes | old_diff).items())
    summary.update({"full_cases": [list(case) for case in CASES],
                    "legacy_142_core_full_candidate_count": history["candidate_count"],
                    "input_npz_hashes_unchanged": len(hashes),
                    "old_mount_failure_check": {"accepted": False, "reason": reason,
                                                "candidate_count": doc["candidate_count"], "trials": rejected}})
    dump(output / "summary.json", summary)
    print(json.dumps({k: summary[k] for k in ("processed_ok", "accepted_ground", "timings_ms")}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    main(parser.parse_args().output)
