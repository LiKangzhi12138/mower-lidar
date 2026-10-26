"""Small repeatable R3 integrity and edge-case check; writes only under R3 outputs."""

import json
import tempfile
from pathlib import Path

import numpy as np
import open3d as o3d

from offline_detector import ROOT, analysis_coordinates, euclidean_clusters, filter_height_indices, run
from npz_to_ply import read_cloud_data


def check_ply(path, expected):
    if expected == 0:
        assert not path.exists(), path
        return np.empty((0, 3))
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    assert len(points) == expected, (path, len(points), expected)
    assert len(cloud.colors) == expected, (path, len(cloud.colors), expected)
    return points


def check_real_result(folder):
    import matplotlib.image as mpimg

    result = json.loads((folder / "result.json").read_text(encoding="utf-8"))
    preview = mpimg.imread(folder / "clusters_preview.png")
    assert preview.ndim == 3 and preview.shape[0] > 0 and preview.shape[1] > 0
    raw, _, _ = read_cloud_data(Path(result["input_file"]), verbose=False)
    raw = np.asarray(raw)
    with np.load(folder / "indices_and_labels.npz") as mapping:
        cleaned = mapping["clean_source_indices"]
        region = mapping["candidate_source_indices"]
        removed = mapping["removed_source_indices"]
        plane = mapping["plane_source_indices"]
        labels = mapping["candidate_labels"]
    assert len(cleaned) == result["preprocess_stats"]["remaining_point_count"]
    assert len(region) + len(removed) == len(cleaned)
    assert len(labels) == len(region)
    assert np.array_equal(np.sort(np.concatenate((region, removed))), cleaned)
    assert len(plane) == result["plane_candidate_point_count"]
    region_ply = check_ply(folder / "candidate_region.ply", len(region))
    check_ply(folder / "clusters_colored.ply", len(region))
    check_ply(folder / "plane_candidate.ply", len(plane))
    check_ply(folder / "removed_experimental.ply", len(removed))
    assert np.allclose(region_ply, raw[region], rtol=0, atol=1e-5)
    assert sum(c["point_count"] for c in result["candidates"]) == int((labels >= 0).sum())
    for candidate in result["candidates"]:
        indexed = raw[candidate["input_indices"]].astype(np.float64)
        cloud = check_ply(folder / candidate["point_cloud"], len(indexed))
        assert np.allclose(cloud, indexed, rtol=0, atol=1e-5)
        assert np.allclose(indexed.mean(axis=0), candidate["center_xyz"])
        assert np.allclose(indexed.min(axis=0), candidate["bbox_min_xyz"])
        assert np.allclose(indexed.max(axis=0), candidate["bbox_max_xyz"])
        distances = np.linalg.norm(indexed, axis=1)
        assert np.isclose(distances.min(), candidate["nearest_sensor_distance"])
        assert candidate["nearest_input_index"] in candidate["input_indices"]
    return result


def main():
    base = ROOT / "outputs" / "r3_offline_detector"
    run_names = [
        "batch1_sample_000001_raw_dbscan", "batch1_sample_000016_raw_dbscan",
        "batch2_sample_000001_raw_dbscan", "batch2_sample_000055_raw_dbscan",
        "batch1_sample_000001_ransac_euclidean",
        "batch1_sample_000001_patchwork_dbscan",
        "batch1_sample_000001_height_dbscan",
    ]
    results = {name: check_real_result(base / name) for name in run_names}
    same_frame = [r for r in results.values() if Path(r["input_file"]).stem == "batch1_sample_000001_clean"]
    assert len({r["input_sha256"] for r in same_frame}) == 1
    assert len({r["preprocess_stats"]["remaining_point_count"] for r in same_frame}) == 1

    # A known transform and known connected components check implementation math.
    transformed, frame = analysis_coordinates(
        np.array([[1., 0., 0.]]),
        {"mode": "experimental_mount", "mount_height": 2.0,
         "mount_pitch_degrees": 90.0, "pitch_axis_sensor": [0, 1, 0],
         "vertical_axis_analysis": [0, 0, 1], "analysis_frame": "test"},
    )
    assert frame == "test" and np.allclose(transformed, [[0, 0, 1]])
    assert np.array_equal(euclidean_clusters(np.array([[0, 0, 0], [0.1, 0, 0], [9, 0, 0]]),
                                             0.2, 2), [0, 0, -1])
    assert np.array_equal(filter_height_indices(np.array([[0, 0, 0], [0, 0, 1]]),
                                                [0, 0, 1], 0, 0.2), [0])

    with tempfile.TemporaryDirectory(prefix="r3_edge_", dir=base) as temporary:
        tmp = Path(temporary)
        base_config = json.loads((ROOT / "configs" / "r3_raw_dbscan.json").read_text(encoding="utf-8"))
        base_config["visualization"]["preview_png"] = False
        config_path = tmp / "config.json"

        def trial(name, points, ground, clustering):
            npz = tmp / f"{name}.npz"
            np.savez_compressed(npz, points=np.asarray(points, dtype=float).reshape(-1, 4),
                                fields=np.asarray(["x", "y", "z", "intensity"]))
            config = {**base_config, "ground": ground, "clustering": clustering}
            config_path.write_text(json.dumps(config), encoding="utf-8")
            result, folder = run(npz, config_path, tmp, name)
            check_ply(folder / "candidate_region.ply", result["candidate_region_point_count"])
            return result

        empty = trial("empty", [], {"method": "none"}, {"method": "dbscan", "params": {"eps": 0.2, "min_points": 2}})
        assert empty["candidate_count"] == 0 and empty["candidate_region_point_count"] == 0
        empty_patchwork = trial(
            "empty_patchwork", [], {"method": "patchwork", "params": {
                "assume_z_up_experimental": True, "sensor_height": 1.0,
                "min_range": 0.1, "max_range": 50.0, "enable_RNR": False}},
            {"method": "none"},
        )
        assert empty_patchwork["candidate_count"] == 0
        assert empty_patchwork["effective_patchwork_parameters"]["sensor_height"] == 1.0
        all_zero = trial("all_zero", [[0, 0, 0, 1], [0, 0, 0, 2]],
                         {"method": "none"}, {"method": "dbscan", "params": {"eps": 0.2, "min_points": 2}})
        assert all_zero["preprocess_stats"]["all_zero_xyz_row_count_removed"] == 2
        assert all_zero["candidate_region_point_count"] == 0
        too_few = trial("no_plane", [[1, 0, 0, 1], [2, 0, 0, 1]],
                        {"method": "ransac", "plane_policy": "retain", "params": {
                            "distance_threshold": 0.1, "ransac_n": 3, "num_iterations": 20,
                            "probability": 0.99, "random_seed": 1}},
                        {"method": "none"})
        assert too_few["plane_candidate_point_count"] == 0 and too_few["candidate_region_point_count"] == 2
        no_cluster = trial("no_cluster", [[1, 0, 0, 1], [5, 0, 0, 1], [9, 0, 0, 1]],
                           {"method": "none"}, {"method": "dbscan", "params": {"eps": 0.1, "min_points": 5}})
        assert no_cluster["candidate_count"] == 0 and no_cluster["noise_point_count"] == 3
        dirty = trial("nonfinite", [[1, 0, 0, 1], [np.nan, 1, 1, 1], [2, 0, 0, np.inf]],
                      {"method": "none"}, {"method": "none"})
        assert dirty["preprocess_stats"]["nonfinite_row_count_removed"] == 2
        assert dirty["candidate_region_point_count"] == 1

    print(json.dumps({"real_runs_checked": len(results), "same_input_methods_checked": len(same_frame),
                      "synthetic_cases": ["empty", "empty_patchwork", "all_zero", "no_plane", "no_cluster", "nonfinite"],
                      "transform_euclidean_height_math": "passed"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
