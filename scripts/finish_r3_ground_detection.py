"""Build and validate R3.2 visual outputs from offline_detector results."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from extract_obstacle_candidates import candidate_color, sha256_file, write_cloud
from npz_to_ply import read_cloud_data
from offline_detector import analysis_coordinates
from prepare_cloud import filter_point_rows


ROOT = Path(__file__).resolve().parent.parent
SOURCE = Path(r"E:\workspace\mower-data\data\newdata\sample_000142\lidar\cloud_01.npz")
OUT = ROOT / "outputs" / "r3_ground_comparison" / "sample_000142_final"
METHODS = ("patchwork", "ransac_normal")
GREEN = np.array([0.08, 0.8, 0.18])
RED = np.array([0.9, 0.22, 0.12])
GRAY = np.array([0.48, 0.48, 0.48])


def save_and_check(path, xyz, color):
    if not len(xyz):
        return None
    colors = np.tile(color, (len(xyz), 1)) if np.asarray(color).ndim == 1 else color
    write_cloud(path, xyz, colors)
    loaded = o3d.io.read_point_cloud(str(path))
    assert len(loaded.points) == len(xyz) == len(loaded.colors)
    assert np.allclose(np.asarray(loaded.points), xyz, atol=1e-5)
    assert np.allclose(np.asarray(loaded.colors), colors, atol=1 / 255 + 1e-5)
    return str(path.resolve())


def draw_grid(path, rows, limits, title):
    # rows: (method name, [(layer name, xyz, colors), ...])
    fig, axes = plt.subplots(3 * len(rows), 4, figsize=(17, 9 * len(rows)), constrained_layout=True)
    axes = np.asarray(axes).reshape(3 * len(rows), 4)
    for method_id, (method, layers) in enumerate(rows):
        for projection, (i, j) in enumerate(((0, 1), (0, 2), (1, 2))):
            for col, (name, xyz, colors) in enumerate(layers):
                ax = axes[3 * method_id + projection, col]
                if len(xyz):
                    ax.scatter(xyz[:, i], xyz[:, j], s=0.8, c=colors, linewidths=0, rasterized=True)
                ax.set_xlim(*limits[i]); ax.set_ylim(*limits[j])
                ax.set_aspect("equal" if projection == 0 else "auto")
                ax.grid(alpha=0.12)
                ax.set_xlabel("XYZ"[i] + " (m hypothesis)")
                ax.set_ylabel("XYZ"[j] + " (m hypothesis)")
                if projection == 0:
                    ax.set_title(method + " | " + name)
    fig.suptitle(title)
    fig.savefig(path, dpi=135)
    plt.close(fig)


def main(source=SOURCE, out=OUT):
    source = Path(source)
    out = Path(out)
    sensor_all, intensity, fields = read_cloud_data(source, verbose=False)
    keep, stats = filter_point_rows(sensor_all, intensity, "exclude")
    sensor = np.asarray(sensor_all)[keep].astype(np.float64)
    source_idx = np.flatnonzero(keep)
    assert fields == ["x", "y", "z", "intensity"]
    source_hash = sha256_file(source)
    results = {}
    layers_by_method = {}
    shared_xyz = None
    for method in METHODS:
        folder = out / method
        result = json.loads((folder / "result.json").read_text(encoding="utf-8"))
        assert result["input_sha256"] == source_hash
        analysis, frame = analysis_coordinates(sensor, result["parameters"]["coordinates"])
        assert result["analysis_frame"] == frame
        if shared_xyz is None:
            shared_xyz = analysis
        else:
            assert np.array_equal(shared_xyz, analysis)
        with np.load(folder / "indices_and_labels.npz") as mapped:
            clean_source = mapped["clean_source_indices"]
            candidate_source = mapped["candidate_source_indices"]
            ground_source = mapped["removed_source_indices"]
            unclassified_source = mapped["unclassified_source_indices"]
            labels = mapped["candidate_labels"]
        assert np.array_equal(clean_source, source_idx)
        assert len(np.intersect1d(candidate_source, ground_source)) == 0
        assert np.array_equal(np.sort(np.concatenate((candidate_source, ground_source))), source_idx)
        assert len(candidate_source) == len(labels) == result["candidate_region_point_count"]
        assert len(ground_source) == result["removed_point_count"]
        assert len(unclassified_source) == result["unclassified_point_count"]
        assert np.all(np.isin(unclassified_source, candidate_source))
        ground_rows = np.searchsorted(source_idx, ground_source)
        candidate_rows = np.searchsorted(source_idx, candidate_source)
        unclassified_rows = np.searchsorted(source_idx, unclassified_source)
        nonground_source = np.setdiff1d(candidate_source, unclassified_source)
        nonground_rows = np.searchsorted(source_idx, nonground_source)
        assert np.array_equal(source_idx[ground_rows], ground_source)
        assert np.array_equal(source_idx[candidate_rows], candidate_source)
        assert len(ground_rows) + len(nonground_rows) + len(unclassified_rows) == len(sensor)
        cluster_colors = np.tile(GRAY, (len(candidate_rows), 1))
        for cluster_id in np.unique(labels[labels >= 0]):
            cluster_colors[labels == cluster_id] = candidate_color(int(cluster_id))
        assert len(result["candidates"]) == len(np.unique(labels[labels >= 0]))
        for candidate in result["candidates"]:
            rows = candidate_rows[labels == candidate["id"]]
            xyz = analysis[rows]
            raw = sensor[rows]
            assert np.array_equal(source_idx[rows], candidate["input_indices"])
            assert np.allclose(xyz.mean(axis=0), candidate["center_xyz"])
            assert np.allclose(xyz.min(axis=0), candidate["bbox_min_xyz"])
            assert np.allclose(xyz.max(axis=0), candidate["bbox_max_xyz"])
            assert np.allclose(np.ptp(xyz, axis=0), candidate["bbox_extent_xyz"])
            distances = np.linalg.norm(raw, axis=1)
            nearest = int(np.argmin(distances))
            assert np.isclose(distances[nearest], candidate["nearest_sensor_distance"])
            assert source_idx[rows[nearest]] == candidate["nearest_input_index"]
        outputs = {
            "original": save_and_check(folder / "original.ply", analysis, [0.65, 0.65, 0.65]),
            "sensor_raw_clean": save_and_check(folder / "sensor_raw_clean.ply", sensor, [0.65, 0.65, 0.65]),
            "ground": save_and_check(folder / "ground.ply", analysis[ground_rows], GREEN),
            "nonground": save_and_check(folder / "nonground.ply", analysis[nonground_rows], RED),
            "obstacles_colored": save_and_check(folder / "obstacles_colored.ply", analysis[candidate_rows], cluster_colors),
            "unclassified": save_and_check(folder / "unclassified.ply", analysis[unclassified_rows], [0.95, 0.6, 0.1]),
            "unclustered": save_and_check(folder / "unclustered.ply", analysis[candidate_rows[labels < 0]], GRAY),
        }
        layers_by_method[method] = (
            ("cleaned + trial leveling", analysis, np.tile([0.42, 0.42, 0.42], (len(analysis), 1))),
            ("ground candidate", analysis[ground_rows], np.tile(GREEN, (len(ground_rows), 1))),
            ("nonground; unclassified orange", analysis[candidate_rows],
             np.where(np.isin(candidate_source, unclassified_source)[:, None],
                      np.array([0.95, 0.6, 0.1]), RED)),
            ("DBSCAN; gray unclustered", analysis[candidate_rows], cluster_colors),
        )
        results[method] = {
            "ground": len(ground_rows), "nonground": len(nonground_rows),
            "unclassified": len(unclassified_rows), "unclustered_dbscan": int(np.count_nonzero(labels < 0)),
            "clusters": result["candidate_count"], "timings": result["timings"],
            "outputs": outputs,
        }
    limits = []
    for axis in range(3):
        lo, hi = np.min(shared_xyz[:, axis]), np.max(shared_xyz[:, axis])
        margin = 0.03 * (hi - lo)
        limits.append((float(lo - margin), float(hi + margin)))
    for method in METHODS:
        path = out / method / "overview.png"
        draw_grid(path, [(method, layers_by_method[method])], limits,
                  f"{source.parent.parent.name}/{source.stem} | +5 deg about sensor +Y ASSUMED | same analysis frame")
        results[method]["overview"] = str(path.resolve())
    comparison = out / "patchwork_vs_ransac.png"
    draw_grid(comparison, [(m, layers_by_method[m]) for m in METHODS], limits,
              "Patchwork++ vs normal-constrained RANSAC | experiment, not validated ground truth")
    summary = {
        "status": "experimental_hypothesis_not_calibrated",
        "source": str(source.resolve()), "input_sha256": source_hash,
        "input_fields": fields, "preprocess": stats,
        "common_analysis_frame": "experimental_level_x_forward_y_left_z_up",
        "unit_status": "meters inferred from official ROS driver, NPZ exporter not located",
        "assumed_vehicle_forward_sensor_axis": "+X; not verified by a mount photo or per-frame extrinsics",
        "rotation": "sensor to level R_y(+5 degrees), followed by +0.415 on analysis Z",
        "height_reference": "0.415 m measured above ground; offset to internal sensor origin unknown",
        "fixed_xyz_plot_limits": limits,
        "methods": results, "comparison_png": str(comparison.resolve()),
        "checks": "source-index partition, same cleaned and leveled input, candidate coordinates, centroid, AABB, nearest raw-sensor-origin point/distance, PLY RGB and point coordinates verified",
    }
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({m: {k: results[m][k] for k in ("ground", "nonground", "unclassified", "unclustered_dbscan", "clusters")} for m in METHODS}))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=SOURCE)
    parser.add_argument("--output-dir", type=Path, default=OUT)
    args = parser.parse_args()
    main(args.input, args.output_dir)
