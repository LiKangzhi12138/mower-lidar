"""R3.2 preflight: reuse raw-frame outputs; do not infer a mount transform."""

import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from extract_obstacle_candidates import candidate_color, sha256_file
from npz_to_ply import read_cloud_data
from prepare_cloud import filter_point_rows


ROOT = Path(__file__).resolve().parent.parent
SOURCE = Path(r"E:\workspace\mower-data\data\newdata\sample_000142\lidar\cloud_01.npz")
OLD = ROOT / "outputs" / "r3_new_data_trial"
OUT = ROOT / "outputs" / "r3_ground_comparison"


def main():
    xyz, intensity, fields = read_cloud_data(SOURCE, verbose=False)
    mask, counts = filter_point_rows(xyz, intensity, "exclude")
    clean = np.asarray(xyz, dtype=np.float64)[mask]
    source_rows = np.flatnonzero(mask)
    source_hash = sha256_file(SOURCE)
    db_dir = OLD / "sample_000142_raw_dbscan"
    plane_dir = OLD / "sample_000142_ransac_plane"
    db_result = json.loads((db_dir / "result.json").read_text(encoding="utf-8"))
    plane_result = json.loads((plane_dir / "result.json").read_text(encoding="utf-8"))
    assert db_result["input_sha256"] == plane_result["input_sha256"] == source_hash
    with np.load(db_dir / "indices_and_labels.npz") as mapped:
        assert np.array_equal(mapped["clean_source_indices"], source_rows)
        labels = mapped["candidate_labels"].copy()
    with np.load(plane_dir / "indices_and_labels.npz") as mapped:
        assert np.array_equal(mapped["clean_source_indices"], source_rows)
        plane_rows = mapped["plane_source_indices"].copy()
        assert len(mapped["removed_source_indices"]) == 0
    plane_local = np.searchsorted(source_rows, plane_rows)
    assert np.array_equal(source_rows[plane_local], plane_rows)
    metadata = json.loads((SOURCE.parent.parent / "metadata.json").read_text(encoding="utf-8"))
    tf = json.loads((SOURCE.parent.parent / "tf_static.json").read_text(encoding="utf-8"))
    lidar_tf = next(item for item in tf["transforms"] if item["child_frame_id"] == "livox_frame")
    OUT.mkdir(parents=True, exist_ok=True)
    diag = OUT / "raw_frame_diagnostic"
    diag.mkdir(exist_ok=True)
    for src, dst, expected in (
        (db_dir / "candidate_region.ply", diag / "original.ply", len(clean)),
        (plane_dir / "plane_candidate.ply", diag / "plane_candidate.ply", len(plane_rows)),
        (db_dir / "clusters_colored.ply", diag / "clusters_colored.ply", len(clean)),
    ):
        cloud = o3d.io.read_point_cloud(str(src))
        assert len(cloud.points) == expected and len(cloud.colors) == expected
        shutil.copyfile(src, dst)
        assert len(o3d.io.read_point_cloud(str(dst)).points) == expected

    colors = np.tile([0.5, 0.5, 0.5], (len(clean), 1))
    for cluster_id in np.unique(labels[labels >= 0]):
        colors[labels == cluster_id] = candidate_color(int(cluster_id))
    names = ("Cleaned original", "Geometric plane candidate", "All candidate region", "Raw DBSCAN clusters")
    fig, axes = plt.subplots(3, 4, figsize=(17, 11), constrained_layout=True)
    for row, (i, j) in enumerate(((0, 1), (0, 2), (1, 2))):
        lo_i, hi_i = clean[:, i].min(), clean[:, i].max()
        lo_j, hi_j = clean[:, j].min(), clean[:, j].max()
        for col, ax in enumerate(axes[row]):
            ax.set_xlim(lo_i, hi_i)
            ax.set_ylim(lo_j, hi_j)
            ax.set_aspect("equal" if row == 0 else "auto")
            ax.grid(alpha=0.12)
            ax.set_xlabel("XYZ"[i] + " (unit unverified)")
            ax.set_ylabel("XYZ"[j] + " (unit unverified)")
            if row == 0:
                ax.set_title(names[col])
            if col == 1:
                ax.scatter(clean[:, i], clean[:, j], s=.4, c="#cccccc", linewidths=0)
                ax.scatter(clean[plane_local, i], clean[plane_local, j], s=.7, c="#175adc", linewidths=0)
            elif col == 3:
                ax.scatter(clean[:, i], clean[:, j], s=.6, c=colors, linewidths=0)
            else:
                ax.scatter(clean[:, i], clean[:, j], s=.4, c="#666666", linewidths=0)
    fig.suptitle("sample_000142/cloud_01 | RAW SENSOR FRAME ONLY | no ground classification or tilt correction")
    fig.savefig(diag / "overview.png", dpi=145)
    plt.close(fig)

    result = {
        "status": "formal_ground_comparison_not_run_preflight_blocked",
        "input_file": str(SOURCE.resolve()), "input_sha256": source_hash,
        "fields": fields, "point_counts": counts,
        "coordinate_unit": "UNKNOWN_FOR_THIS_NPZ",
        "sensor_axes_to_vehicle": "UNKNOWN",
        "pitch_axis_and_sign": "UNKNOWN",
        "mount_height_reported_m": 0.415,
        "mount_height_reference_point": "UNKNOWN",
        "mount_height_measurement_uncertainty": "UNKNOWN",
        "metadata_frame_id": next(c["frame_id"] for c in metadata["clouds"] if c["file"] == "lidar/cloud_01.npz"),
        "tf_static_livox": lidar_tf,
        "raw_geometric_plane_points": len(plane_rows),
        "raw_dbscan_clusters": db_result["candidate_count"],
        "raw_dbscan_noise_points": db_result["noise_point_count"],
        "raw_ransac_core_seconds": plane_result["timings"]["ground_or_plane_seconds"],
        "raw_dbscan_core_seconds": db_result["timings"]["clustering_seconds"],
        "raw_candidate_json": str((db_dir / "result.json").resolve()),
        "diagnostic_outputs": [str((diag / name).resolve()) for name in
                               ("original.ply", "plane_candidate.ply", "clusters_colored.ply", "overview.png")],
        "requested_ground_combinations": {
            "minus_5_patchwork": None, "minus_5_constrained_ransac": None,
            "minus_10_patchwork": None, "minus_10_constrained_ransac": None,
        },
    }
    (OUT / "preflight.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": result["status"], "clean_points": len(clean),
                      "plane_candidate": len(plane_rows), "raw_dbscan_clusters": db_result["candidate_count"]},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
