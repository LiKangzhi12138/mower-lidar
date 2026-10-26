"""Run fixed R3.2 trial configs on sample_000143..149 only."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from extract_obstacle_candidates import sha256_file
from finish_r3_ground_detection import main as finish_visuals
from offline_detector import run


ROOT = Path(__file__).resolve().parent.parent
DATA = Path(r"E:\workspace\mower-data\data\newdata")
OUT = ROOT / "outputs" / "r3_multiscene_validation"
CONFIGS = {
    "ransac_normal": ROOT / "configs" / "r3_2_ransac_normal_trial.json",
    "patchwork": ROOT / "configs" / "r3_2_patchwork_trial.json",
}


def make_contact_sheet(paths):
    # The per-sample files retain all XY/XZ/YZ views. The contact sheet uses
    # larger XZ panels so seven samples remain legible at normal zoom.
    all_points = []
    for sample, _ in paths:
        p = OUT / sample / "patchwork" / "original.ply"
        all_points.append(np.asarray(o3d.io.read_point_cloud(str(p)).points))
    combined = np.concatenate(all_points)
    xlim = np.percentile(combined[:, 0], [0.5, 99.5])
    zlim = np.percentile(combined[:, 2], [0.5, 99.5])
    xlim += np.array([-1, 1]) * 0.05 * (xlim[1] - xlim[0])
    zlim += np.array([-1, 1]) * 0.05 * (zlim[1] - zlim[0])
    fig, axes = plt.subplots(7, 4, figsize=(18, 22), constrained_layout=True)
    columns = (("patchwork", "ground.ply", "Patch ground"),
               ("patchwork", "nonground.ply", "Patch nonground"),
               ("ransac_normal", "ground.ply", "RANSAC ground"),
               ("ransac_normal", "nonground.ply", "RANSAC nonground"))
    for row, (sample, _) in enumerate(paths):
        for col, (method, filename, title) in enumerate(columns):
            points = np.asarray(o3d.io.read_point_cloud(str(OUT / sample / method / filename)).points)
            ax = axes[row, col]
            ax.scatter(points[:, 0], points[:, 2], s=0.55,
                       color="#1eb548" if filename == "ground.ply" else "#e65a3d", linewidths=0)
            ax.set_xlim(*xlim); ax.set_ylim(*zlim); ax.grid(alpha=0.12)
            ax.set_title(f"{sample} | {title}", fontsize=9)
            ax.set_xlabel("X (m hypothesis)"); ax.set_ylabel("Z (m hypothesis)")
    fig.suptitle("Seven samples | fixed parameters and common XZ plot range | experimental mount hypothesis")
    path = OUT / "seven_sample_comparison.png"
    fig.savefig(path, dpi=145)
    plt.close(fig)
    return path


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    reference = ROOT / "outputs" / "r3_ground_comparison" / "sample_000142_final"
    for method, config in CONFIGS.items():
        previous = json.loads((reference / method / "result.json").read_text(encoding="utf-8"))
        assert previous["parameters"] == json.loads(config.read_text(encoding="utf-8")), method
    records = []
    images = []
    for number in range(143, 150):
        sample = f"sample_{number:06d}"
        folder = DATA / sample
        source = folder / "lidar" / "cloud_01.npz"
        output = OUT / sample
        reuse = output.exists() and any(output.iterdir())
        metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
        tf = json.loads((folder / "tf_static.json").read_text(encoding="utf-8"))
        cloud_meta = next(c for c in metadata["clouds"] if c["file"] == "lidar/cloud_01.npz")
        lidar_tf = next(t for t in tf["transforms"] if t["child_frame_id"] == "livox_frame")
        assert cloud_meta["frame_id"] == "livox_frame"
        output.mkdir(parents=True, exist_ok=True)
        for method, config in CONFIGS.items():
            if reuse:
                result_path = output / method / "result.json"
                if not result_path.is_file():
                    raise FileExistsError(f"Partial output requires manual review: {output}")
                previous = json.loads(result_path.read_text(encoding="utf-8"))
                if (previous["input_sha256"] != sha256_file(source) or
                        previous["parameters"] != json.loads(config.read_text(encoding="utf-8"))):
                    raise ValueError(f"Existing result has different input or parameters: {result_path}")
            else:
                run(source, config, output, method)
        finish_visuals(source, output)
        summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
        assert summary["input_sha256"] == sha256_file(source)
        assert summary["methods"]["patchwork"]["ground"] + summary["methods"]["patchwork"]["nonground"] + summary["methods"]["patchwork"]["unclassified"] == summary["preprocess"]["remaining_point_count"]
        record = {
            "sample": sample, "source": str(source.resolve()),
            "input_sha256": summary["input_sha256"],
            "created_utc": metadata.get("created_utc"),
            "scene_hint": metadata.get("scene_hint"),
            "operator_note": metadata.get("operator_note"),
            "cloud_frame_id": cloud_meta["frame_id"],
            "cloud_point_count_metadata": cloud_meta["point_count"],
            "tf_static_base_link_to_livox_frame": lidar_tf,
            "mount_status": "shared_R3_2_assumption_not_per_sample_verified",
            "summary": str((output / "summary.json").resolve()),
            "comparison_png": summary["comparison_png"],
            "clean_points": summary["preprocess"]["remaining_point_count"],
            "methods": {
                method: {key: value for key, value in summary["methods"][method].items()
                         if key in ("ground", "nonground", "unclassified", "unclustered_dbscan", "clusters", "timings", "outputs", "overview")}
                for method in CONFIGS
            },
        }
        records.append(record)
        images.append((sample, Path(summary["comparison_png"])))
        print(json.dumps({"sample": sample, "clean": record["clean_points"],
                          "ransac_ground": record["methods"]["ransac_normal"]["ground"],
                          "patchwork_ground": record["methods"]["patchwork"]["ground"]}), flush=True)
    contact = make_contact_sheet(images)
    aggregate = {
        "status": "experimental_shared_mount_assumption",
        "sample_ids": [r["sample"] for r in records],
        "excluded_reference_sample": "sample_000142",
        "excluded_earlier_samples": "sample_000115..sample_000141",
        "fixed_config_paths": {k: str(v.resolve()) for k, v in CONFIGS.items()},
        "fixed_config_sha256": {k: sha256_file(v) for k, v in CONFIGS.items()},
        "mount_hypothesis": "R3.2: +X forward, +Y left, +Z up; sensor-to-level R_y(+5deg); height 0.415 from ground, internal-origin offset unknown",
        "unit_status": "driver XYZ meters; NPZ exporter chain unverified",
        "contact_sheet": str(contact.resolve()),
        "records": records,
    }
    (OUT / "validation_summary.json").write_text(json.dumps(aggregate, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"samples": len(records), "contact_sheet": str(contact.resolve())}), flush=True)


if __name__ == "__main__":
    main()
