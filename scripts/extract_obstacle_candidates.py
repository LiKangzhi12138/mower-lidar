"""Offline RANSAC-plane and DBSCAN candidate experiment on an R1 clean NPZ."""

from __future__ import annotations

import argparse
import colorsys
import hashlib
import json
import os
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

# Open3D's parallel RANSAC can vary despite a fixed random seed.
# Set this before importing Open3D so repeated CLI runs use one worker.
OPENMP_THREADS = 1
os.environ["OMP_NUM_THREADS"] = str(OPENMP_THREADS)
import open3d as o3d

from npz_to_ply import read_cloud_data


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "outputs" / "r2_obstacle_candidates"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text(encoding="utf-8"))
    required = {
        "random_seed",
        "ransac_distance_threshold",
        "ransac_n",
        "ransac_num_iterations",
        "ransac_probability",
        "dbscan_eps",
        "dbscan_min_points",
    }
    if set(config) != required:
        raise ValueError(f"配置字段须恰好为 {sorted(required)}；实际为 {sorted(config)}")
    if isinstance(config["random_seed"], bool) or not isinstance(config["random_seed"], int):
        raise ValueError("random_seed 必须为整数")
    for name in ("ransac_n", "ransac_num_iterations", "dbscan_min_points"):
        if isinstance(config[name], bool) or not isinstance(config[name], int):
            raise ValueError(f"{name} 必须为整数")
    for name in ("ransac_distance_threshold", "ransac_probability", "dbscan_eps"):
        if isinstance(config[name], bool) or not isinstance(config[name], (int, float)):
            raise ValueError(f"{name} 必须为数值")
        if not np.isfinite(config[name]):
            raise ValueError(f"{name} 必须为有限值")
    if config["random_seed"] < 0 or config["ransac_n"] < 3:
        raise ValueError("random_seed 须非负，ransac_n 至少为 3")
    if config["ransac_num_iterations"] < 1 or config["dbscan_min_points"] < 1:
        raise ValueError("迭代次数和 DBSCAN min_points 必须为正整数")
    if config["ransac_distance_threshold"] <= 0 or config["dbscan_eps"] <= 0:
        raise ValueError("RANSAC 阈值和 DBSCAN eps 必须为正数")
    if not 0 < config["ransac_probability"] <= 1:
        raise ValueError("ransac_probability 必须在 (0, 1] 内")
    return config


def cloud_from_xyz(xyz: np.ndarray, colors: np.ndarray | None = None):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz.astype(np.float64, copy=False))
    if colors is not None:
        cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def write_cloud(path: Path, xyz: np.ndarray, colors: np.ndarray):
    if not o3d.io.write_point_cloud(
        str(path), cloud_from_xyz(xyz, colors), write_ascii=False
    ):
        raise RuntimeError(f"PLY 写入失败: {path}")


def candidate_color(cluster_id: int) -> tuple[float, float, float]:
    hue = (0.11 + cluster_id * 0.618033988749895) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.78, 0.95)


def extract_plane_candidate(
    xyz: np.ndarray, distance_threshold: float, ransac_n: int,
    num_iterations: int, probability: float, random_seed: int,
):
    """Return a geometric plane model and input row indices; no ground semantics."""
    o3d.utility.random.seed(random_seed)
    model, indices = cloud_from_xyz(xyz).segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
        probability=probability,
    )
    return np.asarray(model, dtype=np.float64), np.unique(np.asarray(indices, dtype=np.int64))


def cluster_dbscan(xyz: np.ndarray, eps: float, min_points: int) -> np.ndarray:
    """Return one label per input row, with -1 for DBSCAN noise."""
    if len(xyz) == 0:
        return np.empty(0, dtype=np.int32)
    return np.asarray(
        cloud_from_xyz(xyz).cluster_dbscan(
            eps=eps, min_points=min_points, print_progress=False,
        ), dtype=np.int32,
    )


def describe_candidate(
    analysis_xyz: np.ndarray, sensor_xyz: np.ndarray, input_indices: np.ndarray,
) -> dict:
    """Describe one nonempty cluster; distance is always from the sensor origin."""
    box = cloud_from_xyz(analysis_xyz).get_axis_aligned_bounding_box()
    sensor_distances = np.linalg.norm(sensor_xyz.astype(np.float64), axis=1)
    nearest = int(np.argmin(sensor_distances))
    return {
        "center_xyz": analysis_xyz.astype(np.float64).mean(axis=0).tolist(),
        "bbox_min_xyz": np.asarray(box.min_bound).tolist(),
        "bbox_max_xyz": np.asarray(box.max_bound).tolist(),
        "bbox_extent_xyz": np.asarray(box.get_extent()).tolist(),
        "nearest_sensor_xyz": sensor_xyz[nearest].astype(float).tolist(),
        "nearest_sensor_distance": float(sensor_distances[nearest]),
        "nearest_input_index": int(input_indices[nearest]),
    }


def write_preview(
    path: Path, xyz: np.ndarray, colors: np.ndarray, title: str,
    legend_suffix: str = "blue: plane candidate, gray: DBSCAN noise",
):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), constrained_layout=True)
    for ax, (i, j) in zip(axes, ((0, 1), (0, 2), (1, 2))):
        ax.scatter(xyz[:, i], xyz[:, j], s=0.45, c=colors, linewidths=0)
        ax.set_xlabel("XYZ"[i] + " (coordinate values)")
        ax.set_ylabel("XYZ"[j] + " (coordinate values)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.15)
    fig.suptitle(title + " | " + legend_suffix)
    fig.savefig(path, dpi=145)
    plt.close(fig)


def run_experiment(input_path: Path, config_path: Path, output_dir: Path, run_name: str):
    if not input_path.is_file():
        raise FileNotFoundError(input_path)
    if Path(run_name).name != run_name or run_name in ("", ".", ".."):
        raise ValueError("run-name 必须是单个安全目录名")
    config = load_config(config_path)

    xyz, intensity, fields = read_cloud_data(input_path, verbose=False)
    if fields != ["x", "y", "z", "intensity"] or intensity is None:
        raise ValueError("输入须为 R1 清理 NPZ，包含 x/y/z/intensity 规范字段")
    xyz = np.asarray(xyz)
    intensity = np.asarray(intensity).reshape(-1)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) != len(intensity):
        raise ValueError("输入点数或 XYZ 形状无效")
    if len(xyz) < config["ransac_n"]:
        raise ValueError("点数少于 RANSAC ransac_n")
    if not np.isfinite(xyz).all() or not np.isfinite(intensity).all():
        raise ValueError("R1 清理 NPZ 中仍有 NaN/Inf，请先核查 R1 处理")

    r1_report_path = input_path.with_name(input_path.name.replace("_clean.npz", "_processing.json"))
    r1_report = None
    if input_path.name.endswith("_clean.npz") and r1_report_path.is_file():
        r1_report = json.loads(r1_report_path.read_text(encoding="utf-8"))
        if r1_report.get("remaining_point_count") != len(xyz):
            raise ValueError("R1 处理记录与清理 NPZ 的点数不一致")

    plane_model, plane_indices = extract_plane_candidate(
        xyz,
        config["ransac_distance_threshold"],
        config["ransac_n"],
        config["ransac_num_iterations"],
        config["ransac_probability"],
        config["random_seed"],
    )
    remaining_mask = np.ones(len(xyz), dtype=bool)
    remaining_mask[plane_indices] = False
    remaining_indices = np.flatnonzero(remaining_mask)
    if not len(plane_indices) or not len(remaining_indices):
        raise ValueError("RANSAC 结果缺少平面候选或剩余点，无法进行本实验")

    remaining_xyz = xyz[remaining_indices]
    labels = cluster_dbscan(
        remaining_xyz, config["dbscan_eps"], config["dbscan_min_points"]
    )
    if len(labels) != len(remaining_indices):
        raise RuntimeError("DBSCAN 标签数与剩余点数不一致")

    plane_rgb = np.array([0.16, 0.53, 0.95], dtype=np.float64)
    noise_rgb = np.array([0.48, 0.48, 0.48], dtype=np.float64)
    remaining_colors = np.tile(noise_rgb, (len(remaining_indices), 1))
    candidates = []
    for cluster_id in sorted(int(v) for v in np.unique(labels) if v >= 0):
        local_indices = np.flatnonzero(labels == cluster_id)
        candidate_xyz = remaining_xyz[local_indices]
        details = describe_candidate(
            candidate_xyz, candidate_xyz, remaining_indices[local_indices]
        )
        rgb = candidate_color(cluster_id)
        remaining_colors[local_indices] = rgb
        candidates.append({
            "label": "experimental_spatial_obstacle_candidate",
            "cluster_id": cluster_id,
            "point_count": int(len(local_indices)),
            "centroid_xyz_coordinate_values": details["center_xyz"],
            "axis_aligned_bbox_min_xyz_coordinate_values": details["bbox_min_xyz"],
            "axis_aligned_bbox_max_xyz_coordinate_values": details["bbox_max_xyz"],
            "axis_aligned_bbox_extent_xyz_coordinate_values": details["bbox_extent_xyz"],
            "nearest_point_xyz_coordinate_values": details["nearest_sensor_xyz"],
            "nearest_point_distance_from_input_origin_coordinate_values": details["nearest_sensor_distance"],
            "nearest_point_index_in_r1_clean_npz": details["nearest_input_index"],
            "display_rgb": list(rgb),
        })

    run_dir = output_dir / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "plane_candidate_ply": run_dir / "plane_candidate.ply",
        "remaining_points_ply": run_dir / "remaining_points.ply",
        "clusters_colored_ply": run_dir / "clusters_colored.ply",
        "overview_ply": run_dir / "overview.ply",
        "projection_preview_png": run_dir / "projection_preview.png",
        "segmentation_npz": run_dir / "segmentation.npz",
        "result_json": run_dir / "result.json",
    }
    plane_colors = np.tile(plane_rgb, (len(plane_indices), 1))
    write_cloud(paths["plane_candidate_ply"], xyz[plane_indices], plane_colors)
    write_cloud(paths["remaining_points_ply"], remaining_xyz, np.tile(noise_rgb, (len(remaining_indices), 1)))
    write_cloud(paths["clusters_colored_ply"], remaining_xyz, remaining_colors)
    overview_colors = np.empty((len(xyz), 3), dtype=np.float64)
    overview_colors[plane_indices] = plane_rgb
    overview_colors[remaining_indices] = remaining_colors
    write_cloud(paths["overview_ply"], xyz, overview_colors)
    write_preview(paths["projection_preview_png"], xyz, overview_colors, run_name)
    np.savez_compressed(
        paths["segmentation_npz"],
        plane_indices=plane_indices,
        remaining_indices=remaining_indices,
        cluster_labels_remaining=labels,
    )

    report = {
        "schema_version": 1,
        "label": "experimental_spatial_obstacle_candidates",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "input_clean_npz": str(input_path.resolve()),
        "input_sha256": sha256_file(input_path),
        "r1_processing_json": str(r1_report_path.resolve()) if r1_report else None,
        "r1_zero_xyz_policy": r1_report.get("parameters", {}).get("zero_xyz_policy") if r1_report else None,
        "frame_id_from_r1_metadata": r1_report.get("frame_id_from_snapshot_metadata") if r1_report else None,
        "config_file": str(config_path.resolve()),
        "config_sha256": sha256_file(config_path),
        "algorithm_parameters": config,
        "openmp_threads": OPENMP_THREADS,
        "open3d_version": o3d.__version__,
        "coordinate_frame": "input R1 frame; no transform applied",
        "coordinate_units": "unverified raw coordinate values; not meters",
        "input_point_count": int(len(xyz)),
        "plane_candidate": {
            "label": "largest_RANSAC_plane_candidate_not_verified_ground",
            "model_ax_by_cz_d_eq_0": np.asarray(plane_model).astype(float).tolist(),
            "point_count": int(len(plane_indices)),
            "point_ratio": float(len(plane_indices) / len(xyz)),
        },
        "remaining_point_count": int(len(remaining_indices)),
        "dbscan_cluster_count": len(candidates),
        "dbscan_clustered_point_count": int(np.count_nonzero(labels >= 0)),
        "dbscan_noise_point_count": int(np.count_nonzero(labels < 0)),
        "candidates": candidates,
        "visualization": {
            "plane_rgb": plane_rgb.tolist(),
            "noise_rgb": noise_rgb.tolist(),
            "cluster_color_rule": "HSV hue=(0.11+cluster_id*golden_ratio_conjugate)%1, saturation=0.78, value=0.95",
            "bbox_type": "axis-aligned in input R1 coordinate frame",
        },
        "outputs": {key: str(path.resolve()) for key, path in paths.items()},
        "interpretation_limits": [
            "Largest plane may be ground, wall, or another structure.",
            "DBSCAN groups may split or merge physical objects.",
            "Closest-point distances are from input-frame origin in unverified coordinate values, not vehicle obstacle distances.",
        ],
    }
    paths["result_json"].write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "run": run_name,
        "input": len(xyz),
        "plane_candidate": len(plane_indices),
        "remaining": len(remaining_indices),
        "clusters": len(candidates),
        "clustered_points": report["dbscan_clustered_point_count"],
        "noise_points": report["dbscan_noise_point_count"],
        "result_json": str(paths["result_json"].resolve()),
    }, ensure_ascii=False))
    return report


def main():
    parser = argparse.ArgumentParser(
        description="R1 清理点云的 RANSAC 平面与 DBSCAN 实验性障碍候选提取"
    )
    parser.add_argument("input", type=Path, help="R1 的 *_clean.npz")
    parser.add_argument("--config", type=Path, required=True, help="包含全部算法参数的 JSON")
    parser.add_argument("--run-name", required=True, help="输出子目录名，需区分帧和参数组")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    run_experiment(args.input, args.config, args.output_dir, args.run_name)


if __name__ == "__main__":
    main()
