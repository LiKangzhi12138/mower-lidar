"""Standardize one Snapshot NPZ for downstream geometry experiments."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from npz_to_ply import read_cloud_data


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "outputs" / "r1_input_prep"


def filter_point_rows(xyz: np.ndarray, intensity: np.ndarray | None, zero_policy: str):
    """Return the R1 row mask and counts without changing source arrays."""
    xyz = np.asarray(xyz)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f"XYZ 形状必须是 N×3，实际为 {xyz.shape}")
    if zero_policy not in ("exclude", "keep"):
        raise ValueError("zero_policy 必须为 exclude 或 keep")
    if intensity is not None:
        intensity = np.asarray(intensity).reshape(-1)
        if len(intensity) != len(xyz):
            raise ValueError("intensity 与 XYZ 点数不一致")

    finite_xyz = np.isfinite(xyz).all(axis=1)
    finite_intensity = (
        np.isfinite(intensity) if intensity is not None else np.ones(len(xyz), dtype=bool)
    )
    finite_keep = finite_xyz & finite_intensity
    all_zero_xyz = np.all(xyz == 0, axis=1)
    keep = finite_keep.copy()
    if zero_policy == "exclude":
        keep &= ~all_zero_xyz

    raw_count = len(xyz)
    zero_count = int(all_zero_xyz.sum())
    counts = {
        "input_point_count": raw_count,
        "nonfinite_xyz_row_count": int((~finite_xyz).sum()),
        "nonfinite_intensity_count": int((~finite_intensity).sum()),
        "nonfinite_row_count_removed": int((~finite_keep).sum()),
        "all_zero_xyz_row_count": zero_count,
        "all_zero_xyz_ratio": zero_count / raw_count if raw_count else 0.0,
        "all_zero_xyz_finite_row_count": int((all_zero_xyz & finite_keep).sum()),
        "all_zero_xyz_row_count_removed": (
            int((all_zero_xyz & finite_keep).sum()) if zero_policy == "exclude" else 0
        ),
        "remaining_point_count": int(keep.sum()),
    }
    return keep, counts


def find_snapshot_metadata(input_path: Path):
    if input_path.parent.name.lower() != "lidar":
        return None, None
    metadata_path = input_path.parent.parent / "metadata.json"
    if not metadata_path.is_file():
        return None, None

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    for cloud in metadata.get("clouds", []):
        if Path(cloud.get("file", "")).name == input_path.name:
            return metadata_path, cloud
    return metadata_path, None


def summarize_vector(values: np.ndarray):
    if values.size == 0:
        return None
    return {
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def process_cloud(
    input_path: Path,
    output_dir: Path,
    output_name: str,
    zero_policy: str,
):
    if not input_path.is_file():
        raise FileNotFoundError(input_path)

    xyz, intensity, source_fields = read_cloud_data(input_path, verbose=True)
    if intensity is None:
        raise ValueError(
            "NPZ 中未找到有名称依据的 intensity 字段；拒绝把未知第 4 列当作强度"
        )

    xyz = np.asarray(xyz)
    intensity = np.asarray(intensity).reshape(-1)
    keep, counts = filter_point_rows(xyz, intensity, zero_policy)

    clean_xyz = xyz[keep]
    clean_intensity = intensity[keep]
    remaining_count = counts["remaining_point_count"]
    if remaining_count == 0:
        raise ValueError("过滤后没有点，未写出结果文件")

    # Canonical output schema: N×4 numeric points plus explicit column names.
    clean_points = np.column_stack((clean_xyz, clean_intensity))
    output_dir.mkdir(parents=True, exist_ok=True)
    clean_npz_path = output_dir / f"{output_name}_clean.npz"
    ply_path = output_dir / f"{output_name}_cloudcompare.ply"
    report_path = output_dir / f"{output_name}_processing.json"

    np.savez_compressed(
        clean_npz_path,
        points=clean_points,
        fields=np.asarray(["x", "y", "z", "intensity"]),
    )

    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(clean_xyz.astype(np.float64, copy=False))
    clean_intensity_float = clean_intensity.astype(np.float64, copy=False)
    intensity_range = summarize_vector(clean_intensity_float)
    low = intensity_range["min"]
    high = intensity_range["max"]
    if high > low:
        gray = np.clip((clean_intensity_float - low) / (high - low), 0.0, 1.0)
    else:
        gray = np.zeros_like(clean_intensity_float)
    pcd.colors = o3d.utility.Vector3dVector(np.repeat(gray[:, None], 3, axis=1))
    if not o3d.io.write_point_cloud(str(ply_path), pcd, write_ascii=False):
        raise RuntimeError(f"PLY 写入失败: {ply_path}")

    metadata_path, cloud_record = find_snapshot_metadata(input_path)
    finite_xyz_values = clean_xyz.astype(np.float64, copy=False)
    frame_id = cloud_record.get("frame_id") if cloud_record else None
    report = {
        "schema_version": 1,
        "processed_utc": datetime.now(timezone.utc).isoformat(),
        "input_file": str(input_path.resolve()),
        "snapshot_metadata_file": str(metadata_path.resolve()) if metadata_path else None,
        "frame_id_from_snapshot_metadata": frame_id,
        "source_npz_fields": source_fields,
        **counts,
        "parameters": {
            "zero_xyz_policy": zero_policy,
            "nonfinite_policy": "remove row if any XYZ or intensity value is NaN or Inf",
            "zero_xyz_definition": "x == 0 and y == 0 and z == 0 exactly",
            "coordinate_transform": "none",
            "ground_or_obstacle_filter": "none",
            "coordinate_units": "not declared; values preserved without unit conversion",
        },
        "remaining_xyz_min": finite_xyz_values.min(axis=0).astype(float).tolist(),
        "remaining_xyz_max": finite_xyz_values.max(axis=0).astype(float).tolist(),
        "remaining_intensity_range": intensity_range,
        "ply_visualization": {
            "color": "grayscale normalized linearly over remaining intensity min/max",
            "normalization_intensity_min": low,
            "normalization_intensity_max": high,
            "intensity_is_preserved_exactly_in_clean_npz": True,
        },
        "outputs": {
            "clean_npz": str(clean_npz_path.resolve()),
            "cloudcompare_ply": str(ply_path.resolve()),
            "processing_report": str(report_path.resolve()),
        },
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(
        description="清理单帧 Snapshot NPZ，并输出标准化 NPZ、CloudCompare PLY 和处理记录。"
    )
    parser.add_argument("input", type=Path, help="输入 NPZ 文件")
    parser.add_argument(
        "--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR,
        help=f"输出目录（默认 {DEFAULT_OUTPUT_DIR}）",
    )
    parser.add_argument(
        "--name", default=None,
        help="输出文件前缀；省略时使用父 Snapshot 目录名和输入文件名",
    )
    parser.add_argument(
        "--zero-policy", choices=("exclude", "keep"), default="exclude",
        help="全零 XYZ 行处理方式：默认 exclude；keep 会保留在几何输出中",
    )
    args = parser.parse_args()

    if args.name:
        output_name = args.name
    elif args.input.parent.name.lower() == "lidar":
        output_name = f"{args.input.parent.parent.name}_{args.input.stem}"
    else:
        output_name = args.input.stem

    process_cloud(args.input, args.output_dir, output_name, args.zero_policy)


if __name__ == "__main__":
    main()
