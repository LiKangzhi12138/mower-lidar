
from pathlib import Path
import argparse

import numpy as np
import open3d as o3d
import pypatchworkpp as pyp


def split_ground_indices(xyz, intensity, overrides):
    """Run Patchwork++ with explicit experimental geometry assumptions.

    Indices refer to input rows. Points unclassified by Patchwork++ are
    deliberately left out of the ground set by callers.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("XYZ must be N x 3")
    if not np.isfinite(xyz).all():
        raise ValueError("Patchwork++ input must be finite")
    if "sensor_height" not in overrides:
        raise ValueError("Patchwork++ requires explicit experimental sensor_height")
    if not overrides.get("assume_z_up_experimental", False):
        raise ValueError("Patchwork++ requires assume_z_up_experimental=true")
    params = pyp.Parameters()
    params.verbose = False
    params.enable_RNR = False
    params.intensity_thr = 0.0
    allowed = {
        "sensor_height", "min_range", "max_range", "enable_RNR",
        "enable_RVPF", "enable_TGR", "num_iter", "num_lpr",
        "num_min_pts", "th_seeds", "th_dist", "uprightness_thr",
        "adaptive_seed_selection_margin", "RNR_ver_angle_thr",
        "RNR_intensity_thr", "intensity_thr", "num_zones",
    }
    for key, value in overrides.items():
        if key == "assume_z_up_experimental":
            continue
        if key not in allowed:
            raise ValueError(f"Unsupported Patchwork++ parameter: {key}")
        setattr(params, key, value)
    if not np.isfinite(params.sensor_height) or params.sensor_height <= 0:
        raise ValueError("sensor_height must be positive and finite")
    if not 0 <= params.min_range < params.max_range:
        raise ValueError("Patchwork++ range limits invalid")
    used_placeholder_intensity = intensity is None
    if intensity is None:
        if params.enable_RNR:
            raise ValueError("Patchwork++ RNR requires real intensity")
        intensity = np.zeros(len(xyz), dtype=np.float32)
    else:
        intensity = np.asarray(intensity, dtype=np.float32).reshape(-1)
        if len(intensity) != len(xyz) or not np.isfinite(intensity).all():
            raise ValueError("Patchwork++ intensity invalid")
    effective = {
        key: getattr(params, key) for key in dir(params) if not key.startswith("_")
    }
    effective["assume_z_up_experimental"] = True
    effective["intensity_source"] = "placeholder" if used_placeholder_intensity else "input"
    if len(xyz) == 0:
        empty = np.empty(0, dtype=np.int64)
        return empty, empty, empty, effective
    cloud = np.column_stack((xyz, intensity)).astype(np.float32, copy=False)
    engine = pyp.patchworkpp(params)
    engine.estimateGround(cloud)
    ground = np.unique(np.asarray(engine.getGroundIndices(), dtype=np.int64))
    nonground = np.unique(np.asarray(engine.getNongroundIndices(), dtype=np.int64))
    for indices in (ground, nonground):
        if len(indices) and (indices.min() < 0 or indices.max() >= len(xyz)):
            raise RuntimeError("Patchwork++ returned out-of-range indices")
    if np.intersect1d(ground, nonground).size:
        raise RuntimeError("Patchwork++ returned overlapping ground/nonground indices")
    unclassified = np.setdiff1d(
        np.arange(len(xyz), dtype=np.int64), np.union1d(ground, nonground)
    )
    return ground, nonground, unclassified, effective


def make_cloud(points, color):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(
        np.asarray(points, dtype=np.float64).reshape(-1, 3)
    )
    cloud.paint_uniform_color(color)
    return cloud


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        default=r"outputs\startup_check\cloud_00_colored.ply"
    )
    parser.add_argument(
        "--sensor-height", type=float, required=True
    )
    parser.add_argument(
        "--min-range", type=float, default=0.5
    )
    args = parser.parse_args()

    # 1. 读取点云
    source = Path(args.input)
    if not source.is_file():
        raise FileNotFoundError(source)

    cloud = o3d.io.read_point_cloud(str(source))
    xyz = np.asarray(cloud.points, dtype=np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]

    if len(xyz) < 100:
        raise ValueError("有效点数过少，无法进行可靠的流程测试")

    print("原始有效点数:", len(xyz))
    print("XYZ最小值:", xyz.min(axis=0))
    print("XYZ最大值:", xyz.max(axis=0))

    # 2. 配置 Patchwork++
    params = pyp.Parameters()
    params.verbose = False
    params.sensor_height = args.sensor_height
    params.min_range = args.min_range

    # PLY 没有保留真实回波强度，关闭依赖强度的 RNR
    params.enable_RNR = False

    algorithm = pyp.patchworkpp(params)

    # 官方示例采用 N x 4 输入
    # 第四列这里只是占位值，不代表真实 intensity
    input_points = np.zeros((len(xyz), 4), dtype=np.float32)
    input_points[:, :3] = xyz

    # 3. 地面分离
    algorithm.estimateGround(input_points)

    ground = np.asarray(
        algorithm.getGround(), dtype=np.float64
    ).reshape(-1, 3)

    nonground = np.asarray(
        algorithm.getNonground(), dtype=np.float64
    ).reshape(-1, 3)

    print("地面点数量:", len(ground))
    print("非地面候选点数量:", len(nonground))
    print(
        "未归入上述两类的点数:",
        len(xyz) - len(ground) - len(nonground)
    )

    # 4. 颜色编码
    # 绿色 = 算法判定地面
    # 红色 = 算法判定非地面
    green = make_cloud(ground, [0.1, 0.85, 0.2])
    red = make_cloud(nonground, [0.95, 0.15, 0.1])

    combined = green + red

    # 5. 输出结果
    output_dir = Path(r"outputs\startup_check\patchworkpp")
    output_dir.mkdir(parents=True, exist_ok=True)

    outputs = {
        "ground.ply": green,
        "nonground.ply": red,
        "segmented.ply": combined,
    }

    for filename, result in outputs.items():
        if not o3d.io.write_point_cloud(
            str(output_dir / filename), result
        ):
            raise RuntimeError(f"保存失败: {filename}")

    print("结果保存至:", output_dir.resolve())


if __name__ == "__main__":
    main()
