
from pathlib import Path

import numpy as np


# 原始数据路径
input_file = Path(
    r"E:\workspace\mower-data\airport_mower_samples"
    r"\sample_000001\lidar\cloud_00.npz"
)

# 转换结果路径
output_file = Path(
    r"E:\workspace\mower-lidar\outputs"
    r"\startup_check\cloud_00_colored.ply"
)


def read_cloud_data(path, verbose=True):
    with np.load(path, allow_pickle=False) as data:
        if verbose:
            print("NPZ 内部字段：")

            for name in data.files:
                try:
                    arr = data[name]
                    print(name, arr.shape, arr.dtype)
                except ValueError as e:
                    print(name, "无法直接读取：", e)

        field_names = []
        if "fields" in data.files:
            field_names = [str(value) for value in np.asarray(data["fields"]).ravel()]

        xyz = None
        intensity = None

        # 情况一：分别保存 x、y、z
        if all(k in data.files for k in ("x", "y", "z")):
            axes = [np.asarray(data[k]).ravel() for k in ("x", "y", "z")]
            if len({axis.size for axis in axes}) != 1:
                raise ValueError("x、y、z 字段的元素数量不一致")
            xyz = np.column_stack(axes)
            if "intensity" in data.files:
                intensity = np.asarray(data["intensity"]).ravel()

        # 情况二：Nx3 / Nx4 或结构化数组
        candidates = ["xyz", "points", "cloud", "pointcloud"] + list(data.files)

        if xyz is None:
            for name in dict.fromkeys(candidates):
                if name not in data.files:
                    continue

                try:
                    arr = data[name]
                except ValueError:
                    continue

                if arr.dtype.names:
                    names = {field.lower(): field for field in arr.dtype.names}
                    if all(axis in names for axis in ("x", "y", "z")):
                        xyz = np.column_stack([
                            arr[names[axis]].ravel() for axis in ("x", "y", "z")
                        ])
                        if "intensity" in names:
                            intensity = np.asarray(arr[names["intensity"]]).ravel()
                        field_names = list(arr.dtype.names)
                        break

                if (
                    arr.ndim == 2
                    and (arr.shape[0] >= 10 or (name == "points" and field_names))
                    and arr.shape[1] in (3, 4, 5, 6)
                    and np.issubdtype(arr.dtype, np.number)
                ):
                    # PointCloud2 转出的常见布局同时保存 points 和 fields；
                    # 按字段名取坐标与强度，避免依赖列顺序。
                    if name == "points" and field_names:
                        indices = {field.lower(): i for i, field in enumerate(field_names)}
                        if all(axis in indices for axis in ("x", "y", "z")):
                            xyz = arr[:, [indices[axis] for axis in ("x", "y", "z")]]
                            if "intensity" in indices:
                                intensity = arr[:, indices["intensity"]]
                            break

                    xyz = arr[:, :3]
                    break

        if xyz is None:
            raise ValueError("未自动识别 XYZ，请根据打印字段确认 NPZ 格式")

        if intensity is None and "intensity" in data.files:
            values = np.asarray(data["intensity"])
            if np.issubdtype(values.dtype, np.number):
                intensity = values.ravel()

        if intensity is not None and intensity.size != xyz.shape[0]:
            raise ValueError(
                f"intensity 元素数量 {intensity.size} 与 XYZ 点数 {xyz.shape[0]} 不一致"
            )

        return xyz, intensity, field_names


def read_cloud(path, verbose=True):
    """Return XYZ using the shared NPZ parser; kept for existing callers."""
    xyz, _intensity, _field_names = read_cloud_data(path, verbose=verbose)
    return xyz


def main():
    import matplotlib.pyplot as plt
    import open3d as o3d

    # 1. 读取点云
    points = read_cloud(input_file)
    points = np.asarray(points, dtype=np.float64)

    # 2. 去除无效点
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]

    if len(points) == 0:
        raise ValueError("没有有效的三维坐标")

    print("有效点数：", len(points))
    print("坐标最小值：", points.min(axis=0))
    print("坐标最大值：", points.max(axis=0))

    # 3. 生成颜色；这里仍暂以第三坐标轴着色，不代表已标定的重力高度。
    z = points[:, 2]

    low, high = np.percentile(z, [2, 98])

    if high <= low:
        normalized = np.zeros_like(z)
    else:
        normalized = np.clip(
            (z - low) / (high - low), 0, 1
        )

    colors = plt.get_cmap("turbo")(normalized)[:, :3]

    # 4. 构造 Open3D 点云
    pcd = o3d.geometry.PointCloud()

    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # 5. 保存成 PLY
    output_file.parent.mkdir(parents=True, exist_ok=True)

    success = o3d.io.write_point_cloud(
        str(output_file), pcd
    )

    if not success:
        raise RuntimeError("PLY 保存失败")

    print("PLY 已保存：", output_file)


if __name__ == "__main__":
    main()
