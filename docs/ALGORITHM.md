# 主路线算法与实际参数

本文描述当前 [主配置](../configs/r3_2_ransac_normal_trial.json)和代码的真实行为；所有长度阈值先按 **NPZ 原始坐标数值**理解，米制仅为上游保持原厂单位的实验假设。

## 1. 读取与清理

`read_cloud_data()` 按 `fields` 找 x/y/z/intensity，返回 XYZ、强度和字段名。`filter_point_rows()` 删除任一 XYZ 或 intensity 为 NaN/Inf 的整行；主配置 `zero_policy=exclude`，再排除 XYZ 三轴严格等于零的行。原 NPZ 只读；保留每个剩余点的原输入行号。全零行究竟是无回波占位仍待上游确认。

## 2. 实验安装姿态

`analysis_coordinates()` 使用显式 `coordinates.mode=experimental_mount`，将点云绕传感器 +Y 轴按右手规则旋转 **+5°**，再沿假设的分析 +Z 方向平移 **0.415**：

```text
p_analysis = R_y(+5°) p_sensor + (0, 0, 0.415)
R_y(θ) = [[cosθ, 0, sinθ], [0, 1, 0], [-sinθ, 0, cosθ]]
```

这是对 `sample_000142` 的下倾 5° 和 +X 指车头、+Y 左、+Z 上的实验模型，不是已标定外参。高度相对于 Mid-360 内部坐标原点的测量参考尚不确定；143～149 只是共享安装假设。原始雷达坐标未被覆盖。

## 3. 法向约束 RANSAC

`normal_constrained_plane()` 最多尝试 **6** 个优势平面；每次用 Open3D RANSAC，距平面阈值 `0.12`、最小采样点数 `3`、迭代 `1000`、成功概率 `0.999`、初始随机种子 `20261023`（下一次试验种子加 1）。候选平面 `ax+by+cz+d=0` 需同时满足法向与配置竖直轴夹角 **≤15°**、沿竖直轴的绝对截距 **≤0.2**。被拒绝的优势平面从后续*搜索集合*暂时移开，但不从最终非地面候选删除；只有符合条件的平面索引被移入地面候选。若无合格平面，地面候选为空，点保留给聚类。配置和结果中均记录被拒绝平面诊断。

因此不能把最大平面直接当地面：墙体等高大连续结构可能成为最大平面。即便满足几何法向约束，仍需人工真值核查，输出字段 `ground_note` 明确标为实验候选。

## 4. DBSCAN 与三维几何

`cluster_dbscan()` 复用 Open3D，固定 `eps=0.35`、`min_points=12`。其输出为每个非地面点一个标签；`-1` 为未聚类点，不等于无障碍。当前没有额外高度、尺寸或语义类别过滤，也没有把簇数当真实障碍数。`candidate_region.ply` 和 `nonground.ply` 保留整个非地面区域，包括连续墙面和噪声。

每个非空簇 `P`：中心 `c=mean(P)`；轴对齐包围盒最小/最大坐标逐轴取 `min/max`，`bbox_extent_xyz=max-min`；`nearest_sensor_distance=min ||p_sensor||₂`，按**变换前**的原始雷达原点和簇中每个点计算，并记录对应原 NPZ 行号。`center_xyz` 和包围盒在实验分析坐标内，不能把中心距离称为最近点距离。

## 5. 二维方位与雷达图

`radar_view.xy_polar()` 对分析坐标系的中心 `c` 计算 `range_xy=hypot(c_x,c_y)`、`azimuth_deg=degrees(atan2(c_y,c_x))`；对簇点算 `nearest_range_xy=min hypot(p_x,p_y)`。中心恰在 XY 原点时约定方位值为 `0.0`，同时 `azimuth_defined=false`。图屏幕映射是 `(-Y,+X)`，+X 朝上、+Y 朝左，使用等比例坐标轴；自动距离环按已聚类点最远 XY 范围选取，也可由独立 `radar_view.py` 的参数覆盖。图中灰色表示可见范围内的未聚类点。

二维距离是分析坐标 XY 原点距离；三维 `nearest_sensor_distance` 是原始雷达原点距离。实验变换只沿 Z 平移，所以两者 XY 原点一致，但不能混用数值。NPZ 单位转换链、车辆真实前向和高度参考点未验收，方位只相对于实验坐标系 +X。

历史 Patchwork++、无约束 RANSAC、简单高度过滤及欧式聚类见 [架构文档](PROJECT_ARCHITECTURE.md)与原阶段报告；它们不改变当前固定主参数。
