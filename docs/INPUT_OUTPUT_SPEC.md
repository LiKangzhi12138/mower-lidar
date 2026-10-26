# 输入输出协议（R3.6 离线主入口）

## 输入 NPZ

实测 `sample_000142/lidar/cloud_01.npz` 有 `points: (19200,4) float32` 与 `fields: (4,) <U9`，字段名依次 `x,y,z,intensity`。代码通过 `read_cloud_data()` 依**字段名**取列，不依赖示例的物理列顺序；读取器还兼容独立 x/y/z 数组及结构化 XYZ 字段，但主数据实际采用 `points`+`fields`。`intensity` 保留用于读取/过滤，主流程没有按强度分类。该 NPZ 无逐点时间戳。元数据称 `livox_frame`，但 NPZ 自身不保存可信完整外参；如需按示例配置解释数据，必须另行核实坐标轴、单位和安装状态。

```python
# 形状示意；数值是实际文件布局而非独立的接口承诺
points = np.array([[x, y, z, intensity], ...], dtype=np.float32)
fields = np.array(["x", "y", "z", "intensity"])
```

主配置先删任一 XYZ/intensity 非有限的行，再排除三轴全零的行；原始 NPZ 不改。142 实测 19,200 行、全零 13,534、非有限 0、保留 5,666。`indices_and_labels.npz` 中保存输入行号，因此可以回溯。Livox ROS 驱动上游使用米，但仓库缺本批 NPZ 保存转换链，故 `coordinate_unit_confirmed=false`，输出数值**尚不可作为已核实米值**。

## 主输出目录

| 路径 | 内容与用途 |
| --- | --- |
| `obstacles.json` | 面向调用方的实验候选列表，保留 R3 每簇字段并增加二维信息；**核心数值输出**。 |
| `obstacles_colored.ply` | 完整非地面候选区域；簇彩色、DBSCAN `-1` 灰色；**核心三维输出**。 |
| `radar_view.png` | 同一簇标签的等比例 XY 投影；**核心可视化输出**。 |
| `ground.ply` | 绿色受约束 RANSAC 地面候选，供人工检查，可能为空而不生成。 |
| `nonground.ply` | 含聚类噪声的完整非地面候选区域，灰色，供人工检查。 |
| `detection/result.json` | `offline_detector.run()` 的原始 R3 实验记录、完整参数、耗时和逐簇三维结果；**审计/调试输出**。 |
| `detection/indices_and_labels.npz` | 清理后行、地面/非地面行及聚类标签；审计输出。 |
| `detection/candidate_*.ply` 等 | 逐簇及原处理阶段点云；`point_cloud` 字段相对 `detection/` 解析。 |

`ground.ply` 和 `nonground.ply` 在新入口中从 `offline_detector` 的已生成层映射而来，不增加地面算法。旧 R3.2/R3.3 对比脚本可能另输出 `overview.png`、`unclassified.ply`；主入口不生成 `overview.png`。普通 RANSAC 主配置的未分类点数为 0；Patchwork++ 历史实验可能存在 `unclassified.ply`。空 PLY 层按现有约定不写文件。

## `result.json` 顶层实际字段

`result_type`、`input_file`、`input_sha256`、`source_fields`、`config_file`、`software_versions`、`parameters`、`effective_patchwork_parameters`、`analysis_frame`、`sensor_frame`、`coordinate_unit`、`coordinate_unit_confirmed`、`ground_note`、`ground_method`、`plane_model_ax_by_cz_d`、`plane_candidate_point_count`、`removed_point_count`、`unclassified_point_count`、`rejected_plane_candidates`、`selected_plane_diagnostics`、`candidate_region_point_count`、`noise_point_count`、`candidate_count`、`candidates`、`preprocess_stats`、`timings`、`distance_reference`、`distance_unit`。主路线 `effective_patchwork_parameters=null`；`removed_point_count` 是**实验地面候选点**，不是已验证地面点。`timings` 为该机一次离线处理的分阶段时间，不能推导端到端实时延迟。

## `obstacles.json` 顶层和每簇字段

顶层实际包含 `schema_version`、`result_type`、`sample_name`、`source_result_json`、`input_file`、`input_sha256`、`source_fields`、`preprocess_stats`、`ground_method`、`ground_parameters`、`clustering_parameters`、`coordinate_parameters`、`coordinate_frame`、`sensor_frame`、`coordinate_unit`、`coordinate_unit_confirmed`、`unit_note`、`calibration_status`、`vehicle_forward_sensor_axis_verified`、`mount_height_origin_verified`、`angle_reference`、`range_xy_origin`、`nearest_sensor_distance_reference`、`azimuth_at_xy_origin`、`candidate_count`、`unclustered_point_count`、`candidates`、`radar_plot` 和 `outputs`。

逐簇对象的真实键如下；**代码没有 `size_xyz` 字段**，三轴尺寸是 `bbox_extent_xyz`，不要自行假设别名：

| 字段 | 含义 |
| --- | --- |
| `id`、`obstacle_id` | 同一个从 0 开始的 DBSCAN 标签；是候选 ID，非真实障碍物 ID。 |
| `classification`、`point_count`、`point_cloud`、`input_indices` | 实验候选标签、簇点数、逐簇 PLY 文件名、原输入 NPZ 行号。 |
| `center_xyz` | 分析坐标中簇点算术平均中心 XYZ。 |
| `bbox_min_xyz`、`bbox_max_xyz`、`bbox_extent_xyz` | 分析坐标中的轴对齐包围盒和三轴跨度。 |
| `nearest_sensor_xyz`、`nearest_sensor_distance`、`nearest_input_index` | 原始传感器坐标里距原点最近的簇点、**三维最近点距离**及原 NPZ 行号。 |
| `range_xy`、`nearest_range_xy` | 分析坐标 XY 原点到**中心**、到**簇最近点**的二维水平距离。 |
| `azimuth_deg`、`azimuth_defined` | 中心相对当前分析坐标 +X 的角度；原点按 0° 约定且定义标志为假。 |
| `coordinate_frame`、`angle_reference` | 数值坐标系名称及 +X=0°、+Y=+90° 的约定。 |

以下是实际 142 输出的 ID 0 **字段摘录**，省略了很长的 `input_indices`；摘录不可当完整 JSON 文件替换：

```json
{
  "id": 0,
  "obstacle_id": 0,
  "classification": "experimental_obstacle_candidate",
  "point_count": 232,
  "center_xyz": [3.9805141834524944, -0.6861594827095014, 0.7840962310567998],
  "bbox_min_xyz": [3.8150165345012725, -1.3009999990463257, 0.14210090849183704],
  "bbox_max_xyz": [4.149360495962061, 0.36000001430511475, 1.7431199388034564],
  "bbox_extent_xyz": [0.3343439614607888, 1.6610000133514404, 1.6010190303116194],
  "nearest_sensor_distance": 3.9477599309041804,
  "range_xy": 4.039221212112386,
  "nearest_range_xy": 3.92297681771384,
  "azimuth_deg": -9.780505015747412,
  "azimuth_defined": true,
  "coordinate_frame": "experimental_level_x_forward_y_left_z_up",
  "angle_reference": "+X=0deg; +Y=+90deg; counterclockwise in current XY frame"
}
```

三维最近点距离与二维中心距离、二维最近点距离参考不同；`center_xyz`/包围盒属于**实验校正坐标**，`nearest_sensor_xyz`/`nearest_sensor_distance` 属于**原始雷达坐标**。`coordinate_unit_confirmed=false`，角度仅相对实验 +X，不能解释为已标定车辆方位。`candidate_count` 等于 DBSCAN 候选簇数，不等于真实障碍数量。
