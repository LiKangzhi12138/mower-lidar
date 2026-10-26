# R3.4 障碍候选二维方位与雷达图

日期：2026-10-26。主路线固定为**法向约束 RANSAC + DBSCAN**；本轮没有运行 Patchwork++，没有修改 RANSAC、DBSCAN 参数或原始 NPZ。输入复用 R3.2 的 `sample_000142` 和 R3.3 的 `sample_000143`～`sample_000149` 既有 `ransac_normal/result.json`、源索引文件与彩色 PLY。

## 新增文件与复用方式

- `scripts/radar_view.py`：独立的 XY 几何计算、绘图与命令行导出模块。它使用原 `read_cloud_data()`、R1 `filter_point_rows()`、`offline_detector.analysis_coordinates()` 和既有 RANSAC+DBSCAN 标签；没有重做地面分离或聚类。
- `scripts/check_r3_radar_view.py`：四方向、原点、几何比例和输出文件一致性检查。
- `outputs/r3_radar_view/sample_000142`～`sample_000149`：各含 `obstacles.json`、`radar_view.png`、`obstacles_colored.ply`。PLY 从对应既有 R3.2/R3.3 结果复制并重读验证，没有重新计算或保存整个数据集。

原 `offline_detector.py` 的调用方式和 `result.json` 未改动；`obstacles.json` 保留原逐候选字段，并附加二维字段。生成时先校验输入 NPZ 的 SHA-256、清理点索引、候选标签和原有彩色 PLY 坐标，避免把不同处理批次的 XYZ、标签和颜色混用。

## 坐标系、距离与方位角

现有 `result.json` 的 `center_xyz`、包围盒和 PLY 坐标属于 `experimental_level_x_forward_y_left_z_up`：R3.2 实验假定雷达 +X 指向车头、+Y 向左、+Z 向上，将原始 `livox_frame` 点绕 +Y 旋转 +5°，再按 0.415 的实验高度沿 +Z 平移。**雷达 +X 的真实车头朝向、0.415 m 到内部原点的偏移和 NPZ 保存转换链尚未正式核实。** 143～149 是否与 142 同安装状态也只有共享假设；R3.3 发现 144～149 的候选平面几何与 143 明显不同。故 `azimuth_deg` 仅表示**相对于当前实验坐标系 +X** 的方位，不称为已标定的车辆前后左右方位；所有长度展示为“米制假设，NPZ 导出单位未核实”。

对于中心 `c=(x,y,z)`，在同一实验坐标系中：

```text
range_xy        = hypot(x, y)                       # 候选中心的水平距离
azimuth_deg     = degrees(atan2(y, x))              # +X=0°, +Y=+90°
nearest_range_xy = min(hypot(point.x, point.y))     # 簇中最近点的水平距离
```

三者均以实验坐标系的 **XY 原点**为参考；该变换只沿 Z 平移，因此水平原点与姿态校正后雷达原点的 XY 一致。`nearest_sensor_distance` 是原 R3 字段，含义仍为**原始 `livox_frame` 雷达原点到最近簇点的三维距离**。它与 `range_xy`、`nearest_range_xy` 不应混称。候选中心恰落 XY 原点时，约定 `azimuth_deg=0.0`、`azimuth_defined=false`，避免把数学上未定义的方向当成确定方位。

`obstacles.json` 的候选仍以原 DBSCAN 标签 `id` 为编号（从 0 开始）；新增 `obstacle_id` 与 `id` 相同。`coordinate_frame`、`angle_reference` 同时写在每个候选和顶层元数据中。顶层还记录 `coordinate_unit_confirmed=false`、实验姿态参数、单位限制、二维距离原点、旧三维距离参考、源结果路径及哈希。每个候选继续含 `center_xyz`、`bbox_min_xyz`、`bbox_max_xyz`、`bbox_extent_xyz`、`nearest_sensor_distance`、`nearest_input_index`、`point_count`、`input_indices` 等旧字段；新增 `range_xy`、`nearest_range_xy`、`azimuth_deg`、`azimuth_defined`、`coordinate_frame`、`angle_reference`。例如 142 的 **ID 0** 实测中心约 `[3.981,-0.686,0.784]`，中心水平距离约 `4.039`、方位 `-9.781°`、最近水平点距约 `3.923`；旧三维雷达原点最近距离约 `3.948`。数字均受上述单位与外参假设限制。

## 雷达图设计

绘图使用现有 Matplotlib，屏幕坐标为 `horizontal=-analysis_Y`、`vertical=analysis_X`，因此 +X 在上、+Y 在左。设置 `aspect='equal'`、X/Y 共用半径，未拉伸任何轴；同心环为实验 XY 原点距离。默认根据**已聚类点**的最远水平距离加 10% 边距选择可读的圆环半径和间距，也支持 `--max-radius` 与 `--ring-step`。彩色投影、中心圆标和 `#id` 与三维 PLY 使用同一 DBSCAN 标签及颜色函数；右侧列表显示每簇编号、中心 XY、中心水平距离、方位角。`--show-boxes` 可额外画包围盒的 XY 投影。灰色 DBSCAN 未聚类点默认只显示圆环范围内的部分，图上注明显示数/总数；`--hide-noise` 可关闭。完整灰色点依然保存在原 R3 非地面和彩色 PLY 中。

主图：[sample_000142/radar_view.png](../outputs/r3_radar_view/sample_000142/radar_view.png)。本帧自动环为 `5/10/15` 个实验坐标单位，18 个彩色候选簇的点全部在绘图区内；灰色未聚类点显示 848/1,961。请在演示时保留图上的 `EXPERIMENTAL` 与单位说明，不把画面上方直接称为真实车头。

## 实际输出与运行

统一目录：`E:\workspace\mower-lidar\outputs\r3_radar_view\`。每行对应三个同名文件 `obstacles.json`、`radar_view.png`、`obstacles_colored.ply`；下表直接链接 PNG 与 JSON：

| 样本 | 彩色候选数 | 自动绘图半径 | 雷达图 | 数值结果 |
| --- | ---: | ---: | --- | --- |
| 142 | 18 | 15 | [PNG](../outputs/r3_radar_view/sample_000142/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000142/obstacles.json) |
| 143 | 18 | 15 | [PNG](../outputs/r3_radar_view/sample_000143/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000143/obstacles.json) |
| 144 | 5 | 15 | [PNG](../outputs/r3_radar_view/sample_000144/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000144/obstacles.json) |
| 145 | 9 | 15 | [PNG](../outputs/r3_radar_view/sample_000145/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000145/obstacles.json) |
| 146 | 6 | 15 | [PNG](../outputs/r3_radar_view/sample_000146/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000146/obstacles.json) |
| 147 | 6 | 15 | [PNG](../outputs/r3_radar_view/sample_000147/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000147/obstacles.json) |
| 148 | 10 | 15 | [PNG](../outputs/r3_radar_view/sample_000148/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000148/obstacles.json) |
| 149 | 4 | 10 | [PNG](../outputs/r3_radar_view/sample_000149/radar_view.png) | [JSON](../outputs/r3_radar_view/sample_000149/obstacles.json) |

在工程根目录、已有 `mower-lidar` 环境下生成 142：

```powershell
python -B scripts/radar_view.py --result-json outputs/r3_ground_comparison/sample_000142_final/ransac_normal/result.json --output-dir outputs/r3_radar_view/sample_000142
python -B scripts/check_r3_radar_view.py --samples 142
```

143～149 使用已生成的 R3.3 `ransac_normal` 结果批量生成，再统一检查：

```powershell
foreach ($n in 143..149) {
  $sample = 'sample_{0:D6}' -f $n
  python -B scripts/radar_view.py --result-json (Join-Path (Join-Path 'outputs/r3_multiscene_validation' $sample) 'ransac_normal/result.json') --output-dir (Join-Path 'outputs/r3_radar_view' $sample)
  if ($LASTEXITCODE -ne 0) { throw "Radar export failed for $sample" }
}
python -B scripts/check_r3_radar_view.py --samples 142 143 144 145 146 147 148 149
```

## 验证结果与边界

**全部通过。** 检查包含：四个标准方向 `(±X,±Y)` 分别得到 `0°/±90°/±180°`，XY 原点角度约定与定义标记，最近水平距离和中心水平距离的区分；`screen=(-Y,X)` 的朝向与模长保持；自动环有序且覆盖彩色点；8 帧输入哈希、旧三维候选所有字段不变、DBSCAN 标签与 `id/obstacle_id`、候选中心和逐点二维最小距离重算一致；复制的 PLY 坐标与颜色经 Open3D 重读一致；8 张 PNG 可打开且尺寸有效，绘图实现采用等比例轴。142 的图已人工打开检查，另抽看 149 的图。检查脚本输出 `status=passed`。

图上的颜色只对应**空间聚类候选**。没有人工障碍物真值、可靠逐帧外参或精确 NPZ 单位证据，因此不能判断候选数、位置、尺寸和真实障碍之间的准确关系；R3.3 对 144～149 地面分离异常的提醒继续适用。二维图也不代表 Livox 原始回波，更不能作为车辆停车或规划依据。
