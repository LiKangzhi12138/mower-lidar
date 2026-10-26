# R3 Mid-360 离线障碍候选原型

日期：2026-10-22。输出均为**实验性障碍候选**，没有地面或障碍物真值，不能用于停车、车辆坐标距离或检出率判断。

## 1. 实现与复用

- `scripts/offline_detector.py`：统一命令行入口和可独立调用的 `run()`、`analysis_coordinates()`、`filter_height_indices()`、`euclidean_clusters()`。依次读取/预处理、可选坐标实验变换、可选平面或地面处理、聚类、候选描述与导出。
- 复用 `scripts/npz_to_ply.py:read_cloud_data()` 读取 NPZ，`scripts/prepare_cloud.py:filter_point_rows()` 按 R1 规则过滤非有限 XYZ/intensity 与可选全零点；保留输入文件点行索引。为边界测试，读取器现也接受带明确 `fields` 的空/少量点 `points` 矩阵。
- 复用 `scripts/extract_obstacle_candidates.py` 的 Open3D RANSAC、DBSCAN、PLY/三视角预览和候选几何描述。共用函数已抽出，可由 R2 原流程继续调用。
- `scripts/patchwork_ground.py:split_ground_indices()` 复用既有 Patchwork++ 绑定，要求显式传入实验性传感器高度和 Z 向假设，使用真实 NPZ intensity，并返回对应输入行的 ground、nonground、unclassified 索引。未分类点保留在候选区域。运行记录保存全部绑定生效参数。
- 新增基于 SciPy `cKDTree` 半径邻接连通分量的欧式聚类。DBSCAN 与欧式聚类均允许单独选择；`clustering.method=none` 用于仅导出候选区域。

配置示例位于 `configs/r3_*.json`。`ground.method` 可为 `none`、`ransac`、`patchwork`、`height`；`clustering.method` 可为 `none`、`dbscan`、`euclidean`。全部试验阈值以**未确认单位的坐标数值**给出。RANSAC 默认 `plane_policy=retain`，只标记主平面；只有显式设置 `exclude_experimental` 才会从候选区域排除。主平面没有地面语义。高度基线必须显式配置分析坐标中的竖直方向、参考值和间隔；示例的 `[0,0,1]` 仅为试跑假设，不能解释为实际离地高度。

坐标默认 `mode=raw_sensor`，不做变换。安装高度、俯仰角和坐标单位默认均未知；原始模式若填入高度或俯仰角，会报错，避免记录了参数却未使用。需要实验变换时可设置 `mode=experimental_mount`，并同时提供 `mount_height`、`mount_pitch_degrees`、`pitch_axis_sensor`、`vertical_axis_analysis` 和 `analysis_frame`。变换使用右手轴角旋转，再沿指定分析坐标竖直方向平移；方向与数值由使用者明确给出，不自动采用 DRAFT TF。最近点距离始终按**变换前的雷达原点**计算。

## 2. 输入、输出与复现

本次使用 R1 四个 `*_clean.npz`，输入文件未改变。每次运行目录位于 `outputs/r3_offline_detector/<run-name>/`，包含：

- `result.json`：输入绝对路径和 SHA-256、源码字段、完整配置、Patchwork++ 生效参数、软件版本、分析/传感器参考坐标、单位确认状态、逐阶段耗时、过滤/平面/候选点数和逐候选统计。
- `indices_and_labels.npz`：相对于**本次输入 NPZ**的清理后、候选区域、实验性排除及平面候选索引，以及候选区域逐点聚类标签（`-1` 表示未聚类）。对于 R1 clean NPZ，这些索引并非原始 Snapshot NPZ 的点行索引。
- `candidate_region.ply`：所有未被实验性过滤的点，**包括聚类噪声与墙面等连续结构**；`clusters_colored.ply` 用不同颜色标注各簇，灰色为未聚类点；`candidate_*.ply` 为逐簇点云；若非空，还会有 `plane_candidate.ply`、`removed_experimental.ply`；`clusters_preview.png` 为三视角投影。空类别不写 PLY，因为 Open3D 不能重读零顶点 PLY。

逐候选提供 `center_xyz`、轴对齐包围盒最小/最大坐标及跨度、`nearest_sensor_xyz`、`nearest_sensor_distance` 和对应输入索引。中心/包围盒属于 `analysis_frame`；距离属于原始雷达原点。`coordinate_unit_confirmed=false` 时，距离标为 `unconfirmed_coordinate_values`，绝不作米或车辆坐标距离解释。每个候选均标注 `experimental_obstacle_candidate`。

使用现有 `mower-lidar` 环境，从工程根目录运行，例如：

```powershell
python -B scripts/offline_detector.py `
  --input outputs/r1_input_prep/batch1_sample_000001_clean.npz `
  --config configs/r3_raw_dbscan.json `
  --run-name batch1_sample_000001_raw_dbscan
python -B scripts/check_offline_detector.py
```

新 Snapshot 可直接给 `--input` 指向 NPZ；读取器支持既有字段布局，R1 过滤规则仍会执行。更建议先运行 R1 的 `prepare_cloud.py` 并保留其清理记录。为新采集姿态另建 JSON 配置并显式记下来源与假设；不要直接沿用本报告的实验高度/竖直轴。

## 3. 实际运行结果

同一帧的四种方法均读取相同 R1 clean 文件，SHA-256 和清理后点数相同。表中“排除”只是相应实验策略的结果，不代表真实地面；簇数不代表真实障碍物数量。总耗时包括读取、处理、PNG/PLY/NPZ 写入，不含最后的输入 SHA-256 计算与 JSON 写入；这是 Windows 本机单次离线试跑，不能推算 ROS 2 实时性能。

| 运行名（目录名） | R1 输入点 | 平面候选 | 实验排除 | 候选区域点 | 聚类方法/簇数 | 未聚类点 | 总耗时约 s |
| --- | ---: | ---: | ---: | ---: | --- | ---: | ---: |
| `batch1_sample_000001_raw_dbscan` | 18,279 | 0 | 0 | 18,279 | DBSCAN / 6 | 44 | 1.20 |
| `batch1_sample_000016_raw_dbscan` | 17,735 | 0 | 0 | 17,735 | DBSCAN / 7 | 31 | 1.22 |
| `batch2_sample_000001_raw_dbscan` | 6,476 | 0 | 0 | 6,476 | DBSCAN / 16 | 3,128 | 0.85 |
| `batch2_sample_000055_raw_dbscan` | 13,238 | 0 | 0 | 13,238 | DBSCAN / 84 | 6,140 | 1.00 |
| `batch1_sample_000001_ransac_euclidean` | 18,279 | 4,921 | 0 | 18,279 | 欧式 / 4 | 24 | 1.46 |
| `batch1_sample_000001_patchwork_dbscan` | 18,279 | 0 | 117 | 18,162 | DBSCAN / 6 | 39 | 1.22 |
| `batch1_sample_000001_height_dbscan` | 18,279 | 0 | 4,816 | 13,463 | DBSCAN / 8 | 54 | 1.05 |

四帧 R1 文件本身分别来自 20,160、19,488、19,968、20,064 行 Snapshot 输入；R1 分别排除 1,881、1,753、13,492、6,826 个全零点，非有限行均为 0。本阶段在 R1 clean 输入上再次应用相同过滤规则，没有额外删点。

本次配置：原始几何与三种过滤试跑均用 DBSCAN `eps=0.35`、`min_points=12`；RANSAC 用阈值 `0.05`、三点采样、1,000 次、概率 `0.999`、随机种子 `20261015`，并**保留**平面；欧式聚类半径 `0.35`、最少 12 点。Patchwork++ 试跑显式假设第三轴向上、`sensor_height=1.0`、范围 `0.1..50.0`，关闭 RNR/RVPF/TGR；其余绑定默认值和实际生效值记于 JSON。高度基线试跑使用 `[0,0,1]`、参考值 `0.0`、间隔 `0.2`。这些数字均为未确认单位的实验参数，特别是 `1.0` **不表示已测得的安装高度**。

可从 [RANSAC/欧式候选预览](../outputs/r3_offline_detector/batch1_sample_000001_ransac_euclidean/clusters_preview.png)、[Patchwork++ 候选预览](../outputs/r3_offline_detector/batch1_sample_000001_patchwork_dbscan/clusters_preview.png) 和对应 PLY 做三维人工审查。第一批帧在大区域上呈现少数连通簇，第二批 `sample_000055` 在同样阈值下有大量未聚类点和碎片簇；这只说明采样/场景/参数相互作用明显。R2.5 已见主平面方向随阈值切换，不能将 R3 平面或 Patchwork++ 输出直接认定为真实地面，也不能根据簇数判断哪种方法更准。

## 4. 功能检查

`scripts/check_offline_detector.py` 在既有环境运行通过：

- 重读 7 次真实试跑的非空 PLY 与 PNG；核对点数、颜色数、候选区域原坐标、输入索引分区、每簇点云坐标、中心、包围盒及原点最近距离。
- 核对同一帧四种方法的输入 SHA-256 和预处理后点数一致。
- 临时构造并删除 6 种小 NPZ：空点云、空点云调用 Patchwork++、全零点、点数不足以拟合平面、无聚类、NaN/Inf；均得到合理点数或空候选，不向原始数据目录写入。
- 单独检查显式安装变换、欧式聚类连通分量和高度过滤边界的数学结果。未对真实帧应用实验变换。

## 5. 限制与后续工作

R0/R1 只确认 Snapshot 元数据中的 `livox_frame` 和部分时间字段；静态 TF 是 DRAFT。真实安装高度、0°/-5°/-10° 与各帧对应关系、坐标单位/轴方向、时间同步和全零点语义均未验收。Patchwork++ 依赖高度与竖直方向；本次仅证明绑定可运行、索引可回溯，117 个“地面”点没有准确性含义。其绑定默认 `elevation_thr`/`flatness_thr` 为零数组，相关行为也需在有标定和标签的数据上核对。

`candidate_region.ply` 保留连续墙面和未聚类点，但候选列表仍由连通簇生成，墙面可能与其他结构合并，也可能被切碎。三视角 PNG 无法代替三维旋转检查。当前四帧没有目标草地障碍实例/逐点地面真值；不能给出检出率、误报率、尺寸误差或方法优劣。下一阶段需确认导出单位和零点语义，实测各采集批次/姿态的安装外参与传感器离地高度，并提供目标机场草地场景与少量地面/障碍物标注。随后固定同一输入及参数，评估真实地面点保留/障碍点误删、簇拆分/合并、距离和包围盒误差；再决定地面过滤和候选提取路线。ROS 2 实时节点、车辆决策和控制仍不在本原型内。
