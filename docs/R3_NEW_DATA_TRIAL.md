# R3.1 Mid-360 新数据离线障碍候选实验

日期：2026-10-22。输入仅来自 `E:\workspace\mower-data\data\newdata`。本报告的平面与聚类均为**实验性几何候选**；没有逐点地面/障碍物真值，不能称为已验证的障碍检测，也不能把距离解释为米或车辆坐标系距离。

## 1. 新数据结构、格式与时间

递归检查 `newdata`：`sample_000115`～`sample_000149` 共 35 组 Snapshot、315 个文件。每组有 `lidar/cloud_00.npz`、`cloud_01.npz`、`cloud_02.npz`，另有 `images/image_00.png`、`metadata.json`、`tf_static.json`、`rtk.json`、`health.json`、`imu.csv`。合计 **105 帧 NPZ**、35 张图像。逐帧清单和基础统计保存在 [newdata_manifest.json](../outputs/r3_new_data_trial/newdata_manifest.json)，相机缩略总览为 [snapshot_image_contact_sheet.png](../outputs/r3_new_data_trial/snapshot_image_contact_sheet.png)。审查程序为 `scripts/audit_r3_new_data.py`，只读取原始目录。

105 帧均可由现有 `read_cloud_data()` 直接读取：NPZ 中的 `points` 为 N×4 `float32`，`fields` 为 `['x','y','z','intensity']`；元数据中帧名均为 `livox_frame`，声明点数与文件实读点数一致。每帧原始 16,512～22,656 行，总计 2,097,696 行。没有发现 NaN/Inf 行；全零 XYZ 共 1,474,213 行，占总行数约 70.3%，逐帧占比约 61.0%～77.5%。按 R1 默认规则排除全零点后共 623,483 点，每帧 4,215～7,384 点。强度列存在，有限值最小值均为 0，逐帧最大值为 119～161；这些数值的物理标定含义未确认。全零行的采集/导出语义也仍为 UNKNOWN。

全部元数据使用同一个 `session_id=20261021_155135`，`created_utc` 从 2026-10-21 07:51:53Z 至 08:15:32Z；逐帧 `header_stamp_ns` 和 `receive_time_ns` 见清单。`scene_hint` 和 `operator_note` 均为空，故正式场景标签为 **UNKNOWN**；`label_hint=stone` 是采集提示，不是点云障碍真值。相机总览中，前段画面明显过曝；后段画面可见室外铺装区域中的人/推车样结构，以及草地、围栏和树木背景，但不能据此给每帧点云指派物体类别。选中帧的图像与 `cloud_01` 接收时间相差约 -3.01～+2.20 秒，且精确跨传感器同步未验证。

35 组 `tf_static.json` 都记录同一 `base_link -> livox_frame` 平移 `(-0.03,0,0.2)` 和单位四元数。任务背景说明本批混有 0°、-5°、-10° 倾角且高度变化，但元数据没有逐帧对应关系；这份不变的 TF **不能用于确定各帧真实安装参数**。每帧对应倾角、高度、实际坐标轴、坐标单位和车辆外参均记为 UNKNOWN；本轮没有做坐标转换或调用依赖安装高度的地面过滤。

## 2. 样本与方法

沿采集顺序的前、中、后段选 12 个不同 Snapshot，统一取中间点云 `lidar/cloud_01.npz`：

| 前段 | 中段 | 后段 |
| --- | --- | --- |
| `sample_000115` | `sample_000132` | `sample_000144` |
| `sample_000120` | `sample_000136` | `sample_000145` |
| `sample_000125` | `sample_000139` | `sample_000147` |
| `sample_000130` | `sample_000142` | `sample_000149` |

每个文件的完整绝对路径、SHA-256、清理点数、元数据时间和输出路径在 [selection.json](../outputs/r3_new_data_trial/selection.json) 与 [trial_summary.json](../outputs/r3_new_data_trial/trial_summary.json) 中。所有三种方法都读取**同一个原始 NPZ**，用相同 R1 规则排除 NaN/Inf 和全零 XYZ，保持 `raw_sensor` 坐标；逐帧核对三种运行的输入 SHA-256 与清理索引一致。没有混用 R1/R2 的旧数据。

`scripts/run_r3_new_data_trial.py` 直接调用现有 `scripts/offline_detector.py:run()`。配置保存在 `outputs/r3_new_data_trial/configs/`：

- `raw_dbscan.json`：不做地面过滤，DBSCAN `eps=0.35`、`min_points=12`。
- `raw_euclidean.json`：同一输入，不做地面过滤，欧式连通聚类 `radius=0.35`、`min_points=12`、`max_points=null`。
- `ransac_plane.json`：RANSAC `distance_threshold=0.15`、`ransac_n=3`、`num_iterations=1000`、`probability=0.999`、`random_seed=20261015`；`plane_policy=retain`，只提取平面候选，**不删除平面、不做聚类**。

阈值仅是原始坐标数值的固定实验设置，未按物理单位或点密度标定。Patchwork++ 与高度基线本轮未运行，因为每帧真实高度和竖直方向未确认。RANSAC 运行的候选区域仍等于清理后整帧点云；只有 DBSCAN/欧式运行会给出逐簇中心、轴对齐包围盒、雷达原点最近点数值和 `candidate_*.ply`。完整区域始终在 `candidate_region.ply`，包括未聚类点和连续墙面样结构，不依赖单个聚类框。

## 3. 实际输出与核心耗时

下表的“簇”仅是算法标签数。“小”表示 ≤30 点的簇数，是碎片化观察指标，不代表误检；“噪声%”是 DBSCAN 未聚类点比例；“欧最大%”是欧式最大连通簇占清理输入的比例，不代表一个真实障碍物。时间列为 `result.json` 中的**核心算法步骤耗时**（RANSAC/DBSCAN/欧式聚类，毫秒），不含读写、预处理或可视化。

| Snapshot | 清理点 | 平面候选点 | DB 簇/小/噪声% | 欧簇/小/最大% | R/DB/欧 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| 115 | 5,608 | 2,569 | 26/14/75.8 | 35/17/20.0 | 1.9/9.4/37.6 |
| 120 | 4,656 | 2,695 | 19/13/75.8 | 21/9/42.5 | 0.9/5.3/30.4 |
| 125 | 5,967 | 2,772 | 26/12/65.9 | 29/17/23.3 | 1.2/11.3/44.1 |
| 130 | 5,603 | 2,618 | 21/13/74.0 | 27/14/35.9 | 1.1/6.3/35.3 |
| 132 | 6,240 | 2,834 | 17/9/41.5 | 22/14/40.4 | 1.3/28.0/64.5 |
| 136 | 6,044 | 2,847 | 17/9/46.1 | 20/12/42.6 | 1.2/16.0/49.1 |
| 139 | 5,809 | 2,763 | 16/10/54.5 | 18/13/39.1 | 1.1/9.9/40.7 |
| 142 | 5,666 | 2,774 | 24/13/49.7 | 18/11/41.4 | 1.1/9.6/39.9 |
| 144 | 7,178 | 3,427 | 14/7/47.2 | 15/10/72.5 | 1.5/15.8/57.1 |
| 145 | 7,384 | 3,596 | 18/12/48.3 | 14/9/69.1 | 1.4/16.2/55.3 |
| 147 | 7,263 | 3,424 | 13/6/49.5 | 14/8/70.5 | 1.3/16.0/57.7 |
| 149 | 7,293 | 3,344 | 14/6/48.9 | 14/5/64.9 | 2.6/15.2/58.1 |

12 帧核心步骤范围：RANSAC 约 0.9～2.6 ms、DBSCAN 5.3～28.0 ms、欧式聚类 30.4～64.5 ms。这是本机离线流程内部计时，不能据此断言实时部署性能。逐帧总耗时、所有候选的中心/包围盒/最近点数值及完整配置在各运行目录的 `result.json`。

## 4. 可视化观察与人工检查

12 张 `comparisons/<sample>_comparison.png` 均采用相同 XY/XZ/YZ 投影和**跨所有选中帧固定的 X/Y/Z 数值范围**：X 约 `[-36.95,30.16]`、Y `[-35.30,28.39]`、Z `[-1.74,8.86]`。范围由选中清理点各轴 0.5%～99.5% 分位数再加 5% 边距生成；约 1.64% 的清理后点位于显示范围外，完整 PLY 未裁剪。XY 保持等比例，XZ/YZ 为可读性做纵向放大。每张图依次显示原始 NPZ（橙色叉表示重合的全零行）、RANSAC 平面候选（蓝色）、完整候选区域、DBSCAN 和欧式聚类（灰色为未聚类点）。

程序化可确认的现象：

- 前段 115～130 的 DBSCAN 未聚类点约 65.9%～75.8%，且 19～26 个簇中有 12～14 个 ≤30 点。固定阈值下既有大量稀疏点，又出现较多小簇；这些点是否为真实障碍仍需标签或三维人工核查。
- 在 12 帧里，每帧至少有 1 个欧式连通簇覆盖多个各含 ≥20 点的 DBSCAN 簇；前/中段的这种连通簇有 2～4 个。后段 144～149 最大欧式连通簇占清理点约 64.9%～72.5%，图中多条弧形/边界结构同色，提示**空间合并风险**。但这些区域是否是多个物体不能从聚类标签推定。
- RANSAC 各帧主平面点数占清理点约 45%～58%。115 与 145 的无向法向夹角约 87.3°；后段蓝色点集中在与前段不同方向的几何结构上。可能是优势平面切换或采集几何变化，无法归因于某个安装倾角，更不能把蓝色点直接作为地面删去。

优先人工打开以下四张对比图，并结合同名运行目录的 `plane_candidate.ply`、`candidate_region.ply`、`clusters_colored.ply` 在三维查看器中旋转观察：

| 图 | 重点检查 |
| --- | --- |
| [sample_000115_comparison.png](../outputs/r3_new_data_trial/comparisons/sample_000115_comparison.png) | 前段大量灰色点和 14 个小 DBSCAN 簇是否覆盖有意义的近场结构；蓝色主平面究竟是什么。 |
| [sample_000132_comparison.png](../outputs/r3_new_data_trial/comparisons/sample_000132_comparison.png) | 中段弧形结构被 DBSCAN 切开、被欧式连接的部位是否属于连续背景或独立目标。 |
| [sample_000142_comparison.png](../outputs/r3_new_data_trial/comparisons/sample_000142_comparison.png) | 相机图像可见人/推车样场景，但与点云接收时刻差约 0.66 秒；需确认目标是否实际进入对应点云、哪些簇与其重合。 |
| [sample_000145_comparison.png](../outputs/r3_new_data_trial/comparisons/sample_000145_comparison.png) | 主平面方向切换及欧式最大簇占 69.1% 的合并范围；相机草地/围栏背景不能直接作为点云标注。 |

图是二维投影，颜色仅表示聚类 ID，不表示物体类别。相机图像的过曝与接收时间差限制了跨模态判断；要判定碎片化是否误分、合并是否跨真实障碍，以及主平面是否地面，必须人工三维审查并建立少量点级/实例真值。

## 5. 输出、核对和下一步

全部实验产物位于 `outputs/r3_new_data_trial/`：`configs/` 三份参数，36 个 `<sample>_<method>/` 运行目录（每个有 `result.json`、索引 NPZ 和对应 PLY），`comparisons/` 12 张统一视角图，审查清单、图像总览、抽样记录、汇总 JSON 和 [validation_summary.json](../outputs/r3_new_data_trial/validation_summary.json)。`scripts/check_r3_new_data_trial.py` 已核对 **12 帧、36 次运行、556 个 PLY、12 张 PNG**：三方法源文件哈希/清理索引一致；所有候选点索引与原始 NPZ 坐标相符；中心、轴对齐包围盒和最近雷达原点距离重算一致；RANSAC 平面没有被删除。

复现命令（先激活已有 `mower-lidar` 环境）：

```powershell
python -B scripts/audit_r3_new_data.py
python -B scripts/run_r3_new_data_trial.py
python -B scripts/check_r3_new_data_trial.py
```

若只需根据已有运行结果重画图和汇总，可使用 `python -B scripts/run_r3_new_data_trial.py --reuse-results`，不会重新执行 36 次算法。没有修改原始点云、创建环境或安装依赖，也没有修改 C 模块、部署 ROS 2、开发车辆控制/路径规划。

下一步应先取得每个 Snapshot 的实际倾角、高度和可信外参，确认坐标单位、全零行语义与相机/雷达时间同步；为草地、人员、墙面、树木和路障采集有场景记录的点云，并标注少量地面点与障碍实例。之后在固定输入和明确物理尺度下，分别评估平面/地面过滤误删、聚类拆分与合并、候选位置/尺寸/最近距离误差，再做参数或路线优化。目前不能根据簇数、耗时或图片宣称任何检测准确率。
