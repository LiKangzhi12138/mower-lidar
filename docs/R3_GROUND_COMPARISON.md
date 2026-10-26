# R3.2 Mid-360 地面分离实验：坐标校正预检

日期：2026-10-23。指定输入：`E:\workspace\mower-data\data\newdata\sample_000142\lidar\cloud_01.npz`。

## 结论与执行边界

**四组地面分离对比未执行。** 本任务要求先核实 NPZ 坐标单位、轴定义和俯仰旋转方向；核实失败时停止正式坐标校正。现有文件只能证明点云帧名为 `livox_frame`，不能证明 NPZ 数值单位、传感器各轴与车头的关系、-5°/-10° 应绕哪个轴及按什么符号旋转。用户给出的 0.415 m 未说明测量参考点和误差。因此没有生成 `ground.ply`、`nonground.ply`、`obstacles_colored.ply`、四列地面分离 `overview.png` 或两角度总对比图，也没有运行 Patchwork++、受约束 RANSAC 或后续 DBSCAN。绝不能把空缺当成零点、零簇或零耗时。

输出目录中的图和 PLY 仅为**未校正原始雷达坐标系的既有几何候选诊断**，不是地面/障碍分类结果。-5° 与 -10° 均保持待验证假设，未判断孰真。

## 核查依据

- 阅读了 [R0 审查](R0_DATA_AUDIT.md)、[R1 输入准备](R1_INPUT_PREP.md)、[R3 离线检测](R3_OFFLINE_DETECTOR.md)、[R3.1 新数据实验](R3_NEW_DATA_TRIAL.md)，以及 `scripts/npz_to_ply.py`、`prepare_cloud.py`、`offline_detector.py`、`patchwork_ground.py`、`extract_obstacle_candidates.py` 的相关实现；读取了这帧 NPZ、同组 `metadata.json` 与 `tf_static.json`，检查了新数据目录及已有 R3.1 结果。
- NPZ 为 `points` N×4 `float32`，字段 `x,y,z,intensity`；没有点级时间戳、单位或坐标轴说明。`metadata.json` 的 `frame_id=livox_frame`、`point_count=19200`；该帧 `header_stamp_ns=1792570155409797194`、`receive_time_ns=1792570155561261098`。`label_hint=stone` 仅为采集提示，`scene_hint=null`。
- 该组 `tf_static.json` 记载 `base_link → livox_frame` 的平移 `(-0.03,0,0.2)` 和单位四元数。R3.1 已核查全部 35 组新数据的这个 TF 均相同，而采集说明称安装倾角与高度变化。该 TF 与本任务给出的 0.415 m 的关系不明，不能选作当前帧的实际标定。即便 TF 可信，`base_link` 原点距地高度也未给出，TF 的 z=0.2 不能直接解释为雷达原点离地高度。
- `offline_detector.py:analysis_coordinates()` 已支持显式传入旋转轴、角度、参考竖直轴和高度；这些参数没有从 NPZ 或元数据得到。当前 RANSAC 分支可保留几何平面，但**没有地面法向约束**；在标定不明时不应新增一个假定的竖直轴来删除最大平面。Patchwork++ 的物理阈值与 `sensor_height` 也不能用未知单位和未知高度参考点来填写。
- [Livox ROS Driver 2 文档](https://github.com/Livox-SDK/livox_ros_driver2)说明其 `PointXYZRTLT` 中 XYZ 以米表示；[Livox Mid-360 以太网协议](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)则说明某些原始笛卡尔报文坐标以毫米整数表示。这些是**上游格式规范**，不足以证明本次 NPZ 导出链的单位、缩放和坐标变换。未找到能将导出文件与具体上游字段逐步对应的记录，因此此 NPZ 的单位仍为 UNKNOWN。

0.415 m 是用户提供的安装高度数值。仍需确认它是否从雷达坐标原点量到地面，是否按铅垂线测量，测量时车辆是否处于本帧采集姿态，以及测量误差。它与静态 TF 的 z=0.2 不可直接比较为同一参考量。

## 输入质量与可复用的原始坐标诊断

使用现有 `read_cloud_data()` 和 R1 `filter_point_rows(..., "exclude")` 读取与清理，不改原始 NPZ。输入 SHA-256：`ce36f2fc1daa82dc20d39402f200ced5a1f842d1fb2e7d6a95fc6622b5547315`。

| 项目 | 实测值 |
| --- | ---: |
| 原始行数 | 19,200 |
| 非有限 XYZ 行 / 非有限强度行 | 0 / 0 |
| XYZ 全零行 | 13,534（70.49%） |
| 清理后点数 | 5,666 |
| XYZ 原始数值范围 | X `[-41.109, 35.388]`；Y `[-70.378, 38.692]`；Z `[-3.607, 11.409]` |

上述坐标范围不标米；Z 是第三坐标，不等同于离地高度。排除全零行只用于几何计算，原始文件未改变。R3.1 对**同一文件、同一清理索引**的原始坐标计算得几何主平面候选 2,774 点，原始 DBSCAN 24 簇、未聚类点 2,815；核心 RANSAC 约 1.05 ms、DBSCAN 约 9.62 ms。这些是既有 R3.1 参数下的历史运行结果，不能充当地面点数、非地面点数或地面分离后的障碍数量。既有配置和详细候选记录见 `outputs/r3_new_data_trial/configs/` 与 `outputs/r3_new_data_trial/sample_000142_raw_dbscan/result.json`。

原始坐标诊断产物：[overview.png](../outputs/r3_ground_comparison/raw_frame_diagnostic/overview.png)、[original.ply](../outputs/r3_ground_comparison/raw_frame_diagnostic/original.ply)、[plane_candidate.ply](../outputs/r3_ground_comparison/raw_frame_diagnostic/plane_candidate.ply)、[clusters_colored.ply](../outputs/r3_ground_comparison/raw_frame_diagnostic/clusters_colored.ply)。图中统一采用每个投影下相同 XYZ 坐标范围与视角：第 1 列清理后输入；第 2 列蓝色部分仅是几何平面候选；第 3 列是未经地面过滤的完整候选区域；第 4 列为原始 DBSCAN 着色。二维图中蓝色结构在 XZ 投影呈斜线，但它是否是地面、斜线由安装倾角还是场景几何造成，均不可由此判定。需要在 CloudCompare 中核查 [PLY](../outputs/r3_ground_comparison/raw_frame_diagnostic/) 所示三维结构，并结合真实标定与人工点级标签判断是否误删人员、墙面或推车样结构。

## 用户请求的四组结果

| 假设与方法 | 地面点 | 非地面点 | 聚类数 | 核心耗时 | 地面分离可视化 |
| --- | ---: | ---: | ---: | ---: | --- |
| -5° + Patchwork++ | 未执行 | 未执行 | 未执行 | 未执行 | 未生成 |
| -5° + 法向约束 RANSAC | 未执行 | 未执行 | 未执行 | 未执行 | 未生成 |
| -10° + Patchwork++ | 未执行 | 未执行 | 未执行 | 未执行 | 未生成 |
| -10° + 法向约束 RANSAC | 未执行 | 未执行 | 未执行 | 未执行 | 未生成 |

四组被同一预检条件阻断，详情与机器可读状态见 [preflight.json](../outputs/r3_ground_comparison/preflight.json)。因此无法评价地面分离是否合理、墙面/人员/推车样结构是否被误删、非地面点是否形成独立障碍候选、未分类点数量，以及两种角度的结果差异。原始 DBSCAN 的灰色点是未聚类点，不能称为地面算法的“未分类”点。已有 R3.1 图也不能用簇数、耗时或画面观感选择真实角度或声称检测准确率。

## 一致性检查与恢复实验所需信息

执行 `D:\NJUST\environment\envs\mower-lidar\python.exe -B scripts/r3_ground_preflight.py`，复用既有算法产物而非重跑全部 R3.1。脚本检查：原始输入 SHA-256 与 R3.1 的两次运行一致；R1 清理后源索引与两次运行相同；RANSAC 平面索引均来自清理输入且没有从候选区删除；导出后用 Open3D 重读 PLY，点数分别为 5,666、2,774、5,666；PNG 已打开人工检查。未创建环境、安装依赖、修改原始数据或 C 模块。

恢复四组对比至少需要：

1. NPZ 导出程序或记录，证明 XYZ 单位、是否缩放及是否已经经过坐标变换；最好用可知尺寸的标定物复核物理比例。
2. 本帧雷达坐标轴与车辆前/左/上的对应关系、右手系约定，以及“车头前方下倾”的旋转轴和正负号；需有该帧有效的外参或可复核的 IMU 重力/标定靶依据，不能仅凭最大平面估计。
3. 0.415 m 对应的雷达参考点、到地面的测量方向、车辆姿态和误差范围；若不是雷达坐标原点，应给出参考点到原点的偏移。
4. 少量场景级与点级真值，特别是地面、墙面、人员/推车样结构的标记，以及相机和雷达的时间对应关系，才能判断误删和聚类合并/碎片化。

取得上述信息后，可在 `offline_detector.py` 保留现有清理、Patchwork++、DBSCAN、候选描述和 PLY 输出，实现**最小必要**的法向约束 RANSAC（角度阈值、平面距离阈值、随机种子和法向来源显式配置），先验证原始点索引映射，再用同一输入、视角和坐标范围跑四组；在人工真值下比较，而不是把最大平面直接删去。
