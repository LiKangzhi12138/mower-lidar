# R3.2 Mid-360 地面分离与障碍候选离线实验

实验日期：2026-10-23。输入：`E:\workspace\mower-data\data\newdata\sample_000142\lidar\cloud_01.npz`。结果：`E:\workspace\mower-lidar\outputs\r3_ground_comparison\sample_000142_final\`。所有地面和障碍标签均为**实验候选**，没有点级真值。

## 坐标依据与明确假设

已阅读 R0、R1、R2、R2.5、R3、R3.1 及先前 R3.2 预检报告，检查 `offline_detector.py`、共享 NPZ 读取/清理、Patchwork++、Open3D RANSAC/DBSCAN 与导出函数。NPZ 为 `points: (19200,4) float32`、`fields=[x,y,z,intensity]`，同组元数据注明 `/livox/lidar` 和 `livox_frame`。原始数据目录及本工程没有找到生成该 NPZ 的保存程序，因此**不能独立核实导出时是否保持上游单位或是否施加了变换**。项目数据说明明确称 TF 外参仍是 DRAFT；该帧 `tf_static.json` 的单位四元数、`z=0.2` 与本次 0.415 m 的关系不明，未用于校正。

[Livox Mid-360 官方手册第 12 页](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)给出雷达本体的 O-XYZ 坐标图，Z 轴沿机身竖直方向。 [Livox ROS Driver 2 的点类型定义](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/comm.h)将发布的 XYZ 定义为以米为单位的 `float`；其 [PointCloud2 发布代码](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp)包含 x/y/z/intensity 字段。由于缺少本项目的 NPZ 导出代码，**本实验只假设 NPZ 保留 ROS 驱动的米制 XYZ**；报告中的高度、阈值、尺寸及距离均须按此假设解读，不是已验收的物理量。NPZ 没有逐点时间戳。

用户确认这帧向车头前方下倾 5°。仍没有本帧安装照片或可信的车辆外参能证实雷达本体 **+X 指向车头**；本次将“+X 前、+Y 左、+Z 上，无额外偏航/横滚”列为显式实验假设。若真实前向不是 +X，旋转轴必须随外参重算。按照右手系，雷达 +X 在车体中向下倾 5° 对应**雷达点到水平参考系**绕传感器 +Y 旋转 `+5°`：

```text
R_y(+5°) = [[ cos5°, 0, sin5°],
            [     0, 1,     0],
            [-sin5°, 0, cos5°]]
p_analysis = R_y(+5°) p_sensor + [0, 0, 0.415]
```

正号可从 `R_y(+5°)[1,0,0]` 的 Z 分量为负直接核对。原始 R3.1 几何主平面法向 `(-0.0982,-0.0151,0.9951)` 在该假设旋转后接近竖直，本次约束 RANSAC 法向与 +Z 夹角 1.03°、平面在分析原点的截距约 0.0246（米制假设）。**这只是内部几何相容性，不证明该平面真实是地面，更不证明雷达前向/高度参考点正确。**

0.415 m 是用户提供的对地安装高度；它是否从 Mid-360 手册的内部 O 点沿铅垂方向量至地面、地面平整程度和测量误差均未知。分析坐标把它作为 Z 平移假设；Patchwork++ 需要以雷达为原点的水平点云，因此调用前只传 `R_y(+5°) p_sensor`，并将 `sensor_height=0.415` 单独传入，避免重复加高度。两算法使用同一批点和同一姿态假设。最近点距离仍按**未平移的雷达原点**计算，不是车辆轮廓距离。

## 输入和算法配置

共享读取器与 R1 过滤规则删除非有限和 XYZ 全零行：19,200 原始行，非有限行 0，全零 13,534（70.49%），进入两种算法的点均为 5,666；原始 NPZ 未修改。两组配置分别为 [Patchwork++ JSON](../configs/r3_2_patchwork_trial.json) 与 [法向约束 RANSAC JSON](../configs/r3_2_ransac_normal_trial.json)，有效 Patchwork++ 默认参数及全部参数嵌入各自 `result.json`。

| 项目 | Patchwork++ | 法向约束 RANSAC |
| --- | --- | --- |
| 高度 / 姿态 | 0.415、`R_y(+5°)` 假设 | 同左 |
| 地面参数 | `sensor_height=0.415`，`min_range=0.1`，`max_range=80`，RNR 关、RVPF/TGR 开，其余绑定默认 | 距离阈值 `0.12`，3 点、1000 次、概率 `0.999`、种子 `20261023`；法向偏差 ≤15°，绝对竖直截距 ≤0.2，最多搜索 6 个平面 |
| 聚类参数 | DBSCAN `eps=0.35`，`min_points=12` | 同左 |

所有长度阈值都依赖“NPZ 为米”的实验假设。法向约束 RANSAC 复用现有 Open3D `segment_plane()`：依次寻找优势平面；不满足法向和截距条件的平面只从**搜索池**暂时剥离，其点仍留在最终非地面候选中。只有通过条件的平面被标为地面候选。本帧首个平面通过，没有被拒绝的优势平面；另用“墙面点多于水平面点”的合成数据验证墙面会被拒绝且不进入地面输出。此方法仍可能把水平桌面、路面或其他水平结构混淆，也不能处理所有起伏地面。

## 实际结果

| 方法 | 地面候选 | 非地面候选 | 未分类 | DBSCAN 簇 | 未聚类点 | 地面核心耗时 | DBSCAN 核心耗时 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Patchwork++ | 2,559 | 3,107 | 0 | 19 | 2,018 | 1.98 ms | 3.40 ms |
| 法向约束 RANSAC | 2,706 | 2,960 | 0 | 18 | 1,961 | 1.14 ms | 2.84 ms |

这仅是本机一次离线运行的核心步骤计时；各运行完整读入、变换、写盘耗时及依赖版本见 [Patchwork++ 结果 JSON](../outputs/r3_ground_comparison/sample_000142_final/patchwork/result.json) 与 [RANSAC 结果 JSON](../outputs/r3_ground_comparison/sample_000142_final/ransac_normal/result.json)。`summary.json` 给出统一概览。两法地面候选交集 2,458 点，并集 2,807 点；Patchwork++ 独有 101 点，RANSAC 独有 248 点。交并关系说明两法并非完全一致，不能据此评价谁准确。各法都满足 `5666=地面+非地面+未分类`；DBSCAN 未聚类点保留在非地面 PLY 与彩色 PLY 中，灰色显示，不被当作不存在。此帧 Patchwork++ 未分类数为零，所以按“存在时导出”规则没有 `unclassified.ply`；有未分类点时脚本会输出橙色文件。

## 可视化与人工检查

以下均为**完整绝对路径**，建议用 CloudCompare 分别打开，同一运行目录内的 `original.ply` 与其他分类 PLY 位于同一实验水平坐标；`sensor_raw_clean.ply` 是未校正雷达原始坐标，请勿与校正层直接叠加。

| 文件 | 人工检查重点 |
| --- | --- |
| `E:\workspace\mower-lidar\outputs\r3_ground_comparison\sample_000142_final\patchwork\ground.ply` | 绿色区域是否只落在真实地面；尤其检查较高的 47 个点（分析 Z > 0.5，其中 19 个 > 1.0）是否属于人、推车或竖向结构。 |
| `E:\workspace\mower-lidar\outputs\r3_ground_comparison\sample_000142_final\ransac_normal\ground.ply` | 对照 Patchwork++ 的绿色范围，核查远处低点和近场结构是否被平面延伸误吸收。 |
| `E:\workspace\mower-lidar\outputs\r3_ground_comparison\sample_000142_final\patchwork\nonground.ply`、`...\ransac_normal\nonground.ply` | 两种方法的红色区域是否完整保留外围连续墙面样轮廓、竖向结构及人/推车样点。两套 `nonground.ply` 不宜同时显示；逐个切换。 |
| 两个方法目录各自的 `obstacles_colored.ply` 与 `unclustered.ply` | 检查大结构是否被拆成多簇，灰色未聚类点是否覆盖疑似障碍；不要把 19/18 个簇解释为真实障碍数。 |

每个方法目录还包含 `original.ply`（清理后、实验姿态校正的同框输入）、`sensor_raw_clean.ply`、`overview.png`、完整候选区域 `candidate_region.ply`、各簇 `candidate_*.ply`、索引文件与 `result.json`。[总对比图](../outputs/r3_ground_comparison/sample_000142_final/patchwork_vs_ransac.png)及各自 [Patchwork++ 四列图](../outputs/r3_ground_comparison/sample_000142_final/patchwork/overview.png)、[RANSAC 四列图](../outputs/r3_ground_comparison/sample_000142_final/ransac_normal/overview.png) 采用相同坐标范围、XY/XZ/YZ 三视角；每列依次为清理并实验校正输入、绿色地面、红色非地面、彩色 DBSCAN（灰色未聚类）。各层单独呈现，未把全部点云叠在一幅难辨的图上。

二维投影中，两法均保留了外围矩形及较高竖向轮廓的大部分点；这只是几何观察，不能确认其真实类别。Patchwork++ 地面候选的分析 Z 最高约 1.24，含前述较高点，存在误删竖向结构的风险；RANSAC 地面候选的分析 Z 最高约 0.42，但最低约 -0.95，需核查远处被同一斜平面吸收的点。非地面中分别有 2,018/3,107 和 1,961/2,960 点未聚类，固定 `eps=0.35` 对当前稀疏采样可能留下大量结构；最大彩色簇沿分析 X 跨度约 7.21（米制假设），需三维检查是否合并多个物体。没有真值和精确相机同步时，无法判定地面误删率、候选拆分/合并错误或实际人员、树木、墙面、路障的检出情况。

## 一致性检查与复现

`scripts/finish_r3_ground_detection.py` 已检查两次结果的清理源索引完全相同、实验校正后的点坐标完全相同；地面/候选索引无交叉且并集等于 5,666；全部候选中心、轴对齐包围盒尺寸、最近**原始雷达原点**距离及其原始 NPZ 行号重算一致。Open3D 成功重读每个非空 PLY，点数、坐标、RGB 颜色均通过检查；三视角 PNG 已打开检查。合成墙面测试中，法向约束搜索拒绝优势墙面并选取水平面。结果只说明实现与记录一致，不代表物理分类准确。

复现（使用已有 `mower-lidar` 环境；原始 NPZ 只读）：

```powershell
python -B scripts/offline_detector.py --input 'E:\workspace\mower-data\data\newdata\sample_000142\lidar\cloud_01.npz' --config configs/r3_2_patchwork_trial.json --output-dir outputs/r3_ground_comparison/sample_000142_final --run-name patchwork
python -B scripts/offline_detector.py --input 'E:\workspace\mower-data\data\newdata\sample_000142\lidar\cloud_01.npz' --config configs/r3_2_ransac_normal_trial.json --output-dir outputs/r3_ground_comparison/sample_000142_final --run-name ransac_normal
python -B scripts/finish_r3_ground_detection.py
```

下一步应取得本帧 NPZ 导出代码、雷达 +X 是否对车头的实物安装依据，以及 0.415 m 相对于 Mid-360 内部 O 点的偏移/误差；并在 CloudCompare 对上述高 Z 绿色点、远处平面、外围墙面样结构和灰色点作人工标签。之后再评估阈值与算法优劣；现阶段不输出准确率，不用于车辆控制。
