# Mid-360 雷达工程 R0 数据审查报告

审查日期：2026-10-15  
工程目录：`E:\workspace\mower-lidar`  
原始数据目录：`E:\workspace\mower-data`

## 审查范围

盘点了工程文件和全部原始数据文件的目录/扩展名数量，检查了两个 Snapshot 根目录下全部样本的文件齐备情况与 JSON 元数据。点云数组内容只读取了 10 组 Snapshot：两个根目录各按样本编号均匀抽取 5 组，每组读取 3 帧，共 30 个 NPZ；未复制或修改数据集。抽查样本为：

- `airport_mower_samples`：`sample_000001`、`sample_000008`、`sample_000016`、`sample_000023`、`sample_000030`
- `samples/airport_mower_samples`：`sample_000001`、`sample_000028`、`sample_000055`、`sample_000085`、`sample_000113`

## 1. 当前项目真实进度

工程里目前有两个处理脚本和一批流程检查产物，依赖清单为 `environment.yml` 与 `requirements.txt`：

- `scripts/npz_to_ply.py`：读取 NPZ 中的 XYZ，按第三列数值着色，并导出 PLY。
- `scripts/patchwork_ground.py`：用 Open3D 读取 PLY，调用 `pypatchworkpp` 分离地面/非地面，再写出三份 PLY。
- `outputs/startup_check/`：已有一个着色点云和 Patchwork++ 的地面、非地面、合并结果。

`configs/`、工程内 `data/`、`docs/` 在本次审查前为空；没有 ROS 2 感知节点、配置文件、自动测试或障碍物聚类/跟踪/尺寸/最近距离/有效性输出。当前代码覆盖的是离线文件转换和一次 Patchwork++ 调用，不是实时障碍物感知模块。工作目录也不是 Git 仓库，无法从提交历史验证这些产物的生成过程。

`environment.yml` 声明 Python 3.11，并含有本机绝对前缀 `D:\NJUST\environment\envs\mower-lidar`；实际检查的既有环境中 NumPy 1.26.4、Open3D 0.19.0、Patchwork++ 1.4.1、SciPy 1.17.1、scikit-learn 1.9.1 均可导入。没有发现大段重复实现；两个处理脚本各自做了一次有限值过滤，属于轻微重复。本次审查脚本复用 NPZ 读取函数。

Open3D 实际读取现存 PLY 的点数如下：

| 文件 | 点数 | 说明 |
| --- | ---: | --- |
| `outputs/startup_check/cloud_00_colored.ply` | 20,160 | 含颜色 |
| `outputs/startup_check/patchworkpp/ground.ply` | 535 | 当前产物中约 2.65% |
| `outputs/startup_check/patchworkpp/nonground.ply` | 19,625 | 当前产物中约 97.35% |
| `outputs/startup_check/patchworkpp/segmented.ply` | 20,160 | 两类点数之和与输入点数相同 |

这只能证明流程产出了两类点，不能证明地面分类正确；没有对应人工地面真值或参数运行记录。

## 2. 现有数据格式与质量

### 数据盘点

`E:\workspace\mower-data` 共 1,784 个文件：JSON 671、PNG 469、NPZ 417、CSV 140、BAK 64、TXT 12、YAML 5、DB3 2、LOG 2、MD 2。LiDAR Snapshot 分布在两个目录：

| 根目录 | Snapshot 数 | 编号范围/缺号 | 每组点云 | 元数据情况 |
| --- | ---: | --- | ---: | --- |
| `airport_mower_samples` | 29 | 1–30，缺 12 | 3 个 NPZ | 每组有 `metadata.json` 和 `tf_static.json`；一个 session；全部 `rtk_valid=false` |
| `samples/airport_mower_samples` | 110 | 1–113，缺 63–65 | 3 个 NPZ | 每组有 `metadata.json` 和 `tf_static.json`；4 个 session；全部 `rtk_valid=true` |

总数为 139 组、417 个 NPZ。第一批的目录名并非完整的 1–30 连续集合；第二批元数据的 `label_hint` 全部是 `stone`。这些提示标签不是点云逐点标签。另有 `master/master` 图像标注工程，记录 110 张图像及 `stone/tree/person/other_roadblock` 类别；它不是 LiDAR 地面/障碍物标注。

### 抽查的 NPZ 内容

抽查的 30 个文件布局一致：

```text
points: (N, 4), float32
fields: (4,), Unicode 字符串 ["x", "y", "z", "intensity"]
```

每帧约 18,432–21,120 行。所有 30 帧的实际行数都与对应 `metadata.json` 的 `point_count` 一致；`metadata.json` 给出的 `frame_id` 均为 `livox_frame`。NPZ 中保留 intensity，取值在抽查数据中为 0–255，类型为 `float32`；未发现点级时间戳字段。采集时间戳和帧时间只在 Snapshot 元数据里按帧记录。

抽查合计 596,352 行 XYZ：未发现 NaN/Inf。存在大量 XYZ 三轴全为零的行：第一批 28,353/296,352（9.567%），第二批 133,652/300,000（44.551%）。代码和元数据没有说明这些行代表无回波占位还是有效点；当前统计将其单独列出，没有自动判为坏点，也没有从数据中删除。

下面的范围和距离均是文件里的原始数值；距离按 `livox_frame` 原点计算，并在分布统计时排除三轴全零行。NPZ 和元数据未声明坐标单位，因此这些数值不能直接标注为米。Z 只称第三坐标轴，不能在外参确认前当作重力高度。

| 抽查数据 | XYZ 最小值 | XYZ 最大值 | 原点距离 p50 / p95 / p99 / 最大值 |
| --- | --- | --- | --- |
| 2026-10-01 根目录的 5 组 | `[-4.390, -7.533, -0.697]` | `[5.354, 1.465, 2.840]` | `2.470 / 4.670 / 5.159 / 8.251` |
| `samples/` 根目录的 5 组 | `[-67.237, -70.936, -9.789]` | `[70.559, 60.977, 53.354]` | `7.021 / 30.640 / 46.235 / 88.299` |

排除全零行后，第一批 Z 轴的 p50/p95/p99 为 `0.787/2.384/2.483`，第二批为 `1.674/10.311/14.365`；它们是坐标轴统计，不是已标定的障碍物离地高度。第二批坐标跨度明显更大，且全零行比例也更高；需要先向上游确认数据单位、点云生成方式和全零行语义，再用于同一套阈值比较。

## 3. 已知坐标系、时间关系和缺少的标定

`metadata.json` 为每帧记录 `frame_id=livox_frame`、`header_stamp_ns`、`receive_time_ns`、触发接收时间以及 `alignment_basis="Jetson snapshot node ROS receive time"`。抽查 30 帧的 `receive_time_ns - header_stamp_ns` 从约 94.6 ms 到 3,048.7 ms。记录了两类时间，但本地元数据没有说明各时钟的来源，也没有给出差异原因；跨相机、IMU、LiDAR 的精确同步关系尚不能据此确认。点云没有点级时间字段，无法从 NPZ 做逐点运动去畸变。

两个 Snapshot 根目录的静态 TF 内容不同：

- 第一批 29 组的 `base_link -> livox_frame` 平移为 `(0, 0, 0)`、四元数为单位姿态；摄像头和 GPS 平移也均为零。
- 第二批 110 组记录 `base_link -> livox_frame` 平移 `(-0.03, 0, 0.2)`、四元数为单位姿态。还记录了 `base_link -> camera_front_link`、`base_link -> gps_link` 和相机 optical frame 变换。
- 随数据提供的说明明确写明 TF 外参仍是 DRAFT、尚未正式测量和标定；数值单位也没有在这些 JSON 中声明。

若第二批 TF 按 parent/child 变换语义解释，数据形式上足以计算 `livox_frame` 到 `base_link` 的刚体变换；第一批的全零平移则不能作为可信安装外参。由于安装姿态和数值尚未验收，目前不能把转换结果称为真实车辆坐标，也不能把 TF 中的 Z 平移直接用作 Patchwork++ 的传感器离地高度。样本含 IMU，但没有动态车辆位姿；从这些文件也无法把点云变换到随时间变化的世界坐标系。

## 4. 现有 Patchwork++ 实现和参数

实现位于 `scripts/patchwork_ground.py`：

1. 默认读取 `outputs/startup_check/cloud_00_colored.ply`，并只对文件是否存在作检查。
2. 转成 float32 XYZ，去除 NaN/Inf；有效点少于 100 时退出。
3. 要求调用者提供 `--sensor-height`，`--min-range` 默认 0.5；设置 `params.enable_RNR=False`。
4. 将 XYZ 放进 N×4 数组，第四列全为零，再调用 `estimateGround()`。
5. 通过 `getGround()` / `getNonground()` 取结果，分别写 PLY，并输出合并文件。

本机现有 `mower-lidar` 环境的 `pypatchworkpp` 版本为 1.4.1，与 `requirements.txt` 相符。Patchwork++ Python 参数对象的库默认值包括 `sensor_height=1.723`、`min_range=2.7`、`max_range=80`、RNR/RVPF/TGR 启用；但脚本强制要求 `--sensor-height` 并把 min range 改成 0.5、关闭 RNR。没有工程配置保存其余参数，现有 PLY 也没有记录调用时传入的 `sensor_height`，所以无法复现已有分类参数。对 5 个新建的参数对象进行 introspection 时，`elevation_thr` 和 `flatness_thr` 都是全零数组；`intensity_thr` 在不同对象间变化（例如约 `1.94e-109` 与 `6.95e-310`），不应当作可靠默认值。后续实验应显式检查并设置需要的阈值；当前脚本关闭 RNR，因此 `intensity_thr` 不参与该脚本配置下的分离。

关闭 RNR 是当前 PLY 路径下的合理保护：`npz_to_ply.py` 将真实 intensity 丢弃，只保存 XYZ 与显示颜色，Patchwork++ 收到的第 4 列也只是零占位。但原始 NPZ 已含 intensity，因此若实验需要 RNR，应从 NPZ 读取真实 intensity 并保留正确列序，而不是继续把 PLY 的颜色或零占位当作强度。

参数假设和已发现的问题：

- `sensor_height` 必填且没有可追溯的工程值；不能从 DRAFT TF 推定。
- `min_range=0.5` 没有配置说明。若数据单位最终确认为米，它会改变近场参与分类的范围；也与库默认 2.7 不同。
- 转换器和 Patchwork++ 都过滤非有限 XYZ，但两处都没有按当前样本的全零行比例单独处理；后续应先确认全零行语义。
- `npz_to_ply.py` 按第三轴着色，不能据此称为“高度颜色”。
- 现有输出未保留 intensity、帧号、时间戳或坐标系信息；文件本身不足以追溯参数和源帧。
- PLY 地面/非地面输出当前点数能对上，但没有真值可评估准确性。

## 5. 后续可直接复用的代码

- `scripts/npz_to_ply.py:read_cloud()`：识别独立 x/y/z、结构化数组和带 `fields` 的 `points` 矩阵。本次把它改为按 `fields` 中的 x/y/z 名称取列，并将转换逻辑放入 `main()`，从而可以安全地被统计脚本导入。
- `scripts/npz_to_ply.py` 的 Open3D 写 PLY 与第三轴着色流程：可继续用于可视化检查，但颜色轴应在坐标确认后重命名/解释。
- `scripts/patchwork_ground.py` 的 `make_cloud()`、Patchwork++ 输入适配、地面/非地面结果导出可作为离线原型起点；须补参数记录和强度处理。
- `scripts/audit_snapshot_clouds.py`：本次新增的只读脚本。默认扫描两个 Snapshot 根目录，最多均匀抽查 10 组，打印 NPZ 字段/形状/类型、点数、XYZ 边界和分位数、非有限行、全零行、强度分布、元数据时间和静态 TF。示例：`python scripts/audit_snapshot_clouds.py --max-snapshots 10`。也可用重复的 `--sample-root` 指定其他根目录。

没有创建障碍物检测、聚类、跟踪、尺寸估计或停车/控制逻辑；这些不属于本阶段实现。

## 6. 后续地面过滤和障碍提取实验的数据条件

现有 139 组 Snapshot 和 417 帧 NPZ 足以开始**离线流程/可视化试跑**，也足以比较不同实现对相同文件的输出稳定性。但当前材料还不支持对“机场草地场景地面分离效果”作可靠、可复现的定量结论，原因是：

1. 目标场景的说明不足：第一批数据对应的现有采集说明标为室内；第二批 `scene_hint` 未提供，`stone` 只是 `label_hint`。`rtk_valid=true` 本身不能证明点云来自机场草坪。
2. 没有已确认的坐标单位、LiDAR 安装俯仰/横滚和传感器相对地面的实测高度；现有静态 TF 被标为 DRAFT。
3. 全零行在两批数据中占比不同且最高达 44.551%，其含义和正确过滤规则未确定。
4. 没有 LiDAR 地面/障碍物逐点真值。现有 `master` 是图像标注，不可直接用来计算点云分割精度。
5. 帧时间戳与接收时间有最高约 3.05 秒的差值，且没有点级时间戳；移动采集的同步和运动畸变无法从当前 NPZ 验证。

建议下一轮先确认数据导出规则和零点语义，实测并验收 `base_link`/`livox_frame` 外参与单位；再采集有场景记录的机场草地、平地/坡地/边缘及不同距离障碍物数据。准备一小批人工点级 ground/non-ground 标签或可信地面参考后，固定同一输入帧、单位、零点过滤策略和传感器高度，记录全部 Patchwork++ 参数，再开展可复现的算法对比。静止或低速试验可先隔离运动畸变影响。

## 7. 问题和建议的下一步

按优先级建议：

1. 让数据导出责任方确认 `points` 的单位、字段定义、全零行来源以及 `header_stamp_ns`/`receive_time_ns` 的时钟语义。
2. 正式测量并版本化 LiDAR 到 `base_link` 的外参；测量传感器到草地面的实际高度，不以静态 TF 草稿代替。
3. 为 Patchwork++ 建立显式 YAML/CLI 参数记录，至少记录输入文件、`sensor_height`、`min_range`、所有启用模块及包版本；校验当前 binding 的阈值默认值。
4. 确认地面过滤对比的场景集和标签协议，采集目标机场草地数据并建立小规模点级真值。
5. 之后再在复用的离线接口上比较 Patchwork++ 与其他地面过滤基线，并单独评估地面/非地面质量；障碍物聚类、空间范围和最近距离属于后续阶段。

本次只在工程目录新增此报告和只读审查脚本，并调整 `npz_to_ply.py` 以复用读入逻辑、避免导入时写文件；没有更改原始数据、安装依赖、创建 Conda 环境或运行 ROS/实机服务。
