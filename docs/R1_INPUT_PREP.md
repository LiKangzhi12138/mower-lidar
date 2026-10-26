# R1 点云标准化与可视化记录

审查日期：2026-10-15  
工程目录：`E:\workspace\mower-lidar`  
原始数据目录：`E:\workspace\mower-data`

## 1. 当前工程进度

R0 已有 NPZ 到 PLY 的离线转换脚本，以及一次 Patchwork++ 地面/非地面分离原型；尚无 ROS 2 实时感知节点，也没有障碍物聚类、尺寸/空间范围/最近距离估计或感知有效性输出。本阶段在既有 NPZ 解析逻辑上增加了标准化输入流程，没有实现地面分离或障碍检测。

新增 `scripts/prepare_cloud.py`，并扩展 `scripts/npz_to_ply.py` 的共享读取函数 `read_cloud_data()`，使其返回 XYZ、按命名字段识别的 intensity 和源字段名。原有 `read_cloud()` 保留为兼容包装函数；NPZ 解析没有在新脚本中重复实现。

## 2. 处理流程和输出格式

对单帧 NPZ 执行以下步骤：

1. 通过 `npz_to_ply.py:read_cloud_data()` 读取数据；必须能从字段名识别 intensity，否则报错，不猜测未命名的第 4 列。
2. 分别统计 XYZ 中含 NaN/Inf 的行、intensity 非有限值、两者合并后需删除的行。
3. 统计 XYZ 三轴严格等于零的行数和占原始行数比例。默认 `--zero-policy exclude`，从派生几何文件中排除全零点；原始 NPZ 只读且不会修改。传入 `--zero-policy keep` 可保留这些点。
4. 输出压缩 NPZ，使用规范字段 `points`（N×4，列顺序 x、y、z、intensity）和 `fields`（`["x", "y", "z", "intensity"]`）。数值不做坐标变换或单位换算。
5. 输出 PLY 供 CloudCompare 可视化。PLY 灰度仅为按该帧剩余 intensity 最小值和最大值线性归一化的显示颜色；原始 intensity 数值保存在清理 NPZ 中。
6. 输出逐帧 JSON 处理记录，包含输入路径、源字段、帧 ID（若能从 Snapshot 元数据匹配）、各类过滤计数、过滤参数、坐标范围及输出路径。

用法：

```powershell
& 'D:\NJUST\environment\envs\mower-lidar\python.exe' -B scripts\prepare_cloud.py '<输入NPZ路径>'
```

可用参数：`--output-dir`（默认 `outputs/r1_input_prep/`）、`--name`、`--zero-policy exclude|keep`。本次没有进行坐标转换、地面/障碍过滤或距离单位标注。

## 3. Snapshot 抽样测试

从两个 Snapshot 根目录各抽取两帧 `cloud_00.npz`。使用已有 `mower-lidar` Python 环境执行，未安装或变更依赖。

| Snapshot 批次 / 样本 | 原始点数 | XYZ 全零点数（比例） | NaN/Inf 删除行 | 默认输出点数 |
| --- | ---: | ---: | ---: | ---: |
| `airport_mower_samples/sample_000001` | 20,160 | 1,881（9.330%） | 0 | 18,279 |
| `airport_mower_samples/sample_000016` | 19,488 | 1,753（8.995%） | 0 | 17,735 |
| `samples/airport_mower_samples/sample_000001` | 19,968 | 13,492（67.568%） | 0 | 6,476 |
| `samples/airport_mower_samples/sample_000055` | 20,064 | 6,826（34.021%） | 0 | 13,238 |

这四个原始 NPZ 均为 `points: (N,4) float32` 和 `fields: (4,) Unicode`，字段为 x、y、z、intensity。抽样数据均未发现 XYZ 或 intensity 的 NaN/Inf。第一批两帧 XYZ 原始值范围约为 `[-4.256, -7.028, -0.683]` 至 `[5.306, 1.458, 2.604]`；第二批两帧约为 `[-52.141, -59.204, -9.789]` 至 `[70.559, 52.326, 25.383]`。这些是未换算的坐标值，不代表米；第三轴也不代表已标定的离地高度。

验证结果：

- 对每个输出 NPZ 重读后，字段名和列序均为 x、y、z、intensity；输出内容与原 NPZ 中通过有限值过滤并按默认全零策略保留的行逐元素完全一致。
- Open3D 成功重读 4 个 PLY；PLY 点数分别为 18,279、17,735、6,476、13,238，且每点均有颜色，与对应清理 NPZ 点数一致。
- 每个 JSON 处理记录中的输入点数、删除计数和剩余点数与重读验证一致。元数据中匹配到的 `frame_id` 均为 `livox_frame`。
- 另对第一批 `sample_000001` 使用 `--zero-policy keep` 验证可选策略：输入 20,160 点、全零 XYZ 1,881 点，输出仍为 20,160 点，记录的全零点删除数为 0。该策略验证产物也保存在下方输出目录。

## 4. 产物位置

代码和报告：

- `scripts/prepare_cloud.py`
- `scripts/npz_to_ply.py`（共享 NPZ 解析函数）
- `docs/R1_INPUT_PREP.md`

默认过滤的四帧各生成一组清理 NPZ、CloudCompare PLY 和 JSON 处理记录；另有一组保留全零点策略验证结果，共 15 个文件，均在 `outputs/r1_input_prep/`：

- `batch1_sample_000001_{clean.npz,cloudcompare.ply,processing.json}`
- `batch1_sample_000016_{clean.npz,cloudcompare.ply,processing.json}`
- `batch2_sample_000001_{clean.npz,cloudcompare.ply,processing.json}`
- `batch2_sample_000055_{clean.npz,cloudcompare.ply,processing.json}`
- `batch1_sample_000001_keep_{clean.npz,cloudcompare.ply,processing.json}`

## 5. 坐标、时间与数据限制

Snapshot 元数据能给出 `frame_id=livox_frame` 和逐帧时间字段；R0 对抽样元数据的检查发现 `header_stamp_ns` 与 `receive_time_ns` 存在差值，但其时钟来源和差值原因未确定。NPZ 不含点级时间戳，因此不能用这些文件验证逐点去畸变或跨传感器精确同步。

两批静态 TF 记录不一致：第一批记录 `base_link -> livox_frame` 零平移、单位旋转；第二批记录平移 `(-0.03, 0, 0.2)` 和单位旋转。R0 所见数据说明将相关外参标为 DRAFT、未正式测量标定，且 TF 数值单位也未声明。形式上有 frame ID 和部分静态 TF，不等于安装姿态、LiDAR 到车辆的真实外参或传感器离地高度已经确认。当前流程原样保留雷达坐标值，没有应用 TF。

抽样中全零 XYZ 比例从约 9% 到 68% 不等。虽已提供可选过滤策略，数据本身和元数据仍未说明零点是否为无回波占位或其他导出结果；该比例差异不应被解释为传感器质量结论。抽样覆盖两批数据的各两帧，不代表 417 帧总体的质量分布。

## 6. 下一阶段可复用内容与实验条件

可复用：

- `scripts/npz_to_ply.py:read_cloud_data()`：共享识别独立 XYZ、结构化点记录和带字段名的矩阵，并读取 intensity。
- `scripts/prepare_cloud.py`：以单帧文件为单位生成一致字段格式和逐帧可追溯过滤记录；可作为离线实验的数据入口。
- `scripts/patchwork_ground.py`：R0 的 Patchwork++ 调用和分割结果导出适配，可作为后续原型起点；其传感器高度等参数仍需校验和记录。

目前可进入**离线输入/可视化烟雾测试，以及在固定输入上的算法接口试跑**。还不具备可信、可复现的机场草地地面分离参数对比或定量精度评估条件。开始正式比较前需至少：

1. 确认 NPZ 坐标单位、坐标轴方向，以及两批点云坐标范围不同的原因。
2. 确认全零 XYZ 的产生机制，并固定经过验证的过滤规则。
3. 实测并验收 LiDAR 到 `base_link` 外参、传感器安装姿态和距地高度；不可用 DRAFT TF 推定。
4. 记录场景、采集方式及运动状态，澄清帧时间和接收时间的时钟语义；如需运动去畸变，补充点级时间或可靠位姿来源。
5. 准备机场草地目标场景和少量可信逐点 ground/non-ground 标签或地面参考，并记录所有算法参数。

R0 盘点显示现有数据可支持离线流程开发，但场景语义和逐点地面真值不足；相机图像标签不能替代 LiDAR 点级地面标签。完成以上数据/标定核实后，再开展可解释的地面分离对比。

## 7. 本阶段范围

只读原始 NPZ；所有新输出位于 `outputs/r1_input_prep/`。没有创建 Conda 环境、安装/升级/卸载依赖、修改 C 模块或 ROS 2 驱动，也没有实现地面分离、障碍物聚类、停车决策或路径规划。
