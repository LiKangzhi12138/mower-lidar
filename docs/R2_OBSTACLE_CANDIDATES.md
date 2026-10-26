# R2 Mid-360 离线空间障碍候选实验

实验日期：2026-10-15  
输入：R1 默认排除全零 XYZ 后的四帧 `*_clean.npz`  
结果目录：`E:\workspace\mower-lidar\outputs\r2_obstacle_candidates`

## 1. 目的与边界

本阶段验证：清理后的点云可否经单次 RANSAC 主平面提取，再对剩余点做 DBSCAN，形成可供空间检查的**实验性障碍候选**。它没有使用地面真值、车辆外参或坐标单位，不能据此宣布已验证的地面或障碍物检测。R0 指出第一批采集说明标为室内，第二批场景提示不足；四帧不能代表机场草地验证集。

读取 `docs/R0_DATA_AUDIT.md`、`docs/R1_INPUT_PREP.md` 后，脚本直接调用 `scripts/npz_to_ply.py:read_cloud_data()` 读取 R1 的 NPZ，并核对规范字段、有限值和配套 R1 处理记录。没有重新实现 NPZ 解析，也没有重新清理或改写 R1/原始数据。

## 2. 算法、数据产物与参数

`scripts/extract_obstacle_candidates.py` 对单帧执行：

1. 用 Open3D `segment_plane()` 估计一个主平面候选，保留平面点索引和剩余点索引。平面方程只用于记录几何结果；未按法向或位置把它称为地面。
2. 对剩余点用 Open3D `cluster_dbscan()`，把 `-1` 标签记为噪声；其余每个标签记为实验性空间候选。未对聚类结果施加障碍物语义筛选。
3. 对每个候选计算 XYZ 算术中心、输入坐标轴对齐包围盒的最小值/最大值/长宽高、距输入坐标原点最近的点及其欧氏距离。索引指向 R1 清理 NPZ；距离和包围盒均使用**未确认单位的原始坐标数值**，不是车辆坐标下的米数。
4. 保存平面候选 PLY、剩余点 PLY、彩色聚类 PLY、总览 PLY、三视角投影预览 PNG、索引/标签 NPZ 和逐帧 JSON。蓝色表示平面候选，灰色表示 DBSCAN 噪声，不同聚类用不同颜色。`segmentation.npz` 保存 `plane_indices`、`remaining_indices`、`cluster_labels_remaining`，可回溯到输入 R1 点序。

每组算法参数均在 JSON 中显式给出，`result.json` 还嵌入实际参数、输入和配置文件的 SHA-256、Open3D 版本、随机种子、单线程 OpenMP 设置、R1 零点策略及全部输出路径。运行环境的 Open3D 为 0.19.0。单线程和固定种子用于降低 RANSAC 并行随机性；参数表中的距离阈值与半径都只是原始坐标数值。

| 参数组 | RANSAC 距离阈值 | RANSAC 采样数 / 迭代数 / 目标概率 | DBSCAN eps / min_points | 随机种子 |
| --- | ---: | --- | --- | ---: |
| `batch1_fine` | 0.05 | 3 / 1000 / 0.999 | 0.15 / 12 | 20261015 |
| `batch1_coarse` | 0.15 | 3 / 1000 / 0.999 | 0.35 / 12 | 20261015 |
| `batch2_fine` | 0.20 | 3 / 1000 / 0.999 | 0.60 / 10 | 20261015 |
| `batch2_coarse` | 0.60 | 3 / 1000 / 0.999 | 1.50 / 10 | 20261015 |

参数只是用于观察灵敏度。选取前检查了四帧最近邻间距：第一批中位数约 0.025–0.028，第二批约 0.080–0.097，均为未确认单位的坐标差；因此没有把同一距离阈值直接套到两批数据上。每批对同一帧比较 `fine` 和 `coarse` 两组设置。配置源文件保存在 `outputs/r2_obstacle_candidates/configs/`。

复现单次运行示例：

```powershell
& 'D:\NJUST\environment\envs\mower-lidar\python.exe' -B scripts\extract_obstacle_candidates.py `
  outputs\r1_input_prep\batch1_sample_000001_clean.npz `
  --config outputs\r2_obstacle_candidates\configs\batch1_fine.json `
  --run-name batch1_sample_000001_fine
```

其余运行替换帧名及对应批次的 `fine`/`coarse` 配置和运行名即可。

## 3. 四帧、两组参数的实际结果

| R1 帧 | 参数组 | 输入点 | 平面候选点 | 剩余点 | 实验性候选簇 | 簇内点 | DBSCAN 噪声点 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `batch1_sample_000001` | fine | 18,279 | 4,921 | 13,358 | 53 | 11,535 | 1,823 |
| `batch1_sample_000001` | coarse | 18,279 | 5,330 | 12,949 | 11 | 12,915 | 34 |
| `batch1_sample_000016` | fine | 17,735 | 4,847 | 12,888 | 54 | 11,550 | 1,338 |
| `batch1_sample_000016` | coarse | 17,735 | 5,206 | 12,529 | 10 | 12,496 | 33 |
| `batch2_sample_000001` | fine | 6,476 | 1,967 | 4,509 | 45 | 2,412 | 2,097 |
| `batch2_sample_000001` | coarse | 6,476 | 3,372 | 3,104 | 23 | 2,653 | 451 |
| `batch2_sample_000055` | fine | 13,238 | 2,187 | 11,051 | 98 | 7,899 | 3,152 |
| `batch2_sample_000055` | coarse | 13,238 | 4,489 | 8,749 | 44 | 7,779 | 970 |

每行均满足输入点数 = 平面候选点数 + 剩余点数、剩余点数 = 簇内点数 + 噪声点数。聚类数量对参数很敏感。例如第一批 `sample_000001` 从 53 簇降为 11 簇，同时最大单簇由 6,118 点增至 8,127 点；这提示较大半径可能连接多个表面，不能把这些簇直接当作独立障碍物。第二批 `sample_000001` 的 fine 设置中剩余点有 2,097/4,509 被标为噪声，coarse 设置为 451/3,104；不能把噪声等同于“无障碍”。

平面同样对参数敏感。第一批两个样本的候选平面方程法向第三分量约 0.998，且预览里蓝色点主要沿第三坐标轴数值约 2.25 的上方带状结构分布；其真实场景语义未核实。第二批 `sample_000055` 的 fine 平面法向约为 `(0.949, -0.245, -0.197)`，coarse 约为 `(-0.055, -0.209, 0.976)`，无向夹角约 79°。仅改变阈值就选中了方向显著不同的结构，说明“主平面=地面”在这些数据上不能成立为默认假设。

## 4. 可视化与数据核对

每帧两套参数的三视角 XY/XZ/YZ 投影已上下对照（上半为 fine，下半为 coarse）：

- `comparisons/batch1_sample_000001_fine_vs_coarse.png`
- `comparisons/batch1_sample_000016_fine_vs_coarse.png`
- `comparisons/batch2_sample_000001_fine_vs_coarse.png`
- `comparisons/batch2_sample_000055_fine_vs_coarse.png`

已实际打开检查四张对比图。第一批宽松参数下大面积彩色簇相连；第二批 `sample_000055` 的蓝色平面候选随参数切换明显改变。PNG 是投影辅助图；要看完整空间结构和点级颜色，可在 CloudCompare 中打开各运行目录的 `overview.ply` 或 `clusters_colored.ply`。本阶段未在 CloudCompare 内实际打开文件。

程序化核对覆盖八次运行：重读 `segmentation.npz`，证实平面/剩余索引互斥且覆盖输入；逐簇重算中心、包围盒和最近点距离，均与 JSON 一致；Open3D 成功重读 32 个 PLY，点数与颜色数均等于对应类别的点数；8 个预览 PNG 均可读取。另将 `batch2_sample_000055_fine` 用相同配置重跑一次，平面索引、剩余索引和 DBSCAN 标签数组与原结果逐项相同。四张参数对比 PNG 由同帧两张预览上下拼接而成。结果目录总计 64 个文件：4 个配置、每次运行 7 个产物共 56 个、4 张对比图。

## 5. 输出结构

`outputs/r2_obstacle_candidates/` 下有四份 `configs/*.json`、四张 `comparisons/*.png`，以及以下八个运行目录：

```text
batch1_sample_000001_fine/    batch1_sample_000001_coarse/
batch1_sample_000016_fine/    batch1_sample_000016_coarse/
batch2_sample_000001_fine/    batch2_sample_000001_coarse/
batch2_sample_000055_fine/    batch2_sample_000055_coarse/
```

每个运行目录均有 `plane_candidate.ply`、`remaining_points.ply`、`clusters_colored.ply`、`overview.ply`、`projection_preview.png`、`segmentation.npz`、`result.json`。`result.json` 列出每个实验性候选的中心、轴对齐包围盒和最近点数值，亦列出所有文件的绝对路径。

## 6. 异常情况和正式验证前的事项

- RANSAC 只返回一个优势平面，可能选到墙、顶面或其他结构；同一帧在不同阈值下甚至会改变主要平面方向。要验证地面，需要已测量的重力/安装姿态与人工或可信地面参考，并考虑非平面草地、坡地、多平面情况。
- DBSCAN 对点密度和 eps 敏感，可能把一个物体切成多个簇，也可能把墙、地面残留和物体连成一个簇。当前没有障碍物真值，不能计算检出率或误报率。
- 两批数据坐标范围与采样密度差异明显，NPZ 单位仍未核实；参数中的阈值、包围盒尺度与原点距离不能解释为米，也不能跨批次做物理尺度比较。
- `frame_id` 来自 R1 所引用的 Snapshot 元数据，为 `livox_frame`；外参仍为 DRAFT。最近点距离仅相对输入坐标原点，不能用于车辆避障距离或停车阈值。点级时间缺失、帧/接收时间关系未厘清。
- 全零 XYZ 在 R1 默认结果中已排除，但其原始语义仍不明。第一批数据的场景说明和第二批样本的实际场景仍需核对，需补充目标机场草地场景、障碍类别及逐点/实例标注。

下一步应先确认坐标单位和零点语义，测量并验收 LiDAR 外参与距地高度，采集有场景记录和参考标注的草地数据，再以固定输入、明确的地面过滤与聚类评估指标开展正式算法比较。本 R2 结果仅支持离线链路和参数灵敏度分析。
