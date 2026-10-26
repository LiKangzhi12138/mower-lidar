# R3.3 Mid-360 七组新数据离线验证

日期：2026-10-23。范围严格限定 `E:\workspace\mower-data\data\newdata\sample_000143`～`sample_000149`，每组只用 `lidar/cloud_01.npz`；`sample_000142` 仅作 R3.2 配置参考，未计入本轮统计，也没有读取更早样本参与运行。所有输出为**共享安装假设下的实验性地面与障碍候选**，没有人工真值。

## 安装记录、方法与固定配置

逐组检查了 `metadata.json`、`tf_static.json`：七组点云帧名均为 `livox_frame`，`scene_hint` 和 `operator_note` 均为空；没有逐组倾角、高度或安装变更记录。七份 `base_link → livox_frame` 静态 TF 均为平移 `(-0.03,0,0.2)`、单位四元数，与 142 相同；R3 数据说明已把该 TF 标为 DRAFT，不变的草案值不能证明实物姿态未变化。七组元数据创建时间为 2026-10-21 08:10:48Z～08:15:32Z，具体值见 [汇总 JSON](../outputs/r3_multiscene_validation/validation_summary.json)。

完整复用 R3.2 的 [Patchwork++ 配置](../configs/r3_2_patchwork_trial.json) 与 [法向约束 RANSAC 配置](../configs/r3_2_ransac_normal_trial.json)：同一 `read_cloud_data()`、R1 有限值/XYZ 全零过滤、`R_y(+5°)` 和 0.415 的**实验安装假设**、DBSCAN `eps=0.35`/`min_points=12`。RANSAC 距离阈值 `0.12`、法向容许偏差 `15°`、参考平面截距阈值 `0.2`；Patchwork++ 使用 `sensor_height=0.415`、范围 `0.1..80`，RNR 关、RVPF/TGR 开。其他参数在各 `result.json` 中。没有按帧调整阈值，也没有使用 142 的点云参与统计。

这里把雷达 +X 对车头、+Y 左、+Z 上，向车头前倾 5° 的雷达到分析坐标旋转 `R_y(+5°)`，以及 NPZ 沿用上游米制 XYZ，均视为**实验假设**。NPZ 保存转换链、实际雷达前向、高度 0.415 m 到雷达内部坐标原点的偏移没有核实；因此不能说正式完成外参标定。中心和包围盒处于实验分析坐标，最近距离仍按原始雷达坐标原点计算；输出 JSON 将坐标单位确认证据标为 `false`，不作为车辆轮廓距离或已验证物理尺寸。具体依据与变换式见 [R3.2 报告](R3_GROUND_DETECTION_TEST.md)。

## 七组结果

每一行两算法使用同一 NPZ 哈希和相同的清理后索引。`噪声%` 是 DBSCAN `-1` 点占该方法非地面候选点的比例；这些点仍保留在 `nonground.ply` 与彩色 PLY 中。时间是本机一次离线运行的**地面处理/聚类核心耗时**，不含读取、导出和可视化。簇数不等于真实障碍数量。

| 样本 | 清理点 | RANSAC 地面/非地面 | R 簇/噪声% | R 地面/聚类 ms | Patchwork++ 地面/非地面 | P 簇/噪声% | P 地面/聚类 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 143 | 5,768 | 2,709 / 3,059 | 18 / 72.5% | 1.50 / 3.38 | 2,661 / 3,107 | 17 / 72.5% | 2.77 / 4.27 |
| 144 | 7,178 | 2,194 / 4,984 | 5 / 36.6% | 2.25 / 14.02 | 2,340 / 4,838 | 4 / 39.9% | 2.66 / 12.16 |
| 145 | 7,384 | 2,241 / 5,143 | 9 / 37.0% | 4.35 / 13.88 | 2,392 / 4,992 | 9 / 38.8% | 2.54 / 13.03 |
| 146 | 7,226 | 2,218 / 5,008 | 6 / 37.6% | 1.91 / 12.75 | 2,388 / 4,838 | 6 / 39.9% | 3.79 / 13.56 |
| 147 | 7,263 | 2,179 / 5,084 | 6 / 36.9% | 2.16 / 11.86 | 2,320 / 4,943 | 7 / 39.8% | 2.35 / 11.50 |
| 148 | 7,097 | 2,198 / 4,899 | 10 / 35.6% | 2.26 / 12.08 | 2,341 / 4,756 | 8 / 38.8% | 2.52 / 11.74 |
| 149 | 7,293 | 2,238 / 5,055 | 4 / 37.7% | 2.38 / 13.06 | 2,425 / 4,868 | 5 / 41.0% | 2.01 / 10.39 |

七组清理后合计 49,209 点，两方法的未分类点均为 0；因此按“存在时导出”的规则没有生成空的 `unclassified.ply`。`result.json` 中保存逐候选的中心、轴对齐包围盒及尺寸、原始雷达原点最近点距离与原始 NPZ 行号，供具体簇核查。

## 可视化文件与人工检查顺序

统一输出根目录：`E:\workspace\mower-lidar\outputs\r3_multiscene_validation\`。每个 `sample_000xxx/patchwork/` 和 `sample_000xxx/ransac_normal/` 目录均含 `ground.ply`、`nonground.ply`、`obstacles_colored.ply`、`overview.png`、`original.ply`（清理后、实验姿态校正输入）、`sensor_raw_clean.ply`（未校正原始雷达坐标）、`unclustered.ply`、完整 `candidate_region.ply`、候选 `candidate_*.ply`、`result.json` 与索引文件。**同一目录内的分类层可叠加；`sensor_raw_clean.ply` 不应与校正层直接叠加。**

| 样本 | 两算法四列图 | Patchwork++ 图 | RANSAC 图 |
| --- | --- | --- | --- |
| 143 | [对比](../outputs/r3_multiscene_validation/sample_000143/patchwork_vs_ransac.png) | [概览](../outputs/r3_multiscene_validation/sample_000143/patchwork/overview.png) | [概览](../outputs/r3_multiscene_validation/sample_000143/ransac_normal/overview.png) |
| 144 | [对比](../outputs/r3_multiscene_validation/sample_000144/patchwork_vs_ransac.png) | [概览](../outputs/r3_multiscene_validation/sample_000144/patchwork/overview.png) | [概览](../outputs/r3_multiscene_validation/sample_000144/ransac_normal/overview.png) |
| 145 | [对比](../outputs/r3_multiscene_validation/sample_000145/patchwork_vs_ransac.png) | [概览](../outputs/r3_multiscene_validation/sample_000145/patchwork/overview.png) | [概览](../outputs/r3_multiscene_validation/sample_000145/ransac_normal/overview.png) |
| 146 | [对比](../outputs/r3_multiscene_validation/sample_000146/patchwork_vs_ransac.png) | [概览](../outputs/r3_multiscene_validation/sample_000146/patchwork/overview.png) | [概览](../outputs/r3_multiscene_validation/sample_000146/ransac_normal/overview.png) |
| 147 | [对比](../outputs/r3_multiscene_validation/sample_000147/patchwork_vs_ransac.png) | [概览](../outputs/r3_multiscene_validation/sample_000147/patchwork/overview.png) | [概览](../outputs/r3_multiscene_validation/sample_000147/ransac_normal/overview.png) |
| 148 | [对比](../outputs/r3_multiscene_validation/sample_000148/patchwork_vs_ransac.png) | [概览](../outputs/r3_multiscene_validation/sample_000148/patchwork/overview.png) | [概览](../outputs/r3_multiscene_validation/sample_000148/ransac_normal/overview.png) |
| 149 | [对比](../outputs/r3_multiscene_validation/sample_000149/patchwork_vs_ransac.png) | [概览](../outputs/r3_multiscene_validation/sample_000149/patchwork/overview.png) | [概览](../outputs/r3_multiscene_validation/sample_000149/ransac_normal/overview.png) |

[七组总览图](../outputs/r3_multiscene_validation/seven_sample_comparison.png)按行展示 143～149、按列展示 Patchwork++ 地面/非地面和 RANSAC 地面/非地面的 XZ 投影，使用跨七组统一坐标范围；范围由全部清理点的 0.5%～99.5% 分位数加 5% 边距得到，可能裁掉极端点，**PLY 保留全部点**。各样本的 `overview.png`、`patchwork_vs_ransac.png` 提供不裁剪的 XY/XZ/YZ 三视角和各图层独立列。

## 明显差异与待人工判断的问题

- **143 与 144～149 出现突变。** 143 的 RANSAC 地面候选法向与假设竖直轴夹角约 0.90°；144～149 为约 7.35°～7.69°，在统一 XZ 图中为重复出现的斜长带。后六帧的 RANSAC 首个优势平面各有约 3,279～3,524 点，但法向偏离竖直约 82°，被约束搜索拒绝并保留在非地面候选中；这一点避免了“最大平面直接当地面”的明显风险。被接受的第二平面是否真实地面，以及斜带由坡面、安装变化还是其他空间结构造成，现有元数据和图像无法判定。
- 143 的两法地面索引交并比约 0.876；144～149 降至约 0.687～0.737。后六组 Patchwork++ 额外标为地面的点约 433～526 个，RANSAC 独有地面点约 263～339 个；这说明分类差异值得点级核查，**不能**把重合率作为准确率。
- 在 144～149 的共同校正假设下，Patchwork++ 绿色点中分析 Z > 0.5 的点约 639～747 个；RANSAC 约 806～881 个。部分 RANSAC 绿色点达到分析 Z 约 4.7～6.4。这些“高点”可能属于真实坡面或远处其他平面，也可能提示姿态假设不适用或误删障碍；先在三维与场景记录中确认，不能单靠 Z 阈值判错。
- 143 的未聚类比例两法均约 72.5%，144～149 约 35.6%～41.0%；彩色簇数从 4 到 18 不等。这表明相同 DBSCAN 参数对不同采样/场景的连通性敏感。需检查灰色点是否覆盖真实障碍、连续墙面或树干是否被拆散，以及大簇是否连接多个物体。不能根据簇数变化推定真实障碍数量变化。

建议下一轮优先人工查看 **143、144、145、149**：143 与 R3.2 的 142 作相邻基准；144 查突变和被拒绝的巨大陡立平面；145 查两法大量高 Z 地面候选；149 查两法地面交并比最低及少簇、大结构合并风险。146～148 同样属于 144～149 的异常模式，不应因未列入优先四组而视为通过。

## 完整性检查与当前边界

`scripts/run_r3_multiscene_validation.py` 调用既有 `offline_detector.run()` 和 R3.2 的可视化/几何核验函数；七组固定配置与 142 参考运行的配置内容逐项相同。运行时逐组核对清理输入和坐标变换一致、索引无重叠或丢失、每个候选的中心/包围盒/雷达原点最近距离与原 NPZ 行号一致，导出 PLY 可由 Open3D 重读且坐标/RGB 正确。之后执行 `scripts/check_r3_multiscene_validation.py` 再核对源文件哈希、7 组/14 次运行、70 个基础非空 PLY、114 个逐候选 PLY 和 22 张 PNG，结果 **passed**。`unclassified.ply` 在本轮因 0 点而不生成。原始数据未修改，未创建环境或安装依赖。

从工程根目录、使用已有 `mower-lidar` 环境复现：

```powershell
python -B scripts/run_r3_multiscene_validation.py
python -B scripts/check_r3_multiscene_validation.py
```

运行脚本若发现已有 `result.json`，会先核对源文件哈希与固定配置，再复用算法结果并重新生成/校验可视化；若检测到部分或参数不匹配的结果则报错，不混用。独立完整性脚本另核对逐候选 PLY。

当前结果不能证明安装状态真的与 142 相同、NPZ 导出链完全保留米制、0.415 m 为内部坐标原点高度、地面分类正确、墙面/树木/人员没有误删、候选数量等于物体数量，或满足任何检出率/实时性指标。下一步先取得 143～149 的安装日志/照片或逐帧可信姿态，并为上述四组标少量地面与典型障碍点；在确认共享姿态和实际场景后再评估阈值、误删与聚类拆分/合并。
