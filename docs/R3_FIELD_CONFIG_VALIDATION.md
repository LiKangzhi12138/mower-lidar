# R3.10 Mid-360 现场配置与 Core 回归验证

**现场配置通过本批离线回归验收。** 111 条已转换的现场 PointCloud2 消息全部经正式入口生成 Core 输出。地面候选接受数从 R3.9 旧配置的 0/111 变为本轮的 111/111。建议把配置交给部署人员做现场回放和外参复核。候选接受率表示平面通过几何门限，真实地面分类和在线效果仍待验证。

## 配置与实测加载

逐字段比较确认新配置相对于旧模板只改了 `coordinates.mount_height` 和 `coordinates.mount_pitch_degrees`。入口通过 `--config` 加载现场文件。批量验收逐条检查检测器返回的配置路径、完整生效参数和 `obstacles.json` 的坐标、地面与聚类参数。另在命令行实际运行了消息 92。省略 `--config` 仍从旧模板加载，并用 sample_000142 回归验证。[现场配置](../configs/field_66cm_7p5deg_ransac_dbscan.json) · [正式入口](../scripts/run_perception.py)

| 参数 | 现场生效值 |
| --- | --- |
| 安装高度 / 前向下俯 | 0.66 m / 7.5°，配置写入 0.66 和 +7.5° |
| 变换 | 原始 `livox_frame` 绕雷达 +Y 右手正转，沿分析 +Z 平移 |
| 清理 | 排除非有限点及 XYZ 全零点 |
| RANSAC | 距离 0.12，3 点采样，1000 次迭代，概率 0.999，种子 20261023 |
| 地面约束 | 法向角 ≤15°，绝对竖直截距 ≤0.20，最多 6 次平面搜索 |
| DBSCAN | `eps=0.35`，`min_points=12` |

朝向采用当前实验约定，雷达 +X 与车头方向的关系以及安装高度是否量到内部坐标原点仍需正式外参核实。配置继续保留 `unit_confirmed=false`。以下点云中的量值沿用该分析坐标数值。

## 111 条现场消息

复用 R3.9 验收清单中的 NPZ。111 个 NPZ 的前后 SHA-256 一致。累计 2,220,192 个原始点，过滤后 1,808,162 点，其中 265,164 点划为地面候选，1,542,998 点进入 DBSCAN。

| 数据包 | Core 成功 | 旧配置接受（R3.9） | 现场配置接受 | 现场地面点数最少 / 中位 / 最多 |
| --- | ---: | ---: | ---: | ---: |
| front_1m_20261119-164850 | 27/27 | 0/27 | 27/27 | 2,411 / 2,459 / 2,514 |
| front_1m_20261119-165004 | 51/51 | 0/51 | 51/51 | 1,454 / 2,456 / 3,241 |
| partial_overlap_20261119-165259 | 33/33 | 0/33 | 33/33 | 2,209 / 2,291 / 2,346 |

全部 111 条先拒绝两个优势平面，再通过第三个平面。通过的法向与竖直轴夹角约 5.35°～6.08°，绝对竖直截距约 0.029～0.065。逐消息记录包含输入和有效点数、每个候选平面的指标和拒绝原因、接受平面、地面/非地面点数、簇数、最大簇点数及占比、噪声占比、耗时及输出路径。最大簇与噪声比例以非地面点数为分母。[逐消息验收](../outputs/r3_field_config_validation/per_message.json) · [汇总 JSON](../outputs/r3_field_config_validation/summary.json)

| Windows 离线耗时（ms） | P50 | P95 | 最大值 |
| --- | ---: | ---: | ---: |
| RANSAC 地面阶段，含索引操作 | 33.03 | 44.69 | 49.76 |
| DBSCAN 阶段 | 40.28 | 48.24 | 58.27 |
| 每条消息的 RANSAC + DBSCAN | 73.19 | 86.94 | 99.52 |
| 检测器已有内部 total | 79.36 | 92.24 | 105.40 |
| Core 入口端到端 | **100.83** | **115.92** | **132.21** |

每条消息单次运行，共 111 个样本。端到端从调用 `run_pipeline` 到返回，包含 NPZ 读取、预处理、检测和写入 `obstacles.json`，不含进程启动和回归核验。检测器内部 `total_seconds` 在计算源文件哈希和生成 Core 最终 JSON 之前结束。此结果供离线复核，现场 ROS 2 与 Jetson 延迟尚未测量。

## 输出兼容性及地面失败诊断

- **Core** 111 个现场帧目录均仅生成一个 `obstacles.json`，无 PLY 或 PNG。逐候选独立重算并核对 **4,285** 条记录的原始传感器最近三维坐标和距离，以及分析坐标的中心、AABB、尺寸、中心和最近水平距离及方位。平面与候选几何和簇计数也与 R3.9 现场配置记录一致。
- **Full** 消息 92、144 的 Full 与对应 Core 候选字段一致（逐簇文件引用除外），`ground.ply`、`nonground.ply`、`obstacles_colored.ply` 点数及颜色可读，`radar_view.png` 可读取。
- **旧配置** sample_000142 的默认配置 Core 与显式旧配置 Full 均为 18 个候选。完整候选字段与 R3.4 历史记录一致（Core 的逐簇文件引用按协议为空）。
- **失败诊断** 用旧参数对现场 92 再跑 Core，复现 6 个平面被拒绝。调用方可以用 `selected_plane_diagnostics == null`、`plane_model_ax_by_cz_d == null`、`removed_point_count == 0` 识别无地面候选。此时 15,678 个有效点仍全部进入 DBSCAN，输出 42 簇。现有字段无法区分提前终止时的具体退出原因，`ground_note` 也不编码状态。后续可在显式版本下提供可选的地面状态和终止原因，区分候选通过、搜索耗尽和输入不足等。当前对外协议保持原样。Full 的地面诊断在 `detection/result.json`。

## 消息 92 与 144 的同点复核

对照图统一在现场分析坐标中显示旧、新聚类标签，仅改变画图投影。黑线为用实际 `eps=0.35` 和 `min_points=12` 求出的核心点路径，空心节点为新配置剔除的地面候选。查找较高路径时，用 R3.9 已存的标签选取端点，逐点确认其输入和当前 0.35 标签一致。本轮所有 DBSCAN 调用均为 0.35。

| 消息 | 剔除后的旧低平面连接 | 现场配置仍存在的高处路径 | 整帧最大簇 / 噪声占比 |
| --- | --- | --- | --- |
| partial_overlap 92 | 旧簇 #3 的路径中 4/6 点成为地面候选，两端现在分属 #3、#21。旧簇共 1,703 点被判地面 | 当前 #3 中仍有 4 个核心点连接两个片段，距平面约 0.87～1.82 | 5,087 点，占非地面 37.95% / 39.57% |
| front_1m 第二包 144 | 旧簇 #7 的路径中 7/9 点成为地面候选，两端现在分属 #7、#25。旧簇共 1,784 点被判地面 | 当前 #7 的两个核心点仍以约 0.239 的邻边相连，距平面约 1.14～1.38 | 5,354 点，占非地面 38.97% / 38.15% |

两个旧路径都已被切断，现存较高路径仍值得检查是否把真实物体合并。图中可以看到地面候选延伸到部分竖向结构脚边。按观察窗口内距平面不超过 0.30 的保留点计算，两条消息各有 8 个地面候选在这些保留点的 XY 邻域 0.12 内。邻近可以是地面接触，也可能涉及物体低处误删，需查看实物或逐点真值。上述检查没有覆盖所有可能的低处路径。

- 消息 92 [连接对照图](../outputs/r3_field_config_validation/cases/partial_overlap_20261119-165259__000092/connection_check.png) · [路径原始行号与距离](../outputs/r3_field_config_validation/cases/partial_overlap_20261119-165259__000092/geometry_checks.json)
- 消息 144 [连接对照图](../outputs/r3_field_config_validation/cases/front_1m_20261119-165004__000144/connection_check.png) · [路径原始行号与距离](../outputs/r3_field_config_validation/cases/front_1m_20261119-165004__000144/geometry_checks.json)

建议把现场配置连同以上两条对照交给部署人员，以实测外参和在线同帧输入做回放验收，重点核对障碍脚点及较高连接处。在取得在线同帧数据和标注前，不能判断剩余过度合并是否对应多个真实物体，也不能直接以离线成功率作为在线部署验收。

## 复现

在项目根目录使用已有 Python 环境。运行目录选一个新的空目录。

```powershell
python -B scripts/check_field_config.py --output outputs/r3_field_config_validation_recheck
python -B scripts/plot_field_config_cases.py --output outputs/r3_field_config_validation_recheck
python -B scripts/run_perception.py --input outputs/r3_field_merging_diagnostic/converted/partial_overlap_20261119-165259/message_000092.npz --config configs/field_66cm_7p5deg_ransac_dbscan.json --mode core --output-dir outputs/field_core_demo_92
```
