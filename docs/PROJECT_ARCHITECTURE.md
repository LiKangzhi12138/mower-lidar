# 项目架构与模块调用

## 边界与执行链

本仓库是 Mid-360 Snapshot NPZ 的离线感知实验工程。`scripts/run_perception.py` 是新开发者的单帧入口，只做配置校验和模块串联：`offline_detector.run()` 生成 R3 原始结果和索引，再把原有彩色簇 PLY 映射为 `radar_view.export_result()` 预期的文件名，最后生成 `obstacles.json` 与 `radar_view.png`。它没有重新实现平面搜索或聚类。

```text
NPZ points/fields
  -> npz_to_ply.read_cloud_data()             XYZ、intensity、字段名
  -> prepare_cloud.filter_point_rows()        有效行布尔掩码、原始行索引
  -> offline_detector.analysis_coordinates() 分析坐标 XYZ；原始 XYZ 同时保留
  -> offline_detector.normal_constrained_plane()
       -> extract_obstacle_candidates.extract_plane_candidate()
  -> extract_obstacle_candidates.cluster_dbscan()  非地面行标签，-1 为未聚类
  -> extract_obstacle_candidates.describe_candidate()  逐簇三维几何
  -> offline_detector.run()                   result.json、indices_and_labels.npz、PLY
  -> radar_view.export_result()               校验源 SHA、行索引和 PLY
       -> radar_view.xy_polar()/draw_radar()   obstacles.json、radar_view.png
```

处理时始终保留两组坐标：原始 `livox_frame` XYZ 用于 `nearest_sensor_distance`；`analysis_coordinates()` 输出的实验分析坐标用于平面搜索、DBSCAN、中心、包围盒及 XY 方位。`source_indices` 把清理后每一行映射回输入 NPZ 行。`indices_and_labels.npz` 存候选区域的输入行号与对应聚类标签；`radar_view.py` 根据此映射重新读取同一个 NPZ，并校验哈希、字段、中心及 PLY 坐标，防止把不同运行的点云和标签混合。

## 文件职责

| 文件 | 作用 |
| --- | --- |
| `scripts/run_perception.py` | 主路线单命令入口；只接受 `ransac_normal` + `dbscan` 配置；默认固定 R3.2 配置。 |
| `scripts/offline_detector.py` | 清理后坐标处理、地面/平面策略、聚类调度、索引/JSON/PLY 导出；其他配置支持历史实验。 |
| `scripts/npz_to_ply.py`、`scripts/prepare_cloud.py` | 共用 NPZ 解码和 R1 清理规则。前者的独立 `main()` 是早期本地转换示例。 |
| `scripts/extract_obstacle_candidates.py` | Open3D RANSAC/DBSCAN、中心/包围盒/距离、颜色及 PLY 函数。 |
| `scripts/radar_view.py` | 无检测算法；验证 R3 输出并增加二维信息、静态雷达图。 |
| `scripts/patchwork_ground.py` | Patchwork++ 历史对照，主入口不会选择。 |
| `scripts/finish_r3_ground_detection.py`、`scripts/run_r3_multiscene_validation.py`、`scripts/run_r3_new_data_trial.py` | 历史样本实验和对比可视化，含本地数据路径；不是可移植主入口。 |
| `scripts/audit_*.py`、`scripts/r3_ground_preflight.py`、`scripts/check_*.py` | 数据审查、先验检查与结果自检。 |

`configs/r3_2_ransac_normal_trial.json` 是当前主配置。其他 JSON 对应 Patchwork++、原始点云聚类、欧式聚类或高度基线试验。`docs/R*.md` 保存逐阶段原始结论，`outputs/` 是生成物且不纳入 Git；`data/` 不纳入 Git。

该链只有离线文件输入与输出。没有 ROS 2 订阅器、时间同步/跟踪、车辆坐标系正式外参、停车决策或规划模块。
