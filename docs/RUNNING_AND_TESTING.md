# 运行、配置与检查

## 环境

在 Windows PowerShell 的项目根目录运行。已实际检查的环境为 Python 3.11.17 和 [README 列出的依赖版本](../README.md#环境与快速开始windows-powershell)。项目不要求 GPU；环境创建与依赖安装由使用者执行。`environment.yml` 仅声明 Python 3.11 基础 Conda 环境，`requirements.txt` 是现有 pip 版本快照。

## 单帧主流程

```powershell
cd E:\workspace\mower-lidar
conda activate mower-lidar
$npz = 'E:\workspace\mower-data\data\newdata\sample_000142\lidar\cloud_01.npz'
python -B scripts/run_perception.py --input $npz --output-dir outputs/quickstart_sample_000142
```

该命令在本机 142 点云上实际运行通过。新用户应把 `$npz` 改成自己的文件路径；项目源码不要求 `E:\workspace\mower-data`。`--output-dir` 必须是新目录或空目录，防止覆盖结果。默认 [主配置](../configs/r3_2_ransac_normal_trial.json) 使用 `ransac_normal` + `dbscan`；可用 `--config <JSON路径>` 指定另一个配置，但此入口拒绝其他算法组合。`offline_detector.py --help` 支持研究用其他组合。别把 142 的姿态/高度参数直接当作新设备的标定值。

输出路径及字段见 [协议](INPUT_OUTPUT_SPEC.md)。主入口输出 `obstacles.json`、`obstacles_colored.ply`、`radar_view.png`、地面/非地面检查层，以及 `detection/` 内的完整原始结果。它不会生成四列 `overview.png`；该图属于 R3.2/R3.3 历史对比脚本。

## 批量执行

当多个 NPZ **已确认可共用同一实验配置**时，可逐个调用主入口，每帧使用独立空目录。例如本机历史样本范围：

```powershell
foreach ($n in 143..149) {
  $name = 'sample_{0:D6}' -f $n
  $npz = Join-Path (Join-Path 'E:\workspace\mower-data\data\newdata' $name) 'lidar\cloud_01.npz'
  python -B scripts/run_perception.py --input $npz --output-dir (Join-Path 'outputs/batch_review' $name)
  if ($LASTEXITCODE -ne 0) { throw "Failed: $name" }
}
```

143～149 的安装一致性没有独立证明，故这一批只可称共享安装假设下的实验结果。历史比较可运行 `scripts/run_r3_multiscene_validation.py`，但它依赖本地旧数据和旧输出，供原实验复核，不是通用批量接口。

## 已有自检和输出核验

```powershell
python -B scripts/check_offline_detector.py
python -B scripts/check_r3_radar_view.py --samples 142 143 144 145 146 147 148 149
python -B scripts/check_r3_multiscene_validation.py
```

这些脚本检查**既有本地实验输出**；GitHub 克隆不含 `outputs/`，应先运行自己的样本或跳过依赖历史结果的检查。`check_offline_detector.py` 含空点云、全零、无平面、无簇、非有限值等合成边界检查；`check_r3_radar_view.py` 覆盖四方向、原点、轴比例、JSON/PLY/输入索引一致性。新运行至少检查：JSON 能解析，`candidate_count == len(candidates)`；`ground` 与 `nonground` 点数之和等于清理点数；Open3D 可重读非空 PLY，且有 RGB；PNG 可打开；新旧同配置样本的候选几何逐字段相同。

## CloudCompare 人工审查

先单独打开 `ground.ply` 看绿色平面是否包含墙、树、人等竖直结构；再打开 `nonground.ply` 看应保留的连续结构和地面残留；最后看 `obstacles_colored.ply` 中彩色簇是否过度拆分或合并，灰色点是否覆盖潜在障碍。各 PLY 属于同一个实验分析坐标；切勿与未经校正的原始传感器点云直接叠加。雷达图 +X 向上、+Y 向左，圆环为同一 XY 坐标的等比例数值距离。人工视觉观察不能替代逐点真值和物理距离标定。

## 常见问题

- `FileNotFoundError`：检查 `--input` 是否为存在的 NPZ；本仓库不附原始数据。
- `Output directory is not empty`：换新的输出路径，不要覆盖已审查实验。
- 缺少 `open3d`、`matplotlib` 或其他包：激活已有合适环境；项目不会自动安装。`pypatchworkpp` 为历史 Patchwork++ 分支的现有导入依赖。
- `R3 colored PLY differs...` 或 SHA 不符：输出的索引、源 NPZ 或 PLY 与记录的那一次运行不一致；不要混用不同批次文件。
- 图上方位/距离与实物不符：先核查 NPZ 保存链、安装朝向/俯仰、高度参考点及单位；不得直接调整图像比例来掩盖标定问题。
