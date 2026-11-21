# 运行、配置与检查

R3.8 支持 `--mode full`（默认，保留全部旧输出）和 `--mode core`（只生成 `obstacles.json`）。使用下文主入口配合 `--mode core` 即可，输出目录仍要求为空。core 保留完整几何字段，源结果文件、逐簇 PLY、PNG 和彩色 PLY 等未生成文件的引用为 `null`；不能将 core 输出交给依赖调试文件的旧完整性检查。独立雷达图模块继续读取 full 的文件输出。

R3.8 专项复核：`python -B scripts/check_r3_performance.py`。它会在 `outputs/r3_performance/correctness/` 生成对照产物，按同一份检测结果核对两种输出路径与八帧历史几何。性能批量测量命令与计时限制见 [性能报告](R3_PERFORMANCE_BENCHMARK.md)。

## 环境

在 Windows PowerShell 的项目根目录运行。已实际检查的环境为 Python 3.11.17 和 [README 列出的依赖版本](../README.md#环境与快速开始windows-powershell)。项目不要求 GPU；环境创建与依赖安装由使用者执行。`environment.yml` 仅声明 Python 3.11 基础 Conda 环境，`requirements.txt` 是现有 pip 版本快照。

## 现场单帧主流程

```powershell
conda activate mower-lidar
$npz = 'data/field/message_000092.npz'
$cfg = 'configs/field_66cm_7p5deg_ransac_dbscan.json'
python -B scripts/run_perception.py --input $npz --config $cfg --mode core --output-dir outputs/field_core
python -B scripts/run_perception.py --input $npz --config $cfg --mode full --output-dir outputs/field_full
```

将 `$npz` 指向本地符合协议的点云。`--output-dir` 必须是新目录或空目录。现场命令显式使用 [0.66 m / +7.5° 配置](../configs/field_66cm_7p5deg_ransac_dbscan.json)。历史 142 应使用 [R3.2 配置](../configs/r3_2_ransac_normal_trial.json)，它仍是省略 `--config` 时的默认值。主入口支持 `ransac_normal` + `dbscan`，`offline_detector.py --help` 另提供历史对照入口。[参数调优指南](PARAMETER_TUNING_GUIDE.md)

输出路径及字段见 [协议](INPUT_OUTPUT_SPEC.md)。主入口输出 `obstacles.json`、`obstacles_colored.ply`、`radar_view.png`、地面/非地面检查层，以及 `detection/` 内的完整原始结果。它不会生成四列 `overview.png`；该图属于 R3.2/R3.3 历史对比脚本。

## 批量执行

当多个历史 NPZ **已确认可共用同一实验配置**时，可逐个调用主入口，每帧使用独立空目录。以下示例假定使用者已把历史样本目录放入 `data/historical`。

```powershell
foreach ($n in 143..149) {
  $name = 'sample_{0:D6}' -f $n
  $npz = Join-Path (Join-Path 'data/historical' $name) 'lidar/cloud_01.npz'
  python -B scripts/run_perception.py --input $npz --config configs/r3_2_ransac_normal_trial.json --mode full --output-dir (Join-Path 'outputs/batch_review' $name)
  if ($LASTEXITCODE -ne 0) { throw "Failed: $name" }
}
```

143～149 的安装一致性没有独立证明，故这一批只可称共享安装假设下的实验结果。历史比较可运行 `scripts/run_r3_multiscene_validation.py`，但它依赖本地旧数据和旧输出，供原实验复核，不是通用批量接口。

## 已有自检和输出核验

公开克隆可直接用合成数据检查正式 CLI。

```powershell
python -B scripts/check_release_smoke.py --output-dir outputs/release_smoke
```

下面三项历史检查需要本地现场数据或历史产物。

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
