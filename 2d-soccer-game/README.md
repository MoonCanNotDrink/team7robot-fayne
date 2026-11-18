# TDP 足球迁移基线

## 第一次看比赛：双击就能开球

1. 在 Ubuntu 文件管理器中打开工程文件夹 **`tdp-migrated-r1`**。本机完整路径是 `/home/fayne/Documents/Robotics & AI/Robotics TDP/2d-soccer-game/tdp-migrated-r1`。
2. 第一次看比赛，**双击 `START_2D.desktop`（TDP 二维足球仿真）**。需要观察中场接管后自动把追球权交还前锋时，双击旁边的 **`START_2D_RECOVERY.desktop`（TDP 二维足球仿真〔自动恢复实验〕）**。两个图标分别启动默认 V1 和实验策略。
3. 首次打开若桌面询问信任，核对文件名和上述目录后选择“信任并启动”，或右键选择“允许启动”。可能先出现启动日志窗口，无需输入。首次模型编译可能需要几分钟。看到球场上的计时开始增加，蓝红两队和球在移动，就已启动。每次双击会新开一个进程，编译时请等候。

**已经打开 MATLAB，或者工程放在其他目录？** 在 MATLAB 的“当前文件夹”打开你自己的工程根目录。打开下表对应文件，点编辑器绿色 **Run ▶（运行）** 即可。若弹出工作目录选择框，选 **Change Folder（更改文件夹）**。这也是文件管理器把图标当文本打开时的启动方法。本机两个桌面图标指向上面写出的固定安装目录。

| 想看哪一场 | 文件管理器双击 | MATLAB 打开后点 Run |
| --- | --- | --- |
| 球队 V1（默认） | `START_2D.desktop` | `start_2d.m` |
| V1 自动交还实验 | `START_2D_RECOVERY.desktop` | `start_2d_recovery.m` |

每场默认 **600 秒、1 倍速**。蓝队 1–4 号、红队 5–8 号依次为门将、后卫、中场、前锋。金色外圈标出当前获准追球的队员。可在窗口暂停、继续、切换倍速、停止和重开。双击启动时，关闭球场窗口就会结束该次 MATLAB 进程。

只想确认自动交还逻辑？先做[8 秒无画面练习：让中场接管，再看球队恢复正常分工](docs/chapters/recovery_experiment.md#自己制造一次接管并看逐拍日志)。

[动画操作、运行记录和排错](docs/chapters/live_animation.md) · [动画验收与截图](docs/generated/animation_numbers.md) · [中文开发手册](docs/DEV_MANUAL.md) · [开发进度](docs/PROGRESS.md)

## 从终端启动或复现

在工程根目录运行 `./start_2d.sh` 可观看默认 V1，运行 `./start_2d.sh --recovery` 可观看实验策略。无画面复现则使用以下命令：

```bash
./scripts/run_matlab.sh "tdpSetup; reproduce_all;"
```

本分支包含冻结 A 基线、纯 MATLAB 算法、Stateflow 控制器、二维环境、测试和证据。ROS/Webots 的实际层级与依赖阻塞以手册状态表为准。

历史 `robot-soccer-starter/` 保留作结构参考，其原说明见该目录，不是本迁移工程的运行入口。

只复现新战术的二维验收可运行 `./scripts/run_matlab.sh "tdpSetup; reproduce_v1;"`。它从空白重建模型，测试指定场景，并跑 20 个种子、两种周期的完整比赛。`runMatch(tdp.defaultConfig())` 可用于原策略的对照实验。具体场景、参数和验收指标见[战术开发章节](docs/chapters/tactics_v1_design.md)。 [40 场实测成绩和复现耗时](docs/generated/v1_results.md)

自动交还实验与默认 V1 的完整同种子对照使用 `./scripts/run_matlab.sh "tdpSetup; reproduce_recovery;"`。本机干净克隆实测约 21 分钟。[交还条件与验证方法](docs/chapters/recovery_experiment.md) · [80 场实测成绩](docs/generated/recovery_results.md)

要区分“一队自己切换”和“对手切换”的作用，见[单队对照方法](docs/chapters/team_ablation.md)和[双方追球授权、射门与进球实测表](docs/generated/team_ablation.md)。
