# 开发日志

按日期记录已核实的开发事件。每条尽量同时标出**原实现日期**（代码最初写成/提交的时间）和**集成日期**（本次整合、验证的时间），两者不要混用。
进度叙述见 [PROGRESS.md](PROGRESS.md)，决策与假设见 [DECISIONS.md](DECISIONS.md)。时间均为 UTC。文中的 C1–C4 和提交号对应当时的本地集成记录。

| 来源标签 | 含义 |
|---|---|
| 已提交 | 可由 `git log` 核对 |
| 工作树文件时间 | 仅文件修改时间，不等于提交时间 |
| 先前报告 | 引用既有报告，未在本条重新执行 |
| 本机独立执行 | 集成日期当天在本机实际运行并留有日志 |

**当时的提交状态（2026-11-13）**：C1 V1.2（`865ed1c`）、C2 Qin 原始指标模块（`a9e5e1c`）、C3 Qin 指标校验加固（`4b28b82`）已作为本地提交存在于分支 `phase-e-b3-v11-fallback`；**未 push、未 merge、Qin 尚未完成团队代码审阅**。本批文档（C4）是这些提交之后的文档维护提交，见文末 C4 条目。下文各条按原实现日期与集成日期分别记录；条目内已标出“当时”状态的描述是历史信息。

Qin 的指标模块 `qin-metrics-2` 来自 Qin 的仓库。已有记录没有逐行的人工审阅。

## 2026-11-10 · 回放检查器、事件导航与视频导出

- 提交 `65c2d6f`（2026-11-10 20:53:42，作者名 MoonCanNotDrink）。［已提交］
- 当时的记录未列出这批工具的人工审阅。［先前记录］

## 2026-11-11 · V1.1 前锋回退

- 提交 `dd2ba68`（2026-11-11 20:00:39，作者名 MoonCanNotDrink）：“Add opt-in V1.1 striker fallback with repeated-failure takeover and timed hand-back”。分支 `phase-e-b3-v11-fallback`。默认 V1 行为不变，仅在 `P.v11Fallback` 为真时生效。［已提交］
- 同一提交还：把 `evidence/` 下已跟踪的回执文件移出版本控制，并把 `.gitignore` 的 `/evidence/**/*.mat` 改为 `/evidence/`。［已提交，经 `git show --stat dd2ba68` 核对。本日志不推测这些改动的原因］

## 2026-11-12 · F1/F2 行为审阅

- 报告：`ENG5325_Fayne_F1_F2_Behaviour_Review.md`（工程目录之外）。［先前报告］
- 基线检查：V1 11/11、V1.1 11/11 测试在独立副本中通过；同种子结果可重复。［先前报告］
- 根因：已批准的 V1 在防守阶段只有中场有资格追球，且要同时满足三个条件（球在 1.2 m 内、球速低于 0.35 m/s、所有对手距球超过 0.6 m）。这是有意的阵型规则，过于保守，不是死锁。［先前报告］
- 无人追球时间过长：40 场 600 s 比赛中，防守阶段无追球者的时间占比 V1 为 93–95%、V1.1 为 92–94%（按时间合并）；V1.1 的“三次失败接管”在 40 场中从未触发；40 场中 V1 有 28 场、V1.1 有 32 场 0–0。［先前报告］
- 未决：先触球总是蓝队的具体机制未定位；种子只改变踢球噪声，开局相同。

## 2026-11-12 · V1.2 防守拦截与按队策略（原实现日期 2026-11-12；已于 2026-11-13 提交为 C1 `865ed1c`）

- 新增 `algorithms/+tdp/v12Config.m`、`v12TeamConfig.m`、`v12PolicyParams.m`、`tests/test_v12_policies.m`；修改 `v1TeamPlans.m`、`validateV1Config.m`、`scripts/v1MatchMetrics.m`。原实现时的工作树文件修改时间为 2026-11-12 15:31–15:41（当时尚未提交）。［工作树文件时间］
- **提交状态（C1）：已提交。** `865ed1c4c96ee549b7ca9b57b94a1f819f54825b`（短 `865ed1c`），2026-11-13 12:24:57 UTC，分支 `phase-e-b3-v11-fallback`，父提交 `dd2ba68`，标题“Add V1.2 per-team Defend interception policies”；Git 作者与提交者均为 MoonCanNotDrink。提交恰含上述 7 个文件（+698/−7），提交内容与此前验收的 SHA-256 逐一相同。［已提交，`git show 865ed1c` 核对］
- 单一拦截者：防守阶段每队每个控制拍最多选一名场上球员（后卫、中场；aggressive 策略还包括前锋）发起争球，取允许上抢者中离球最近的一人，同距离时角色编号小的优先；门将永不拦截。［先前报告］
- 拦截者滞回：当前拦截者保持任务，除非队友近出一个余量（neutral 0.4 m、aggressive 0.2 m、defensive 0.6 m，即 `v12SwitchMargin`）。［先前报告，参数见 `v12PolicyParams.m`］
- 按队策略参数：neutral、aggressive、defensive 是同一协调器 `tdp.v1TeamPlans` 的三组 `P.v12*` 数值，由 `tdp.v12TeamConfig(蓝, 红)` 按队写入，`P.v12Policy=[蓝 红]` 记录标签（0 中性、1 激进、2 防守）；每队 13 个参数，部分按角色取值。省略这些字段时走已批准的 V1 路径。［先前报告］
- 补位与解围：后卫上抢时，策略允许（`v12CoverDefender`）则中场回到球与本方球门之间；拦截者收到“朝对方球门踢”的普通意图，解围速度 1.5 m/s（不计作射门，射门门槛 1.8 m/s）。［先前报告］
- 测试与证据：`test_v12_policies` 16 项（16/16 为 2026-11-12 的先前报告；2026-11-13 在本仓库提交后又实际运行两次，C2 后 16/16、C3 后 16/16）；7 种配置 × 2 种步长 × 20 个种子共 280 场 600 s 比赛，`v1MatchMetrics` 0 违规（`F2_F3_evidence/long.csv`，修改时间 16:14）；防守无追球比例降到约 7%（neutral 6.6–8.0%）。［先前报告］
- 已知局限（来自先前报告）：无留出种子；空间阈值暂定，待 Yu 确认；aggressive 策略在 80 场换边比赛中 18 胜 18 平 44 负，未调优；中性对中性的失球约为 V1.1 的十倍（比赛变为双向开放）；HEAD 上另有 3 个既有测试失败，与本工作无关。
- 报告：`ENG5325_F2_F3_Implementation_Report.md`（2026-11-12）；交接文档 `F4_Qin_Metrics_Handoff.md`、`F4_Yu_Spatial_Handoff.md`。

## 2026-11-12 · Qin 离线行为指标 `qin-metrics-2`（原实现日期以 Qin 自述为准）

- Qin 的 `Qin_Fayne交接核对与默认半场规则.md` 标注更新日期 2026-11-12，称 18 项测试通过。［先前报告，Qin 自述］
- **提交状态（C2）：已导入本仓库。** `a9e5e1cbea4c2d94d7ff7d97866683b52bba63ab`（短 `a9e5e1c`），2026-11-13 12:41:10 UTC，父提交 `865ed1c`，标题“Import Qin offline behaviour metrics (qin-metrics-2)”。**Git Author 为 `QXS-1412 <qinxingsheng666@gmail.com>`**（已对照上游 `kai402063-dev/team7robot@319f57a96a9f888e0657e1f6073b6ac8b3558cf9` 的作者字段核实），Committer 为 MoonCanNotDrink；没有 `Co-Authored-By`，因为导入的是 Qin 的未改动原文件。恰含 3 个新增文件（+259）：`scripts/qinAnalyzeRecording.m`、`scripts/qinBehaviourMetrics.m`、`tests/test_qin_metrics.m`；三个文件的 Git blob 与上游 `qin_handoff/code/` 对应文件逐一相同。C2 提交后在本仓库实际运行：`test_qin_metrics` 18/18、`test_v12_policies` 16/16。［已提交；本机独立执行 2026-11-13］

## 2026-11-13 · Qin 指标集成验证（集成日期）

- Qin 仓库 `https://github.com/kai402063-dev/team7robot`，分支 `QXS-1412-patch-1` 提交 `319f57a96a9f888e0657e1f6073b6ac8b3558cf9`（2026-11-13 00:01:54，“Add files via upload”）。［先前报告］
- 导入的模块：`scripts/qinBehaviourMetrics.m`、`scripts/qinAnalyzeRecording.m`、`tests/test_qin_metrics.m`（与 `319f57a` 中对应文件逐字相同，先前核对过哈希）。模块纯离线，不运行也不改动仿真。［先前报告］
- 指标范围：时间对齐的机器人秒数；按半场（球心）的停留时间，中线 |x|≤1e-9 m 单列；追球（chasing）、闲置（idle）、物理静止（stationarity）和几何控球（possession）估计，几何控球仅为代理指标；Defend 无追球比例（分母为 `running & phase==Defend`）。决策相关量取自 `trace(k-1)`，物理量取自 `trace(k)`。［先前报告，Qin 源文件头注释］
- 在 scratch 副本（当时 V1.2 为未提交修改，加 Qin 三个 `.m`）上用 MATLAB R2026b 实际执行：`test_qin_metrics` 18/18、`test_v12_policies` 16/16、`test_v1_tactics` 11/11、`test_v11_fallback` 11/11；六组 60 s 冒烟共 204 项检查通过；对 80 场已保存 V1/V1.1 记录和 3 场 600 s V1.2 重跑，Qin 的无追球统计与 `long.csv` 精确一致（秒数差 ≤1e-13，比例差 ≤5e-5，受 CSV 四位小数舍入）；边界用例 20 项中 17 项通过。［先前报告，本机独立执行（2026-11-13）］
- **发现的缺陷**：Qin 模块未拒绝门将（ID 1/5）为授权追球者；未校验 `P.v12Policy`；Qin 的一个测试夹具使用门将 ID 1 作为追球者。原工作树未被修改。［先前报告］

## 2026-11-13 · Qin 指标模块校验加固（集成日期；**本次整合改动，非 Qin 原实现**；已提交为 C3 `4b28b82`）

- `scripts/qinBehaviourMetrics.m`：拒绝门将 chaser（蓝仅 `[0 2:4]`，红仅 `[0 6:8]`，错误标识 `Qin:Tactics`）；`P.v12Policy` 存在时必须为 1×2 数值且只含 0/1/2（`Qin:Policy`）；订正文件头对行对齐的描述（决策取自上一行 `trace(k-1)`，物理量取自 `trace(k)`）。
- `tests/test_qin_metrics.m`：新增 `testRejectGoalkeeperChaser`、`testValidatePolicyIds` 两项回归测试；`testStateOverlapAndMovement` 的夹具由门将 ID 1 改为后卫 ID 2，断言数值不变。
- 提交时在本仓库实际运行（MATLAB R2026b，2026-11-13）：`test_qin_metrics` 20/20、`test_v12_policies` 16/16、`test_v1_tactics` 11/11、`test_v11_fallback` 11/11，共 58/58；边界用例 20/20；六组 60 s 冒烟 204/204；有效输入上加固前后指标 344/344 逐位相同。［本机独立执行］
- **提交状态（C3）：已提交。** `4b28b823611b50d73507100725aab98d37f6d131`（短 `4b28b82`），2026-11-13 14:10:14 UTC，父提交 `a9e5e1c`，标题“Harden Qin metrics input validation and row alignment”；Git 作者与提交者均为 MoonCanNotDrink。恰含 2 个文件（+40/−6）。正文写明这是后续整合加固而非 Qin 原实现，原文件仍可在 `a9e5e1c` 中追溯。［已提交］
- 没有人工逐行审阅记录。**Qin 尚未完成团队代码审阅**，团队合并前需 Qin 确认是否采纳。

## 2026-11-13 · 全量集成验证（先前报告，本次文档任务未重跑）

- 全量回归（`tests/test_*.m`；HEAD 基线与 HEAD+四个补丁各运行一次，均提供 `evidence/` 与两份 animation 记录）：基线 187 项，184 通过、3 失败，失败的是 `test_manual_numbers` 三个既有缺陷（生成表与回执不一致，F2/F3 报告 §5.3）；打补丁后 223 项（187 + V1.2 的 16 + Qin 的 20），219 通过、4 失败：同样 3 个既有缺陷，另有 1 个耗时敏感的 `test_replay_player/testPlaybackRateScalesSimulationTime`，两个 MATLAB 并行运行时失败（8 倍速时仿真推进 10.7 s，超过该测试的上限），隔离重跑 12/12 通过、整个文件单独重跑 16/16 通过，判断为负载造成的偶发失败［推断，非证明］。新增 36 项全部通过；没有其他既有测试的结果被补丁改变。
- 未验证：无其他负载下对补丁树做完整单次全量重跑；无数据裸副本的回归；V1.2 的 A-A、D-D、其他种子和 dt=0.04 的 600 s 数值复现；280 场批量的独立复现。
- 证据：`ENG5325_Integration_Commit_Prep/evidence/`（工程目录之外）。

## 2026-11-13 · 文档整合（C4）

- 本批提交是 C1–C3 之后的文档维护：新增本日志；`DECISIONS.md` 追加 D029–D032；`PROGRESS.md` 增加状态段；把 V1.2、Qin 指标导入和加固的状态从“未提交/未导入”更新为 C1–C3 的实际提交。
- C4 只改文档，不改变 MATLAB/Simulink/Stateflow 行为，没有改战术参数，也没有重跑 MATLAB 测试；本日志中的测试数字来自上文标明来源的实际运行或先前报告。
- C4 自身的提交号不写在本文件里（文件无法包含自己提交的哈希）；以 `git log` 为准，仓库外的集成计划也有记录。

## 技术待办

1. Yu 对三条空间阈值（−1.5、−2.5、−1.0 m）和半场规则 H1（出界、暂停、重开转换）的确认——**尚未确认**。
2. Qin 对加固改动和提交署名方式的确认；Qin 的独立交叉验证。
3. V1.2 的 A-A、D-D、其他种子和 dt=0.04 的 600 s 复现；在未用过的种子上调 aggressive 参数。
4. 团队合并前：Qin 对 C3 的代码审阅；其余成员对 C1 的验收。
