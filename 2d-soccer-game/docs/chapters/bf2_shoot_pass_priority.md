# B-F3：射门/传球优先级（Test 2 的实现）

本章对应设计 B-F2-01 至 B-F2-08（Notion「B-F2 · Shoot-first / Pass-first 设计决定」），实现基线为团队仓库 main `5eaa413`（`2d-soccer-game/` 与其后的 `1a33a5b` 相同）。设计不在这里重复，只记录代码落在哪里。

## 怎么打开

```matlab
P = tdp.bf2Config('shoot','pass');   % 蓝队 Shoot-first，红队 Pass-first
P = tdp.bf2Config('pass','shoot');   % 换边
P = tdp.v12Config();                 % 基线：无 B-F2，与 main 一致
```

`bf2Config` 在 `v12Config`（neutral 防守 + V1.1 前锋失效接管）上只加 `bf2*` 字段，唯一变量是各队的进攻优先级。字段不存在或某队优先级为 0 时，该队的计划与 main 逐行一致。

| 参数 | 初值 | 来源 |
|---|---|---|
| `bf2PassMin` / `bf2PassMax` | 0.5 / 2.0 m | B-F2-02，初始工程值，未调优 |
| `bf2SoftTicks` | 3 拍（0.15 s） | B-F2-05 |
| `bf2CommitTimeout` | 2.0 s | B-F2-05 |
| `bf2ReceiveTimeout` | 3.0 s | B-F2-06 |

## 代码在哪里

| 文件 | 作用 |
|---|---|
| `algorithms/+tdp/bf2Plan.m` | 选择动作、承诺与释放、接球锁与保持。核心逻辑都在这里 |
| `algorithms/+tdp/v1TeamPlans.m` | 仅 3 处挂钩：每拍重置日志、暂停/丢球时释放、追球者计划由 `bf2Plan` 覆盖。9 个输入且该队优先级非零时才生效 |
| `algorithms/+tdp/bf2TeamMemory.m` | `v1TeamMemory` 加 B-F2 状态。只有 `tdpV1Match` 使用，恢复实验模型的总线不变 |
| `algorithms/+tdp/bf2Config.m`、`validateV1Config.m` | 配置与校验 |
| `environment/V1Bf2Record.m`、`scripts/bf2TraceColumns.m` | 新日志信号 `bf2Records`（23 列，每拍一行），直接记录动作类型、可行性、承诺、锁、释放原因 |
| `scripts/v1MatchMetrics.m` | 审计白名单：优先级开启的队，DF/MF/ST 都可能成为追球者（接球者） |
| `scripts/build_match.m`、`build_v1_match.m` | 总线用 `bf2TeamMemory`，并加 `bf2Records` 信号 |
| `scripts/bf2OffCheck.m` | B-F2-08 硬性要求：关闭时 `records`（98 列）和 `tacticsRecords`（171 列）与 main 逐行一致 |
| `tests/test_bf2_priority.m` | 每条决定一个确定性场景（仿 B-Y3），加模型级测试 |

既有的 171 列 `tacticsRecords` 与 98 列 `records` 完全不变：很多脚本（回放、Qin 指标、导出）依赖这两个宽度，所以新增信号单独输出。

## 释放原因码（`bfRelease`）

1 终态反馈　2 追球者变化　3 离开 Attack　4 丢球　5 承诺超时　6 接球者/追球者不可用　7 软失效　8 暂停　9 传球出脚已确认（交权）　11 接球成功　12 被拦截　13 接球超时。同一拍有多个原因时只记第一个（根因）。

动作码（`bfAction`）：0 无，1 带球，2 射门，3 传球，4 接球。

## 实现时的选择（设计没写到的细节）

- **只在 Approach/Align 之外的拍里不计软失效**；一旦技能进入 Kick，不再做几何撤销，计划保持 Kick（接口要求 `plans(i).type` 仍是 Kick）。
- **不允许在 Kick 准备期内新建承诺**，否则带球踢的 Done 会被误认成传球出脚确认。承诺绑定的是进入 Kick 后那次请求的 ID，只有 ID 相同的 Done 才算出脚确认。
- 接球锁期间，接球者执行普通的带球追球（`dribbleSpeed`），不做新决策；接球成功后才由它做下一次进攻决策。
- 接球者保持追球权，直到它这次踢球有终态反馈、不可用、丢球、进入 Defend 或前锋接管；之后回到前锋。
- 传球者出脚后到接球者接球期间，前锋按原有默认形状站位（没有给它新的跑位规则）。
- "两者都可行时 100% 符合优先级"按**新决策那一拍**统计（`bfFresh==1 && bfBoth==1`）。已承诺的动作不会因为另一动作变得可行而互换，所以不能按每一拍统计。
- 承诺超时后按正常规则重新选择，仍可能再选同一动作；超时次数会出现在日志里，留给 B-F4 判断 2.0 s 是否合适。
