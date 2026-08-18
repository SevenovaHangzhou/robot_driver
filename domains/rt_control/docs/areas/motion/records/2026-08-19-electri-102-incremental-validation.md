---
id: motion-20260819-05
area: motion
title: ELECTRI-102 rolling 增量后缀校验
date: 2026-08-19
type: perf
trigger: ELECTRI-102 / T-10 / D-13
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [domains/rt_control/docs/electri-102-incremental-validation-proof.md, src/rt_control/rolling_trajectory_controller/test/test_rolling_correctness.cpp]
supersedes: []
related: [ELECTRI-102]
---

## 结论

前置论证成立，普通 replacement 只从内部 splice right state 开始验证新 suffix 连续段；Prime
继续全量验证。完整证明见 `electri-102-incremental-validation-proof.md`。

生产路径不执行全量 oracle。test-only friend peer 对同一 candidate 执行全量校验：合法长前缀
加两点 suffix 时增量仅检查 1 段；破坏新末点时增量与全量均返回 `PositionLimit`。完整 rolling
CTest 12/12 通过，没有 schema 或拒绝顺序变化。

没有访问现场总线、reset、enable 或运动。

## 冻结事实

- F1: Prime 全量校验；普通 replacement 只检查新 suffix 动态段。
- F2: checker 若增加跨段状态，本论证失效，必须 fail-closed 恢复或重新证明。
- F3: production 不执行“增量后再全量”的双重校验。
