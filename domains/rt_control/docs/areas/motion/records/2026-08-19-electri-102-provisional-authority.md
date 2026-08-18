---
id: motion-20260819-08
area: motion
title: ELECTRI-102 provisional 与 test-only 限值授权隔离
date: 2026-08-19
type: feat
trigger: ELECTRI-102 / T-14 / E102-D34
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rolling_trajectory_controller/include/rolling_trajectory_controller/limit_checker.hpp, src/rt_control/rolling_trajectory_controller/src/limit_checker.cpp, src/rt_control/rolling_trajectory_controller/test/test_interpolation_and_limits.cpp]
supersedes: []
related: [ELECTRI-102]
---

## 改动

`LimitsSource` 增加与 public `RollingLimitsSource.PROVISIONAL=3` 一致的
`kProvisional`。`LimitChecker`、`RollingBuffer` 和 `StopTrajectory` 增加独立的
`allow_provisional_limits` 授权；它与 `allow_test_only_limits` 互不替代。

production 始终允许进入字段校验；test-only 只由 test-only opt-in 放行；provisional 只由
provisional opt-in 放行。所有来源仍必须有非零版本，并通过 14 轴有限值、正限值和安全位置
区间完整校验。

## 验证

- 先加入三态授权矩阵测试，确认旧实现因缺少 `kProvisional` 和第三个授权参数编译失败。
- 实现后 targeted test 通过；rolling 完整 CTest 12/12 通过。
- 显式验证 test-only 开关不能放行 provisional，provisional 开关也不需要放行 test-only。

没有加载 provisional 数值文件，也没有访问现场总线、reset、enable 或运动；YAML 来源、文件
SHA-256、controller 启动 WARN 和 public state 持续暴露在后续原子任务闭环。

## 冻结事实

- F1: provisional 与 test-only 使用独立授权，任何一个开关不得扩大另一个的权限。
- F2: public/local limits-source 枚举值固定一致，provisional 为 3。
- F3: opt-in 只控制来源准入，不绕过版本或 14 轴字段合法性检查。
