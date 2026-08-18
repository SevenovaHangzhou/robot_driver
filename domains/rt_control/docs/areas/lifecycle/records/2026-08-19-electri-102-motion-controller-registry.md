---
id: lifecycle-20260819-03
area: lifecycle
title: ELECTRI-102 enable_manager motion controller 注册表
date: 2026-08-19
type: feat
trigger: ELECTRI-102 / T-06 / E102-D24
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/enable_manager/src/enable_manager_controller.cpp, src/rt_control/enable_manager/test/test_enable_manager_state_machine.cpp, src/rt_control/rt_control_bringup/config/controllers.yaml]
supersedes: []
related: [ELECTRI-102, lifecycle-20260814-02]
---

## 改动

把单一 `jtc_name` 参数替换为 `motion_controller_names` 注册表和
`default_motion_controller`。当前默认集合固定为 `whole_body_jtc` 与
`rolling_trajectory_controller`，普通 enable 目标仍是 `whole_body_jtc`。

configure 拒绝空集合、空名称、重复名称和默认目标未注册。普通失能、紧急失能和 enable
失败清理复用同一 switch 路径：先读取 controller_manager 状态，只把实际 ACTIVE 的注册成员
加入 deactivate；默认 enable 同时停用其他 ACTIVE writer。STRICT switch 成功后再次读取状态，
要求默认 enable 为“仅默认 ACTIVE”，失能为“全部 INACTIVE”；无法复核返回 ambiguous，由既有
路径锁存 restart-required，而不宣称自动回滚。

## 验证

- RED：4 个非法注册表配置在旧单参数实现中全部错误地 configure 成功。
- GREEN：非法矩阵全部 fail-closed；纯 switch-plan 测试证明 rolling ACTIVE→默认 enable 会
  同批 deactivate rolling/activate JTC，整组 disable 会列出两个 ACTIVE 成员。
- 复核矩阵覆盖仅默认 ACTIVE、全部 INACTIVE、双 ACTIVE 和成员缺失；只有前两种匹配各自目标。
- enable_manager 基线由 36 扩为 39 个 gtest，39/39 通过；原有 CiA402、并发、故障和失能
  状态机用例无回归。

本任务没有启动真实 controller_manager、访问总线、reset、enable 或运动。service 成功路径
和 launch mandatory-inactive 顺序在后续完整 mock bringup 任务验证。

## 冻结事实

- F1: enable_manager 的命令 writer 集合来自非空、无重复的显式注册表。
- F2: 普通 enable 默认只保留 whole_body_jtc ACTIVE；rolling 不自动激活。
- F3: 所有失能路径以“注册表成员全部 INACTIVE”的二次查询为成功条件。
- F4: controller-manager 结果无法复核时按 ambiguous/restart-required 收敛，不伪装事务回滚。
