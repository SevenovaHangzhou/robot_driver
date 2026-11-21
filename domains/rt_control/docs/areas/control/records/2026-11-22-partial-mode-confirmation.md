---
id: control-20261122-02
area: control
title: enable_manager 可只确认部分关节的启动模式（期望模式 0 表示不确认）
date: 2026-11-22
type: feature
trigger: 研华整机（ELECTRI-239）：底盘需要确认 CSV/CSP 启动模式，双臂 ZeroErr 驱动的 PDO 没有模式显示
commits: [feature/partial-mode-confirmation]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [control-20261122-01]
---

## 背景

`expected_modes` 必须覆盖每个受管关节，且每个关节都要有 `mode_of_operation_display`。整机里底盘的行走轮模式切换依赖启动模式确认，而双臂 PDO 不含 0x6061。

## 改动

- `ctrl_enable_manager`：`expected_modes` 允许 0，表示该关节不确认启动模式；不认领它的模式显示接口，使能和运行中的模式检查跳过它。被模式切换的驱动必须有非 0 的启动模式。
- 使能策略：`enable_groups` 每项可写 `mode_confirmation`（`none`/`startup`），覆盖策略级设置；组合器对不确认的关节输出 0。

## 验证

- `ctrl_enable_manager` 新增 3 项（不认领模式显示、不确认关节不触发模式检查而确认关节仍会触发、被切换驱动拒绝 0），全部 132 项通过；组合器新增按组确认测试。
- 未验证：mock 与实机。

## 结论与冻结事实

- F1: `expected_modes` 中 0 表示不确认该关节启动模式，不需要其模式显示接口。
