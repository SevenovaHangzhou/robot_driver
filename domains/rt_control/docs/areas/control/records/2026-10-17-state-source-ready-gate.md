---
id: chassis-control-20261017-03
area: chassis-control
title: 外置编码器就绪状态与使能前置条件
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 决定：就绪=OPERATIONAL 后新帧且连续 50 ms
commits: [feature/electri-127-ready-gate]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261017-02]
---

## 背景

硬件组件 lifecycle 为 active 不代表数据源就绪。研华部署在编码器未就绪时会先使能八台驱动器，
再因控制器激活失败整体回滚。本次把“就绪”作为独立约定，在第一批使能之前检查。

## 改动

- ros2_canopen 补丁 `0007-expose-nmt-state-hook.patch`：CanopenSystem 增加可覆盖的
  `on_nmt_state_changed`，默认行为不变（仍写入 nmt_state）。native bootstrap 与 Dockerfile 按 0006 之后应用。
- SwerveEncoderFeedback：每节点 NMT 纪元；收到非 OPERATIONAL 报告（boot-up、pre-op、stop、reset）
  即作废已有样本，直到收到新帧。
- SwerveEncoderSystem：每个编码器新增 `ready` 状态；ready=样本晚于最近一次非 OPERATIONAL 报告
  且帧龄不超过 `ready_feedback_age_ms`（协议 profile 20 ms，约 5 个 4 ms SYNC 周期）。
  NMT 回调只写无锁反馈，不改基类 map。
- enable_manager：新增 `ready_state_interfaces`、`ready_hold_time`。所有数据源连续就绪满
  `ready_hold_time` 前，第一批不开始、控制字全为 0；批次超时内未就绪或使能过程中失去就绪 →
  `state_sources_not_ready` 并回滚。
- composer：从 scope 的 `required_state_modules` 收集带 `ready` 的传感器；底盘策略 `ready_hold_time: 0.05`。

## 验证

- `colcon test --packages-select enable_manager`：84 tests，0 failures（新增 3 个就绪门禁用例）。
- `colcon test --packages-select robot_hw_canopen`：18 tests，0 failures（新增 boot-up 作废与过期不就绪用例）。
- composer 测试：42 passed；`tools/tests/test_canopen_rpdo_hook.py`：3 passed；quality_gate PASS。

离线测试，未连接 CAN 总线。实际节点是否上报 START 不影响判据：判据只依赖“非 OPERATIONAL 报告之后的新帧”。

## 遗留

- 运行中失去就绪的收尾（控制器故障 → 轮速归零后有序禁能）见后续变更。
