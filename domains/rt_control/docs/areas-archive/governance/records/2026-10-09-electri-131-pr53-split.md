---
id: governance-20261009-02
area: governance
title: ELECTRI-131 从 PR 53 分离 BMS
date: 2026-10-09
type: corrective
trigger: 用户要求 BMS 独立修复；ELECTRI-131；GitHub PR 53
commits: [bugfix/electri-131-pr53-remainder]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [/tmp/electri131-remainder-quality.log]
supersedes: []
related: [ELECTRI-131]
---

## 背景

PR #53 的 BMS 范围拆出，其他改动保留。
双电池修复与 auto 字节序修复分别评审。

## 改动

- 本分支恢复 BMS 包、rt_io 电池配置和对应契约测试到 main 基线。
- 双电池流程、进度和 BQ-152 补充随独立 BMS 分支迁移。
- 头部、IMU、LED、力传感器及 enable_manager 测试修复保持原提交内容。
- 原程序优化验证记录保留为历史证据。其中的 BMS 测试不代表本分支继续交付双电池控制。

## 验证

- `tools/quality_gate.sh`：通过。
- 对照 PR 原 head 检查非 BMS 源码，保持一致。
- 本轮不重复执行未改动模块的实机验证。
- 本轮完整构建证据以拆分提交触发的 CI 为准。

本记录不授权使能或运动。

## 结论与冻结事实

- F1：PR #53 的最终范围不含双电池控制与字节序修复。
- F2：拆分不改变其他模块的硬件准入边界。

## 遗留

独立 BMS PR 仍受 ELECTRI-131 的系统策略裁决与实机准入限制。
