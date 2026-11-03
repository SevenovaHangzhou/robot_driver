---
id: control-20261103-02
area: control
title: 设备离线判据阈值改为 500 ms
date: 2026-11-03
type: decision
trigger: 用户 2026-11-03
commits: [feature/arch-offline-limit-500]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [control-20261103-01]
related: []
---

## 背景

control-20261103-01 的离线阈值为 100 ms；用户改为 500 ms，与机械臂轨迹控制器、运动模式切换的反馈年龄限值一致。

## 改动

`ctrl_enable_manager` 参数 `feedback_age_limit_ms` 默认值 100 → 500；其余行为不变。

## 验证

`ctrl_enable_manager` 测试（离线样本改为 500.5 ms）。未上台架：运行中的栈重启后生效。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 反馈年龄超过 500 ms 时，使能与故障复位均不开始，报 `device_offline`（用户 2026-11-03）；取代 control-20261103-01#F1。

## 遗留

无。
