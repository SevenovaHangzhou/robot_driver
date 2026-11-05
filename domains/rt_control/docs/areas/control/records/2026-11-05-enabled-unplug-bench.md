---
id: control-20261105-04
area: control
title: 使能保持中拔 EtherCAT 网线：急停失能、锁存重启、重启后复位恢复（台架）
date: 2026-11-05
type: commissioning
trigger: control-20261104-01、control-20261104-02 的遗留台架项；用户 2026-11-05 授权使能、保持、拔线、恢复、复位/重启
commits: [d125022]
env: native
risk: T3
writes: {reset: yes, enable: yes, motion: no, plc: no}
verified: PASS
evidence: [ar@192.168.100.40:~/rt-control-arch-20261105c/runs/arms-head-after-unplug-20261105-211248/unplug-timeline.jsonl]
supersedes: []
related: [control-20261104-01, control-20261104-02]
---

## 背景

control-20261104-01 的断电试验中驱动先报欠压，过期路径未被触发；拔线试验当天未完成。

## 改动

无代码改动。

## 验证

台架 `bench_dual_arm_head`（main d125022 + 头部 CAN ID 互换），双臂与头部已使能、静止保持，现场有人守急停。时间相对拔线：

- 0 s：主站只剩 10 个从站可见，其余 AL 状态读不到。
- +0.9 s：使能管理器 `EMERGENCY_QUICK_STOP`，阶段 `device_offline`，`restart_required=true`，离线来源 `ethercat_slave_1/al_state` 与过程数据年龄；readiness `device_offline, not_enabled, restart_required`，错误 `ethercat/master0 RT_DEVICE_OFFLINE`。
- +4.9 s `EMERGENCY_DISABLE`，+8.9 s `FAILED`；readiness 指明 `bench_dual_arm/left_joint1/drive` 离线。
- 插回网线后约 14 s 主站把 15 个从站带回 OP（中途多个从站短暂 SAFEOP+ERROR）。此时 `/rt/reset_fault` 被拒绝，`stage=restart_required`。
- 重启运行栈：从站全部 OP；零差驱动锁存 0xA000（主站掉线），readiness 按 `RT_DEVICE_FAULT_RESETTABLE` 列出；一次 `/rt/reset_fault` 成功回到 IDLE，readiness 仅 `not_enabled`。

本次实机操作经现场授权，操作人：用户（现场守急停）与 Agent，确认短语入口：用户 2026-11-05 授权“使能双臂和头部……根据系统状态执行 Fault Reset 或 Stack Restart”。

## 结论与冻结事实

- F1: 使能中通信中断由过期/AL 状态路径在 1 s 内急停失能并锁存重启；从站回到 OP 也不能用复位解除，必须重启（与 control-20261104-02 F2 一致）。
- F2: 重启后零差驱动的 0xA000 由一次故障复位清除。

## 遗留

- 只断头部电源的场景未做。
