---
id: control-20261104-01
area: control
title: 使能中反馈过期时急停失能并要求重启
date: 2026-11-04
type: fix
trigger: 用户 2026-11-04；2026-11-03 台架驱动在使能中断电，使能管理器仍认为已使能
commits: [feature/arch-enabled-offline-stop]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261103-01, control-20261103-02]
---

## 背景

驱动断电后过程数据不再更新，状态字停在断电前的“运行使能”，使能管理器的状态检查看不出变化。

## 改动

已使能（含激活/停用轨迹控制器）阶段每周期检查 `feedback_age_interfaces`，任一超过 `feedback_age_limit_ms`（500 ms）即走急停失能路径，失败阶段记为 `device_offline`，并置“需要重启”：重新上电的 EtherCAT 驱动不能在 OP 中重新同步（2026-11-03 dmesg “No Sync Error”）。

## 验证

单元测试：使能后反馈年龄 500.5 ms → 离开已使能、进入失败、失败阶段 device_offline、头部使能请求撤回。实机需现场断驱动电验证，另行授权。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 使能中反馈过期超过 500 ms，使能管理器急停失能，状态为失败且需重启运行栈（用户 2026-11-04 方案 A）。

## 遗留

台架断电验证。
