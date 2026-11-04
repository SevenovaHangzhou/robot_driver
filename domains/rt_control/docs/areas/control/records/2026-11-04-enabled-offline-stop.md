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
writes: {reset: yes, enable: yes, motion: no, plc: no}
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

单元测试：使能后反馈年龄 500.5 ms → 离开已使能、进入失败、失败阶段 device_offline、头部使能请求撤回。

台架 2026-11-04（`~/rt-control-arch-20261104b`，main + 本分支 + #188，作用域 bench_dual_arm_head，用户授权使能、复位、断电、拔网线；无运动指令）：

- 使能保持中断开双臂 + 头部电源：14 个驱动在断电过程中先报 0x603F=0xF008（母线欠压，ZeroErr 手册 p145），使能管理器按 `fault_detected` 在 3 ms 内急停失能并进入失败；过程数据随后才过期（约 150 ms 后）。本分支的过期路径未触发，因为驱动先报了故障。
- 第一次恢复供电（15:47:57）：驱动全部回到 OP，现场故障复位（15:51:10）回到空闲，不需要重启。
- 现场第二次断电、恢复供电（15:56:14）：左臂 1–5 轴停在 SAFEOP + “No Sync Error”，与 2026-11-03 相同，必须重启运行栈；按需自检报出这 5 轴 `RT_DEVICE_OFFLINE`（AL 状态 4）。同一种断电，恢复后是否需要重启取决于从站能否重新进入 OP，与故障码无关。
- 拔 EtherCAT 网线的试验未完成：运行栈在 16:17:28 被另一会话按正常流程停止，主站没有记录到断链。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 使能中反馈过期超过 500 ms，使能管理器急停失能，状态为失败且需重启运行栈（用户 2026-11-04 方案 A）。

## 遗留

- 在使能保持中突然中断通信（拔 EtherCAT 网线或只断头部电源），验证 `device_offline` 路径。
- 重启与复位的区分方式（按从站 AL 状态，还是按故障码）待用户决定。
