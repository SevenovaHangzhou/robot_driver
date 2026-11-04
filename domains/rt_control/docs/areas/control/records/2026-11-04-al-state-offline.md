---
id: control-20261104-02
area: control
title: 按 EtherCAT 从站 AL 状态区分“复位即可”与“需要重启”
date: 2026-11-04
type: fix
trigger: 用户 2026-11-04“按 AL 状态来区分”；同日台架两次恢复供电结果不同
commits: [feature/arch-al-state-offline]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261104-01, control-20261103-01]
---

## 背景

2026-11-04 台架同一种断电（驱动报 0xF008 欠压）恢复供电后：第一次主站把全部从站带回 OP，复位即可；第二次左臂 1–5 轴停在 SAFEOP（“No Sync Error”），只能重启运行栈。过程数据年龄在第二次已恢复正常，所以只看反馈年龄判断不出来。零差手册没有按故障码给出“复位可清 / 需重启”的分类。

## 改动

- 渲染器为每个 EtherCAT 从站导出 `ethercat_slave_<n>/al_state`，组合器给使能管理器填 `al_state_interfaces`。
- 使能管理器把这些 AL 状态与反馈年龄一起作为“离线来源”：
  - 使能、复位前：任一从站不在 OP（8）→ 拒绝，阶段 `device_offline`，不锁存重启；从站回到 OP 后复位即可。
  - 已使能中：任一从站离开 OP → 急停失能、`device_offline`、要求重启（同 control-20261104-01）。
- 诊断新增 `offline_now`（每周期刷新的当前离线来源）与 `restart_required`；readiness 据此给出 `device_offline`（指明哪个从站）与 `restart_required`。

## 验证

单元测试：失败锁存后从站 SAFEOP → 复位被拒、不要求重启；回到 OP → 复位成功。使能中从站 SAFEOP → 失败、`device_offline`、要求重启。readiness：空闲时从站不在 OP 阻断并指明驱动；重启锁存为阻断项。未上台架。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 是否需要重启由从站能否回到 OP 决定，不按故障码分类（用户 2026-11-04）。
- F2: 未使能时从站不在 OP 只拒绝使能与复位，不锁存重启；使能中离开 OP 锁存重启。

## 遗留

- 台架：恢复供电后从站停在 SAFEOP 时，readiness 显示 `device_offline` 并指明从站，复位被拒；回到 OP 时复位成功。
