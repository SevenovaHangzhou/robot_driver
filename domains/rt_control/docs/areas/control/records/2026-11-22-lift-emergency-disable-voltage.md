---
id: control-20261122-04
area: control
title: 同毅升降急停改发 Disable Voltage（0x0005），不发快速停止
date: 2026-11-22
type: fix
trigger: 研华整机故障停机后升降轴长时间停在 Operation Enabled（0x8637），复位多次 fault_reset_timeout joint=updown（ELECTRI-242）
commits: [fix/lift-no-quick-stop]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261122-01, hw-config-20261122-05]
---

## 背景

2026-11-22 研华（Robot_sev `runtime/evidence/chassis-20261122T154524.csv` 等）：

- 正常 `/rt/disable`（0x0007 → 0x0006 → 0x0000）时升降在同一秒变为 0x8270（Switch On Disabled）。
- 故障停机（`fault_detected` → 紧急停机）先发快速停止 0x0002，再逐级失能；升降一直回报 0x8637
  （Operation Enabled），之后失败/空闲阶段持续 0x0000 也不退出，17:13–17:20 持续 7 分钟以上；
  三次复位 `fault_reset_timeout joint=updown`。直到下一次使能流程开始它才退出。
- 同毅协议手册 v1.4：0x0002 为“QuickStop，电机快速停止，电机通电状态”，0x0005 为“退出伺服使能，
  电机断电”，6041 Bit5 标为保留；ESI（XMC_ESC_250325）没有 0x605A–0x605E 与 0x6085。
- 厂家经用户确认（2026-11-22）：0x0005 立即断使能、退磁、抱闸；可以不用快速停止。

## 改动

- `ctrl_enable_manager`：新参数 `emergency_disable_voltage_joints`（默认空，须为受管关节的不重复子集）。
  紧急停机的快速停止阶段对这些轴不发 0x0002：未失能发 0x0005（Disable Voltage，快速停止位为 1），
  已 Switch On Disabled 发 0x0000；其他轴不变。判断在 `enable_manager_emergency_stop.cpp`。
- `enable_policy.schema.json`：可选字段 `emergency_disable_voltage_joints`。
- `Alfa_V3_advantech_enable_policy.yaml`：`emergency_disable_voltage_joints: [updown]`，并加入
  `runtime_views.normal.parameter_names`。
- `devices/tongyi_xmc_esc.yaml`：记录停机命令事实。
- 正常失能（0x0007 → 0x0006 → 0x0000）不变。

## 验证

- `ctrl_enable_manager` 单元测试：状态机 97、握手设备 17、模式切换 9、运动组 11 全部通过；新增用例：
  另一轴故障触发紧急停机时 `updown` 收到 0x0005、其余 0x0002，最终 `updown` Switch On Disabled、命令 0x0000；
  未知或重复关节配置被拒。
- 组合器测试：整机参数含 `emergency_disable_voltage_joints: [updown]`。
- 未验证：研华实机故障停机时升降 1 s 内退出使能、抱闸吸合，之后复位与再使能。

## 结论与冻结事实

- F1: 同毅 XMC_ESC 急停只用 0x0005（立即断使能、退磁、抱闸，厂家 2026-11-22），不用快速停止 0x0002。
- F2: 研华整机紧急停机对 `updown` 发 0x0005，其他轴仍先快速停止。

## 遗留

- 同毅在已失能后收到 0x0000（快速停止位为 0）的行为未经厂家确认；实测已失能状态下保持失能。
- 运动中 0x0005 直接抱闸的冲击需现场观察。
