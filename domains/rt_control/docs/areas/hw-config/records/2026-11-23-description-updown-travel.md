---
id: hw-config-20261123-01
area: hw-config
title: 升降行程两端按实测更正为 -1.033..+0.017 m（零点不变）
date: 2026-11-23
type: commissioning
trigger: 用户 2026-11-23 现场确认升降行程两端在当前关节坐标中的读数（ELECTRI-241）
commits: [chore/sync-description-travel]
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [hw-config-20261122-03]
related: [protocol-20261122-01]
---

## 背景

hw-config-20261122-03 按 11-22 卷尺推算的零点，把升降行程写成 Robot Model `[-1.05, 0]` m。2026-11-23 用户在
当前关节坐标（`zero_counts` 8887150 不变）下确认：读数 -1033 mm 是行程底端（机械硬限位上方 2 cm），+17 mm 是
行程顶端，总长 1.05 m。整段相对模型限位偏 +17 mm；用户决定改模型限位，不改零点。

## 改动

- robot_description#23 已将 `updown` 限位改为 `[-1.033, 0.017]` m。合并前按用户裁决保留主线正式装配模型及其后续轮组修正 `a41105c`（robot_description#27），不再导入旧快照 `405f9ee`，模型和金样不变。
- `machines/Alfa_V3/updown.yaml`：`stroke_min_m` −1.05 → −1.033，`stroke_max_m` 0 → 0.017；`hard_stop_margin_m`
  0.02 不变，启动预检原始位置窗口为 −1.053..+0.037 m。
- 只补齐硬件行程与预检范围，使其与当前正式模型一致。

## 验证

- 组合器、EtherCAT、诊断、bringup 测试（见 PR）。
- 未验证：网页按新范围走到两端；规划回归。

## 结论与冻结事实

- F1: 研华升降在当前关节坐标（`zero_counts` 8887150）中的行程为 −1.033..+0.017 m（用户 2026-11-23）。

## 遗留

- 11-23 用户报告指令 1.04 m 实测 0.978 m 的位移比例差未查明（与本次行程两端无关）。
