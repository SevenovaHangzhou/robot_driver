---
id: contract-20260813-01
area: contract
title: /joint_states 频率裁决为实测 125 Hz（BQ-135）
date: 2026-08-13
type: decision
trigger: ELECTRI-77 mock 契约测试实测发现（工控机，Domain 142）
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [工控机 /home/ar/rt-control-dev/electri-77-evidence.log]
supersedes: []
related: [ELECTRI-77, BQ-135]
---

## 背景

launch_testing 实测 `/joint_states` 为 125 Hz，文档标称 100 Hz；250 Hz 控制环
将 100 量化为 125。用户裁决以实测为准。

## 改动

`docs/cross-domain-interfaces.md` R-OUT-03 改为 125 Hz 并注明量化关系；
BLOCKED-questions.md 追加 BQ-135。controllers.yaml 配置值不动（保护 diff_legacy
冻结基线）。

## 验证

- 已验证（T1）：实测 ~125 Hz（ELECTRI-77 套件，mock）。
- 未验证：实机频率（推断同为 125，待 T2 实测确认）；robot_interfaces 权威仓库
  与消费域的同步更新未执行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `/joint_states` 契约视图频率 = 125 Hz（实测口径）；配置标称 100 Hz 保留，
  两者关系为 250 Hz 环量化，见 BQ-135。

## 遗留

- 跨域通知：Motion/Perception/Autonomy 与 robot_interfaces 仓库待同步。
