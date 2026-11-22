---
id: control-20261122-03
area: control
title: 研华底盘电机/外置编码器差阈值放宽到 2°
date: 2026-11-22
type: decision
trigger: 2026-11-22 研华整机低速转向时 module_3 电机角与外置编码器差 1.046°，超过 1° 报 feedback_fault 并整机失能；用户决定放到 2°（ELECTRI-233）
commits: [fix/swerve-encoder-2deg]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [chassis-control-20261018-01, hw-config-20261122-04]
---

## 背景

转向零点按 hw-config-20261122-04 在 0° 附近重算后，0° 处电机推算航向与外置编码器差为 0。整机转向到约 −65° 时
（失能读回）：module_1 +0.76°、module_2 −0.05°、module_3 −0.94°、module_4 +0.17°；运动中 module_3 达到
1.046°（`latched_fault_modules=module_3:encoder_difference`），超过 `max_encoder_difference`（1°）持续
`encoder_fault_window` 后锁存 `feedback_fault`。差值随角度增大且各模块符号不同，原因（齿轮回差或编码器比例/安装）未测。

## 改动

`ctrl_swerve/config/controller.draft.yaml`：`max_encoder_difference` 0.017453（1°）→ 0.034907（2°）。
该参数同时是激活时的编码器核对门限、运行中的编码器监视门限，以及保持中电机角偏离锁存目标的保持故障门限。

## 验证

- 组合器测试与质量门禁（见 PR）。未验证：实机转向全行程差值、回差与方向相关性。

## 结论与冻结事实

- F1: 研华底盘电机/外置编码器差与保持偏差门限为 2°（用户 2026-11-22，临时值，待测量后定正式值）。

## 遗留

- 正反向、−90°…+90° 多点测量各模块差值，区分回差与比例误差，再定正式门限或修正标定。
