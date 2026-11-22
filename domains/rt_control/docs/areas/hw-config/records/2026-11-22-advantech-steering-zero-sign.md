---
id: hw-config-20261122-04
area: hw-config
title: 研华转向零点符号更正：关节角为外置编码器航向角取负
date: 2026-11-22
type: commissioning
trigger: 2026-11-22 整机首次使能 `jtc_activate_failed`，`swerve_controller` 激活被拒（ELECTRI-233，#321）
commits: [bugfix/steering-zero-sign]
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [hw-config-20261122-02]
related: [hw-config-20261122-02, control-20261017-01]
---

## 背景

hw-config-20261122-02 按外置编码器重算转向零点时令关节角 `q` 等于外置编码器角。机型约定为
`heading = -q`（模型转向关节绕 X 翻转 π，`model_heading_sign = -1` 导出为 `steering_to_yaw_sign`，
见 control-20261017-01），外置编码器给的是航向角，所以应为 `q = -外置角`。四轮都在 0° 附近，目视无法区分。

`swerve_controller` 激活时要求 `|q·sign − 外置角| ≤ max_encoder_difference`（1°）。实际差为两倍角度
（2.5°–6.1°），激活返回 ERROR，整机使能以 `jtc_activate_failed` 回退。

## 改动

`units/Alfa_V3-Advantech-01/swerve_calibration.yaml` 的 `steering_zero_counts`（`position_counts_per_unit`
= −1460252.97 计数/rad；新零点 = 旧零点 + 计数/rad·(q + 外置角)）：

| 模块 | 失能时 q（rad） | 外置角（rad） | 旧零点 | 新零点 |
| --- | --- | --- | --- | --- |
| module_1 | −0.021906 | −0.021861 | −25747 | 38164 |
| module_2 | −0.022875 | −0.022910 | 11433 | 78291 |
| module_3 | +0.028449 | +0.028339 | 54374 | −28551 |
| module_4 | −0.053072 | −0.053042 | −53061 | 101892 |

q 与外置角为栈运行、驱动失能时 `/rt_internal_state_broadcaster/dynamic_joint_states` 的 565 个样本均值。
停栈后 SDO 读 `0x6064`（6242 / 44837 / 12831 / 24438）按新零点换算的 q 与 −外置角之差小于 1e-5 rad。

## 验证

- T2 只读：上述读回与换算。
- 未验证：使能后 `swerve_controller` 激活、低速转向方向与角度（ELECTRI-233 T3/T4）。

## 结论与冻结事实

- F1: 研华转向零点满足 `q = −外置编码器航向角`；hw-config-20261122-02 的 F2 零点作废。
- F2: 研华 2026-11-22 转向零点为上表新值。

## 遗留

- 整机使能失败时，已激活的 `arms_jtc` / `head_position_controller` 未被撤下，`swerve_controller` 停在
  unconfigured，需重启栈（enable_manager 另立缺陷）。
