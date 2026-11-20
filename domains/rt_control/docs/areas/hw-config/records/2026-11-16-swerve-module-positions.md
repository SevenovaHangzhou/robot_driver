---
id: hw-config-20261116-02
area: hw-config
title: 研华底盘轮组坐标取负，修正原地旋转方向反向
date: 2026-11-16
type: bugfix
trigger: 用户 2026-11-16 反馈直行方向正确、顺逆时针相反；IMU 对比确认后用户同意修改
commits: [bugfix/swerve-module-positions]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: [Advantech /tmp/rot_probe.jsonl（2026-11-16 只读采集）]
supersedes: []
related: [chassis-control-20261017-01, hw-config-20261116-01]
---

## 背景

研华底盘直行和平移方向正确，但原地旋转与指令相反。2026-11-16 只读采集（底盘栈 `arch-peripherals-20261109`，低速档 Q/E）：

| 指令 ωz | 轮式里程计 ωz | IMU ωz | 转向 |
| --- | --- | --- | --- |
| +0.20 | +0.16 | −0.17 | 旋转姿态 ±45°，编码器与电机角差 < 0.003 rad |
| −0.20 | −0.17 | +0.17 | 同一姿态，行走轮反向 |

线速度全程为 0，IMU 与现场目视一致。平移正确且旋转等幅干净反向，只有当四个轮组坐标同时取负 `(x, y) → (−x, −y)` 时成立：
转向姿态不变，只有行走方向反。只镜像一轴会导致轮间对抗，不会干净反转。里程计使用同一坐标，所以 ωz 也是反的。

## 改动

- `units/Alfa_V3-Advantech-01/swerve_calibration.yaml`：四个模块 `module_x_m/module_y_m` 全部取负：
  module_1 (+0.305, −0.305)、module_2 (−0.305, −0.305)、module_3 (+0.305, +0.305)、module_4 (−0.305, +0.305)。
  坐标系仍为 `base_footprint`，+X 为用户定义的新前方。
- 转向零点、编码器偏置、方向号、驱动器 `607E` 与 Robot Model 不变。

## 验证

- 离线：质量门与 composer/hw_ethercat 测试，结果见 PR。
- 未验证：修改后实机 Q/E 的 IMU 符号与指令一致、里程计 ωz 符号正确。需要重启后低速旋转，使能与运动需另行授权。

## 结论与冻结事实

- F1: 研华 Alfa_V3-Advantech-01 轮组在 `base_footprint`（+X 新前方）中的位置：module_1（ring 0/1）右前、
  module_2（ring 2/3）右后、module_3（ring 4/5）左前、module_4（ring 6/7）左后，各距中心 ±0.305 m。

## 遗留

- 实机旋转复测。
- 状态页底盘平面图的轮位来自 Robot Model `steerN_joint`，与本单机标定不同源；是否对齐另议。
