---
id: chassis-control-20261122-01
area: chassis-control
title: 舵轮默认行驶方式改为连续模式（steer_while_driving）
date: 2026-11-22
type: decision
trigger: 用户 2026-11-22 要求底盘默认使用连续模式（ELECTRI-235）
commits: [feature/swerve-continuous-default]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261021-01, chassis-control-20261018-02]
---

## 背景

chassis-control-20261021-01 起默认 `drive_mode: align_first`：任一模块舵角误差超过 5° 时四轮停驱，
都在 3° 内才恢复。用户 2026-11-22 要求默认改为连续模式。`drive_mode` 仍在 configure 时读取，运行中不切换。

## 改动

`ctrl_swerve/config/controller.draft.yaml`：`drive_mode` `align_first` → `steer_while_driving`。
连续模式下各轮按 cos(舵角误差) 降速，误差超过 `alignment_threshold`（90°，未改）才停驱；
`align_first_threshold/release` 保留，只在 `align_first` 下生效。

## 验证

- 组合器测试与质量门禁（见 PR）。未验证：研华实机起步、换向与弧线行驶。

## 结论与冻结事实

- F1: 研华底盘默认行驶方式为连续模式（用户 2026-11-22）。

## 遗留

- 连续模式在转向未到位时行走轮已出力，起步与变向时轨迹偏离、横向擦地（chassis-control-20261018-02）；需现场确认。
- 导航运行中切换模式没有接口。
