---
id: canopen-chassis-20260821-01
area: canopen-chassis
title: 履带主动轮半径与轮距更正
date: 2026-08-21
type: decision
trigger: 用户明确更正主动轮物理半径并要求轮距改为原值 2.39 倍
commits: []
env: main-candidate
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - "python3 -m pytest -q tools/tests/test_track_mechanics.py: PASS"
  - "tools/quality_gate.sh: PASS"
related: [BQ-064, BQ-139, REQ-CAN-003]
---

## 背景

此前 BQ-064 将 `0.2088 m` 解释为主动轮半径。用户现明确更正物理半径为
`0.1044 m`，即 `0.2088 m` 是直径；轮距同时由 `0.82 m` 更正为其 2.39 倍，
即 `1.9598 m`。

## 改动

- Node 2/3 的位置、速度 to-device 比例：`-304894.5269959681` →
  `-609789.0539919361`。
- Node 2/3 的位置、速度 from-device 比例：`-3.27982273034774e-6` →
  `-1.63991136517387e-6`。
- 位置偏移保持零，`diff_drive_controller.wheel_radius=1.0` 保持不变；
  `wheel_separation=1.9598`。
- 新增按 `40 * 10000 / (2*pi*0.1044)` 推导比例并冻结轮距的回归测试。

## 验证

测试先在旧比例下按预期 RED，修改后 GREEN。focused pytest、YAML 解析、仓库质量门禁、
main 镜像构建及无硬件 Mock 启停均通过；Mock 不证明实车机械比例。本任务未发送 CAN
控制帧、reset、enable、运动或 PLC 输出。

## 结论与冻结事实

- F1: 主动轮物理半径为 `0.1044 m`、直径为 `0.2088 m`，履带轮距为 `1.9598 m`。
- F2: 左右履带位置与速度共用精确且互逆的米制换算；diff-drive 逻辑半径保持 `1.0`。
- F3: 相对旧配置，直行 raw 需求约 2 倍；同角速度原地转向 raw 需求约 4.78 倍。

## 遗留

必须在受控低速运动中复验实车直行距离、速度、左右方向、原地转向角度和里程计比例；
完成前不得声明 T4 实机验收通过。
