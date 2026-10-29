---
id: motion-20260824-01
area: motion
title: 履带有效轮距调整为 0.95089496 m
date: 2026-08-24
type: decision
trigger: 用户明确要求将履带差速控制器轮距缩小为现值的 0.4852 倍
commits: []
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - "python3 -m pytest -q tools/tests/test_track_mechanics.py: PASS"
  - "tools/quality_gate.sh: PASS (193 tests, gated coverage 83%)"
  - "rt-control:ipc-main-sync-20260828 image build: PASS (28 packages)"
  - "image id sha256:d26242edc6f3d5dce1ea08e3c9d99f4d10ca035c10a6567baaa34c7b01bed8ab"
  - "Domain 142 hardware-free Docker Mock: PASS (devices=[], exit=0)"
supersedes: [canopen-chassis-20260821-01#F1-wheel-separation]
related: [BQ-139, BQ-140, REQ-CAN-003]
---

## 背景

BQ-139 将履带主动轮半径更正为 `0.1044 m`，并将当时的控制器轮距配置为
`1.9598 m`。用户后续明确要求将履带差速控制器中的轮距缩小为该值的 `0.4852`
倍。该裁决改变控制器有效轮距，不将其写成未经测量的物理机械尺寸。

## 改动

- `diff_drive_controller.wheel_separation`: `1.9598` → `0.95089496`，其中
  `1.9598 * 0.4852 = 0.95089496`。
- 同步更新履带机械参数回归测试。
- CANopen 主动轮半径、比例、方向、PDO/SDO 和限速均不变。

## 验证

配置回归测试、受影响包构建、仓库质量门禁、main 镜像构建与无硬件 Docker Mock 启停
通过。Mock 只证明安装后配置能被加载，不能证明实车转向比例或里程计精度。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `diff_drive_controller.wheel_separation` 冻结为 `0.95089496 m`。
- F2: 该值是用户裁决的有效轮距配置，不等同于已实测的物理轮距。
- F3: BQ-139 的 `0.1044 m` 主动轮半径、CANopen 比例和 `wheel_radius=1.0` 保持不变。

## 遗留

必须在低速、急停、隔离区和监护条件下，对比实测原地转角、角速度与 `/wheel/odom`；
完成前不得声明 T4 实机机械参数验收通过。
