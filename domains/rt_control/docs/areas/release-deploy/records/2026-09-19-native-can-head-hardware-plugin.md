---
id: release-deploy-20260919-01
area: release-deploy
title: 双达妙头部电机原生 CAN 硬件包
date: 2026-09-19
type: feature
trigger: GitHub Issue #40（pciecan2 双电机 ros2_control 原生 CAN 包命名 robot_hw_can）
commits: [feature/rt-control-head-can-integration]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-150]
---

## 背景

pciecan2 上两台达妙电机 CAN ID 1/2、Master ID 0x11/0x12；其原生 CAN 协议不同于已有 CANopen 履带主站。

## 改动

新增 `robot_hw_can` C++ SystemInterface、SocketCAN/位置速度帧编解码和控制器/Xacro 模板；草案 owner_ref 从 `dm_swerve_driver` 改为 `robot_hw_can`。未改生产 14 轴控制器、模型或启动入口。

## 验证

`robot_hw_can` 在 `/tmp` 隔离目录构建通过，包测试 11 项通过；相关 Python/Xacro/原生脚本聚焦测试 152 项通过；`rt_control_bringup` 隔离构建通过，包测试 165 项通过；`tools/quality_gate.sh` 286 通过、13 跳过。公共接口使用固定 SHA `92d6ff2ed0b45684d7da2170d96703ca8be569f4`，构建和测试产物均未写入仓库。生产共享总线、机械限位、时序和真实激活均未验证。本记录不授权使能或运动。

## 结论与冻结事实

- F1: 头部达妙硬件适配归属 `robot_hw_can`，使用原生 CAN 而非 CANopen；生产运行仍为 draft。

## 遗留

BQ-150 共线准入；正式关节名、机械方向/零位/限位、速度和反馈/看门狗时序的确认；Mock 联调及独立授权实机验收。
