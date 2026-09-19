---
id: release-deploy-20260919-02
area: release-deploy
title: 头部原生 CAN 硬件 opt-in 接入总装
date: 2026-09-19
type: feature
trigger: GitHub Issue #40（把 robot_hw_can 并入 rt_control_bringup，并按 CAN2 接线配置控制器）
commits: [feature/rt-control-head-can-integration]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-150, release-deploy-20260919-01]
---

## 背景

两台头部电机已登记 CAN ID 1/2、Master ID 0x11/0x12；现有总装只组合 EtherCAT 与 CANopen。

## 改动

总装可选择加载 `robot_hw_can/DamiaoSystem`；`controllers.yaml` 为两台电机配置同一个状态广播器和默认 inactive 的双关节轨迹控制器。硬件插件要求两轴同时切换命令接口，不允许单独使能一台电机。选择头部配置时校验两轴名称、机械范围、速度上限和时序值。原生启动脚本始终把 PCI L2 命名为 `can2`；仅 opt-in 时要求 L2 已是 1 Mbit/s 并将其启动，不改其波特率。CAN0/CAN1 仍是 500 kbit/s。缺省启动保持 L2 DOWN，自动掉电恢复不接纳头部配置。

## 验证

已执行相关 Python/Xacro/原生脚本聚焦测试，152 项通过；`tools/quality_gate.sh` 286 通过、13 跳过，`robot_hw_can` 单包构建与 11 项测试成功。`rt_control_bringup` 在 `/tmp` 隔离目录构建成功，并在固定 SHA `92d6ff2ed0b45684d7da2170d96703ca8be569f4` 的公共接口和已验收控制器依赖 overlay 上通过 165 项测试（含无硬件 Mock），无失败或跳过。未执行实机、上电、使能或运动验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 头部插件通过显式配置加入现有 controller_manager；控制器 YAML 是同一总装的配置，不另起进程。
- F2: CAN2 对应 PCI L2 的运行名 `can2`，仅 opt-in 时在确认已有速率为 1 Mbit/s 后启动；不写入 L2 波特率，历史两路名称和速率不变。

## 遗留

BQ-150 共线准入、正式机械零位/方向/范围和超时参数、模型头部关节、整机 Mock 与经授权实机验收。
