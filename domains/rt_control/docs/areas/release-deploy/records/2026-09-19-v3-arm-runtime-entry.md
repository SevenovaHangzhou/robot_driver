---
id: release-deploy-20260919-01
area: release-deploy
title: V3 机械臂运动 runtime 显式入口
date: 2026-09-19
type: feature
trigger: ELECTRI-102；V3 需要独立于 validation/enable-only 的运动入口
commits: [feature/v3-arm-motion-runtime]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [motion-20260919-01, lifecycle-20260919-01]
---

## 背景

默认 V3 启动保持 validation-only，需要显式选择机械臂运动 runtime，且加载失败必须退出。

## 改动

新增 `rt_control_arm_runtime.launch.py` 和 `rt_control_start --arm-runtime`；先加载 JTC/rolling
为 INACTIVE，再激活 broadcaster 与 enable_manager。真实 builder 要求完整校准，缺失时拒绝。

## 验证

Mock launch 以 1 kHz 启动并停在 IDLE，FJT/rolling 生命周期和 state 发布通过；退出后无
`ros2_control_node` 遗留。冻结上游补丁按顺序 apply-check 通过。本记录不授权使能或运动。

## 结论与冻结事实

- F1: 默认 `rt_control_start` 仍为 validation；运动入口只能通过显式 `--arm-runtime` 选择。
- F2: mandatory motion controller 配置失败会关闭整个 launch，不留下半套可使能系统。
- F3: 当前旧工控机 `1256885` release 未包含本实现，current link 未改变。

## 遗留

当前分支尚未在旧工控机旁路构建；本机缺 EtherLab，真实 EtherCAT 闭包由目标环境验证。
