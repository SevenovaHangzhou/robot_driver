---
id: release-deploy-20260919-03
area: release-deploy
title: 达妙手册寄存器只读准入校验
date: 2026-09-19
type: feature
trigger: GitHub Issue #40（用户要求对照头部-达妙.pdf 完善双电机程序）
commits: [feature/rt-control-head-can-integration]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-150, release-deploy-20260919-01]
---

## 背景

DM-J4310-2EC V1.1 手册第 15-18 页列出只读寄存器协议、控制模式、CAN ID、Master ID、CAN 超时及波特率代码；这些是协议事实，不是已安装机构的机械安全值。

## 改动

`robot_hw_can` 的 configure 阶段对两台电机额外读取 0x08 (ESC_ID)、0x07 (MST_ID)、0x09 (CAN TIMEOUT) 和 0x23 (CAN 波特率代码)。仅在 ID 与配置一致、波特率代码为 4 (1 Mbit/s)、CAN 超时非零且原有模式 2 与 PMAX/VMAX/TMAX 校验通过时继续；不写电机寄存器，不改变波特率。新增协议单测覆盖寄存器地址和拒绝条件。

## 验证

`colcon build --symlink-install --packages-select robot_hw_can` 通过；`colcon test --packages-select robot_hw_can` 与 `colcon test-result --verbose --test-result-base build/robot_hw_can/test_results` 为 11 tests、0 errors/failures/skipped。`tools/quality_gate.sh` 为 286 passed、13 skipped；`git diff --check` 通过。未接入或读取实机，也未测试共享总线负载和真实驱动器参数。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 仅在启动前只读核对每台电机 ID、1 Mbit/s 代码及非零 CAN 看门狗；手册未定义安装后的零位、关节方向、机械限位和安全速度，不用手册示例值替代。

## 遗留

BQ-150 共线准入、现场确认看门狗实际周期（每计数 50 us）与控制周期/反馈超时关系、机械方向/零位/限位、速度及正式模型仍待验收；零值看门狗会拒绝 configure，须由授权调试流程处理。
