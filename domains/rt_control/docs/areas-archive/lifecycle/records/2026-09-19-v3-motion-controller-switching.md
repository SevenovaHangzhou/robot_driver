---
id: lifecycle-20260919-01
area: lifecycle
title: V3 十六轴使能与运动控制器严格切换
date: 2026-09-19
type: feature
trigger: ELECTRI-102；V3 使能后必须交给 Motion，故障时停止当前 writer
commits: [7d2600ac7a4e91e6e30413e0bdcd8b30739a3ee1]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [motion-20260919-01, contract-20260919-01]
---

## 背景

旧 enable_manager 只认单个 JTC，二代 rolling 实现又写死 14 轴拓扑，均不能直接用于 V3。

## 改动

保留通用 16 轴 CiA402 topology，增加可选 14 轴 motion registry、反馈年龄/稳态/接管误差
准入和 JTC/rolling STRICT 切换；enable 默认激活 JTC，disable/fault 停止实际 ACTIVE writer。

## 验证

enable_manager 65 tests 通过，包括伪 controller_manager 的完整 STRICT 切换与幂等结果；
Mock 初态为 enable_manager ACTIVE/IDLE、两个 motion controller INACTIVE。本记录不授权使能或运动。

## 结论与冻结事实

- F1: V3 enable_manager 管理 14 CSP + 2 PP，但 motion controller 只认领 14 CSP position。
- F2: 使能成功后默认进入 FJT_READY；JTC 与 rolling 任一时刻最多一个 ACTIVE。
- F3: 模式切换歧义进入失能收敛，必要时锁存 restart-required，不自动恢复 writer。

## 遗留

真实驱动状态转换、故障中断和有序停机仍需无运动及低速现场验证。
