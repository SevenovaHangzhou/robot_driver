---
id: contract-20260918-01
area: contract
title: V3 机械臂 RT-Control 与 Motion 对接方案
date: 2026-09-18
type: decision
trigger: 用户要求基于 robot_driver/v3 输出三代机机械臂与运控对接文档
commits: [docs/v3-arm-motion-integration]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-102, ELECTRI-118, release-deploy-20260918-01, release-deploy-20260918-02]
---

## 背景

V3 已冻结双七轴 CSP、双 PP 夹爪和模块化 physical profile，但默认入口仍是
validation-only。现有公共 FJT 契约和 ELECTRI-102 rolling 实现均使用二代 14 轴语义，
不能直接宣称可用于三代机。

## 改动

新增 `docs/v3-arm-motion-integration.md`，分开描述当前 V3 能力与目标 FJT/rolling 方案；
提出 V3 目标轴序 `right_joint1..7,left_joint1..7`，将 PP 夹爪排除在 14 轴数组之外，
并登记公共契约、1 kHz 参数重标定、模式切换、状态/错误和分级验收要求。

## 验证

静态交叉核对 `v3@1256885` 的 machine manifest、Robot Model、安装 CMake、validation launch，
当前 `robot_interfaces@92d6ff2` 契约，以及未合入 V3 的 ELECTRI-102 rolling feature 和
rolling contract 恢复提交。未运行 V3 轨迹 controller 或真实机械臂。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `v3@1256885` 当前没有 FJT/rolling runtime；公共 R-IN-02 是目标契约，不是已运行 endpoint。
- F2: V3 目标 FJT/rolling 只包含 7+7 CSP 臂轴；两个 PP 夹爪独立，不进入固定 14 轴数组。
- F3: ELECTRI-102 的 session/完整 future 模型可迁移，但二代 axis hash、4 ms timing guard 和 dynamic envelope 不可复用。
- F4: `/joint_states` 与 R-IN-02 从二代 14 轴变为 V3 14 臂轴是行为语义迁移，所有消费者必须同 SHA 原子升级。

## 遗留

在 `robot_interfaces` 正式合入 V3 轴序/rolling/PP 契约；完成 V3 hardware runtime、JTC、
enable_manager、状态适配与故障传播；随后执行 Mock、无运动使能、低速 FJT 和 rolling HIL。
