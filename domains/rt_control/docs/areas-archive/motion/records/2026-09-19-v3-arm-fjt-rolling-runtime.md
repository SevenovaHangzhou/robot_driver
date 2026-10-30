---
id: motion-20260919-01
area: motion
title: V3 双七轴 FJT 与 rolling 实时控制
date: 2026-09-19
type: feature
trigger: ELECTRI-102；用户要求 V3 双臂可由 Motion 控制且接入实时 rolling
commits:
  - db2960b725e45b49e8c4694a3e8d95ef82eecd6b
  - 7d2600ac7a4e91e6e30413e0bdcd8b30739a3ee1
  - 6cc216ab73b74a598618c090d24e17cd90545918
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [contract-20260919-01, lifecycle-20260919-01, release-deploy-20260919-01]
---

## 背景

远端 V3 只有 validation/enable-only，不能接受 Motion 的 7+7 CSP 轨迹。

## 改动

迁入 `rolling_trajectory_controller`，新增 14 轴 `whole_body_jtc`，固定顺序为
`right_joint1..7,left_joint1..7`；PP 夹爪不进入任一运动 writer。控制周期改为 1 kHz，
provisional 包络仅作为未实测低速配置。

## 验证

rolling 126 tests、V3 runtime/hardware contract 66 tests 通过；Mock 完整 14 轴 FJT 成功，
STRICT JTC 到 rolling 切换成功且 writer 唯一，rolling state 可观测。本记录不授权使能或运动。

## 结论与冻结事实

- F1: V3 FJT/rolling 运动数组固定为右臂 7 轴在前、左臂 7 轴在后，单位 rad/rad/s。
- F2: 两个 PP 夹爪只做使能与保持，不进入 14 轴 FJT/rolling。
- F3: V3 控制器初始周期合同为 nominal 1 ms、maximum 2 ms，仍待目标机测量冻结。

## 遗留

逐轴方向、动态包络和停车参数待实机确认；未执行真实 EtherCAT 运动。
