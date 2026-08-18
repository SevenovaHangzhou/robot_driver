---
id: motion-20260819-01
area: motion
title: JTC 首点一致性启用时关闭 topic 命令旁路
date: 2026-08-19
type: fix
trigger: ELECTRI-102 / D-15 A
commits: [功能/视觉伺服-ELECTRI-102]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [patches/ros2_controllers/0001-jtc-start-consistency.patch, tools/tests/test_electri_102_jtc_action_only.py]
supersedes: []
related: [ELECTRI-102]
---

## 背景

现有首点位置与反馈龄准入只位于 FJT action 的 goal 回调；JTC 原生
`~/joint_trajectory` 订阅没有 goal-response 阶段，因此可以绕过两道准入。视觉伺服新增
rolling writer 后，保留该第三条无准入命令路径还会使“当前唯一 writer”无法证明。

## 改动

在冻结的 ros2_controllers 补丁中，把 topic 的存在条件绑定到
`trajectory_start_consistency_check.enabled`：启用一致性准入时不创建订阅并输出结构化
`JTC_TOPIC_COMMAND_DISABLED` 日志；未启用时保持上游 JTC 的 topic 行为。没有增加第二个
容易错配的配置开关。

补丁同时加入两个上游级测试，直接断言启用时 subscriber 为空、关闭时 subscriber 存在；
driver 侧策略门禁断言补丁形态和生产 `controllers.yaml` 始终启用一致性准入。

## 验证

- 在 ros2_controllers 冻结基线 `cbcf66218ff43353f9fb5fe7a2c33f458d578d73` 执行
  `git apply --check`：通过。
- 隔离 ROS 2 Humble 构建 `joint_trajectory_controller` 及其冻结 ros2_control 依赖：
  7 个包构建通过。
- 新增两个 subscriber 行为测试：2/2 通过，并观测到预期结构化日志。
- driver 侧 `pytest -q tools/tests/test_electri_102_jtc_action_only.py`：2/2 通过。
- 上游 JTC 完整 5 组 CTest 继续作为回归门，结果记入本次原子提交的最终自检。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当首点一致性准入启用时，whole_body_jtc 只接受 FJT action，不暴露 trajectory topic 命令入口。
- F2: 当首点一致性准入关闭时，通用上游 JTC 的 topic 行为保持不变。
- F3: topic 是否存在与同一个准入开关绑定，禁止引入可独立错配的第二配置位。

## 遗留

真实 bringup 图上的 topic 缺席仍需在 ELECTRI-102 综合 mock 场景中复验；本记录没有运行
driver 完整 bringup，也没有访问总线或真实硬件。
