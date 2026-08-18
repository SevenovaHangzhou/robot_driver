---
id: motion-20260819-09
area: motion
title: ELECTRI-102 rolling 运行参数 YAML 化并冻结
date: 2026-08-19
type: feat
trigger: ELECTRI-102 / T-16 / E102-D06 / E102-D07 / E102-D36
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rt_control_bringup/config/controllers.yaml, src/rt_control/rolling_trajectory_controller/src/rolling_trajectory_controller.cpp, src/rt_control/rolling_trajectory_controller/test/test_controller_lifecycle.cpp, src/rt_control/rolling_trajectory_controller/test/test_controller_callbacks.cpp]
supersedes: []
related: [ELECTRI-102]
---

## 改动

移除 controller 对 capacity、horizon、timeout、replace lead、state/prime period、controller
period、四项 stopping guard 和 feedback-age gate 的编译期使用；在 configure 时从 ROS 参数
读取并交叉校验。takeover、splice position 和 splice velocity 均使用固定协议轴序的 14 值
数组，值在 `controllers.yaml` 明列。

成功 configure 后，controller 自有参数由回调门拒绝修改；不影响同一 node 上非本控制器
拥有的参数。Open response 从冻结快照回报实际生效的 capacity、horizon、timeout、lead 和
nominal period，不再回报代码常量。

## 验证

- RED：capacity=32、max horizon=700 ms、timeout=317 ms、lead=24 ms 仍返回旧常量，且
  `buffer_capacity` 未声明，冻结测试失败。
- GREEN：同一覆盖值逐项出现在 Open response；configure 后 set parameter 明确失败。
- 18 个非法矩阵覆盖 capacity 两端、timeout 两端、horizon/period/guard 关系、零值、数组长度、
  负数、NaN/Inf，全部 configure fail-closed，无钳制。
- `controllers.yaml` 三个数组均为 14 项；rolling 完整 CTest 12/12 通过。

没有访问现场总线、reset、enable 或运动。当前 YAML 的 provisional source/file 在下一原子
任务接入；本记录不把 test-only controller 测试升级为实机证据。

## 冻结事实

- F1: controller 运行参数来自 configure-time 快照，活动期不可热改。
- F2: 三类 tolerance 都是显式 14 轴数组，轴序等于 rolling 协议轴序。
- F3: Open response 回报冻结后的实际能力值，非法配置失败而非钳制。
