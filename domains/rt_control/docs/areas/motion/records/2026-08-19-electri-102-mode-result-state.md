---
id: motion-20260819-11
area: motion
title: ELECTRI-102 模式结果进入 rolling 公共状态
date: 2026-08-19
type: feature
trigger: ELECTRI-102 / R-OUT-07 / E102-D39
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/interfaces/rt_control_interfaces/msg/JointControlModeResult.msg, src/rt_control/enable_manager/test/test_enable_manager_state_machine.cpp, src/rt_control/rolling_trajectory_controller/test/test_state_publisher.cpp]
supersedes: []
related: [ELECTRI-102, lifecycle-20260819-04]
---

## 背景

公共 `SetJointControlMode` service 的即时响应由 enable_manager 产生，而公共
`RollingJointControlState` 必须由 rolling controller 持续发布最近一次模式结果、request ID、
source/target controller 证据和 restart-required。若让 enable_manager 也发布 rolling state，
会形成两个互相不完整的公共事实源；若保留原来的固定零值，重连的 Motion 又无法恢复观察。

## 改动

- 在域私有 `rt_control_interfaces` 新增 `JointControlModeResult`，只承载两个 RT controller
  之间的结果交接；其枚举字段复用公共 `robot_rt_control_interfaces`，避免复制数值语义。
- enable_manager 在 `/rt/internal/joint_control/mode_result` 发布非零、单调 sequence 的结果，
  字段逐项来自实际 service response。精确幂等重放不重复发布；同 ID 不同 payload 的拒绝会
  产生新的可观测结果。
- rolling 使用已命名的 reliable/volatile state QoS 接收事件，严格校验协议 1.0、非零
  request/sequence、枚举范围和单调顺序，再写入 public state 的 `last_mode_*` 与三项证据。
- rolling 仍是 `/rt/rolling_joint_control/state` 的唯一 publisher。deactivate 发布 TERMINATED
  前清除旧事件，避免把上一次 activation 证据误报成当前 rolling→FJT 切换；deactivate 后收到
  的新结果可留给下一次 activation 的初始状态。
- 更新私有接口所有权测试；外部域不得依赖私有 msg 或内部 topic。

## 验证

- 独立 Humble overlay 构建 `rt_control_interfaces`、`enable_manager` 与
  `rolling_trajectory_controller`：PASS。
- enable-manager CTest：1/1 PASS；真实 ROS publisher/subscriber 验证 service 结果按序发布，
  精确幂等 replay 不重复事件。
- rolling CTest：13/13 PASS；状态测试证明最新合法事件逐字段进入公共 state，旧 sequence 与
  错协议事件不覆盖当前结果，既有 deactivate terminated 证据不回归。
- 私有／公共接口所有权测试：15/15 PASS；`git diff --check`：PASS。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `/rt/rolling_joint_control/state` 只有 rolling controller 一个公共事实源，enable_manager 不得成为第二 publisher。
- F2: controller 间模式结果只经 RT-Control 私有、reliable/volatile 的有序事件交接，外部域只消费公共 service/state。
- F3: rolling 仅接受协议、标识、枚举和 sequence 全部合法的新事件；deactivate 的终止证据不复用旧 activation 事件。

## 遗留

- DDS 发现与延迟分布将在 T-19 桌面性能报告中记录；目标机 QoS／线程调度仍需另行验证。
- 私有事件不是新跨域契约，不进入 robot_interfaces PR；public R-OUT-07 字段本身仍来自待提交的公共接口分支。
