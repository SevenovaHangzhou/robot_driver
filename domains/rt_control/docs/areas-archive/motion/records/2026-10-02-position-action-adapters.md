---
id: motion-20261002-01
area: motion
title: PP 头部升降位置 Action 适配
date: 2026-10-02
type: feature
trigger: ELECTRI-152，RTC-13～17 软件实现
commits:
  - feature/electri-152-rt-interface-implementation
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [RTC-11, RTC-12, RTC-13, RTC-14, RTC-15, RTC-16, RTC-17]
---

## 背景

> 停止/保持执行器后端和完整取消确认尚未实现，当前 Action 为禁用执行的原型。
> 后续完成边界以 [Draft 复核](../../contract/records/2026-10-03-interface-dependent-draft.md) 为准。

V3 需要将左右 PP、头部和升降从底层保持／工程 Topic 提升为具备到位、取消、停止确认和
故障语义的公共位置 Action，同时保留 14 轴 FJT／Rolling 协议。

## 改动

- 新增 `position_action_adapter`，为 `/pp/{left,right}/move`、`/updown/move` 和 `/head/move`
  提供位置 Action；同资源忙时拒绝，不排队／抢占，取消下发当前实测位置作为保持目标并等待
  停止确认。反馈失效、超时和停止未知分别返回明确失败。
- 目标必须有限并位于配置限位；没有零位、行程、容差、停止能力时 `configured=false`，Goal
  fail closed。左右 PP draft 改为 position-only ForwardCommandController，与公共适配器单 writer。
- 机械臂与头部 launch 增加 `publish_robot_state` 开关，独立调试默认 true，组合运行可以关闭
  重复 RSP。`/joint_states` 仍按各已配置模块发布真实子集，不填未配置 PP／升降零值。
- `/cmd_vel` 保持现有底盘唯一速度入口；未实现底盘与双臂／升降／PP 的跨组互斥。FJT 与
  Rolling 同一 14 轴 writer 的严格互斥不变。

## 验证

- 位置状态机 4 项核心测试及接口适配器聚焦测试 PASS；launch 默认 validation／Mock 检查 PASS。
- `control_api_adapter` 隔离构建与 40 项包测试 PASS。
- `rt_control_bringup` 隔离构建 PASS；完整闭包因本机 EtherLab／ros2_control_test_assets 缺失阻塞。
- 未加载 PP／升降／头部真实 controller，未使能或运动。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: PP、头部、升降公共接口采用绝对位置 Action，同资源忙时拒绝，取消后停止并保持。
- F2: 未确认实物限位、零位、容差和停止保持能力时，位置 Action 不准入真实目标。
- F3: 组件可并行执行；不同组件活动不单独使另一组件 readiness=false 或触发自动停止。
- F4: 整机组合只允许一套 RSP，本体 TF 和每个关节状态各有唯一权威来源。

## 遗留

左右 PP、升降、头部的真实目标范围、位置反馈、停止距离、保持／制动及 controller 启动组合
待配置和实机验证；FJT／Rolling 的既有目标机／实机缺口不由本次软件适配关闭。
