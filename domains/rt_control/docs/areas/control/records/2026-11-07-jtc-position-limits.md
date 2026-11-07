---
id: control-20261107-01
area: control
title: JTC 准入增加可选位置上下限
date: 2026-11-07
type: behavior
trigger: 用户 2026-11-07：底盘轮子轨迹（R-IN-24/R-IN-25）的转向位置不能超过导航生效的 ±120°，由 JTC 准入整条拒绝，而不是在驱动插件里触发锁存故障
commits: [feature/arch-jtc-position-limits]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261104-03]
---

## 背景

补丁 0006 的准入只检查速度与加速度，不检查位置。Kinco 驱动插件收到超出 `min_position/max_position` 的位置指令时锁存错误并退出 Operation Enabled，enable_manager 随之急停。底盘转向位置轨迹需要在进入实时路径之前被拒绝。

## 改动

新增 `patches/ros2_controllers/0007-position-limit-admission.patch`，接入 bootstrap 和 Dockerfile：

- 新参数 `joint_position_limits_lower` / `joint_position_limits_upper`（rad，每个关节一对，只读）。两者为空时不检查位置，现有双臂配置不变。
- 配置了上下限时必须同时启用 `trajectory_start_consistency_check`，并且每个关节一对有限值且 lower < upper，否则配置失败。
- 准入时每个点、每段三次 Hermite 曲线（包括从上一条指令接续的引入段）的位置范围都必须在上下限内；曲线范围取两端与速度零点处的值。超限整条拒绝，日志 `FJT_REJECT reason=position_limit`。action 与话题入口共用。

## 验证

本机打 0001–0007 补丁构建 JTC：`colcon test` 643 项 0 失败；新增 2 例（上下限成对校验；点超上限、点超下限、两点都在限内但曲线越限 0.308 rad 被拒，限内轨迹接受）。未上台架；本补丁暂无配置使用者，底盘轮子轨迹控制器在后续 PR 中配置。
