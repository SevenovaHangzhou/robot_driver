---
id: integration-20261104-01
area: integration
title: RT-Control 公开接口切换到 robot_interfaces cb379db 的 /rt/ 名称
date: 2026-11-04
type: behavior
trigger: 用户 2026-11-04 给定目标接口表；robot_interfaces#13 合并为 cb379db
commits: [feature/arch-rt-endpoint-names]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 背景

公共契约 robot_interfaces#13 把 RT-Control 跨域接口统一到 `/rt/`，删除 `/control/set_enabled` 与未实现接口，公开 `/rt/enable`、`/rt/disable`、`/rt/reset_fault`（新公共 `RtEnable`，带 DREE 错误）。本仓库一次切换，不保留旧名。

## 改动

- pin 升到 `cb379db`。
- 使能管理器直接提供三个公共服务，响应 `error` 按阶段给 DREE 码（`operation_in_progress`、`controller_inactive`、`device_offline`、`restart_required` 各有专码，其余按操作给 `RT_ENABLE_FAILED`/`RT_DISABLE_FAILED`/`RT_RESET_FAILED`），并记一行 `<服务> accepted|rejected: …`。`integ_api_adapter`（只为 `/control/set_enabled` 存在）与私有 `RtEnable` 删除。`/rt/enable` 不再自动复位，`fault_requires_reset` 时先显式 `/rt/reset_fault`。
- 轨迹控制器 action 名由上游 JTC 窄补丁 `patches/ros2_controllers/0005-public-action-name.patch` 的 `action_name` 参数给出（Humble 不支持重映射 action 名，本地已验证）；名字只在 `integ_composer/config/interface_naming.yaml`，控制器绑定用新参数源 `interface` 引用。双臂作用域 `/rt/arms/follow_joint_trajectory`，单臂 `/rt/arms/{left,right}/follow_joint_trajectory`。
- 其余改名：rolling 四个、模式切换、`/rt/base/cmd_vel`、`/rt/base/odom`（原 `~/odom` 未接到 `/wheel/odom`）、`/rt/readiness`、`/rt/run_self_check`、`/rt/sensors/state`、`/rt/infrared/state`、`/rt/imu/data`、`/rt/led/set_rgbw`、`/rt/ultrasonic/channelN/range`、`/rt/force_torque/<侧>_wrist/wrench`、`/rt/battery/{left,right}/state`（BMS 地址 1=左、2=右，用户 2026-11-04）。

## 验证

本机全量构建与 1155 项测试通过（含补丁后的 JTC 构建；action 名重映射在 Humble 不生效的本地实验）。未上台架。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: RT-Control 公开接口名以 robot_interfaces `cb379db` 为准，本仓库不保留旧名。
- F2: 公开使能入口为 `/rt/enable`、`/rt/disable`、`/rt/reset_fault`，由使能管理器直接提供。

## 遗留

- `/rt/head/move` Action 未实现；网页仍写内部 `/head_position_controller/commands`（契约已列为删除名）。
- 真空泵/阀与压力在本仓库无实现。
- 台架：部署前须给台架 vendor 的 ros2_controllers 打 0005 补丁并重建 JTC。
