---
id: control-20261107-02
area: control
title: 底盘行走轮 CSV/CSP 运行时切换与轮子轨迹控制器
date: 2026-11-07
type: behavior
trigger: 用户 2026-11-07：只保留已关闭的 #189 中的底盘 CSV/CSP 切换；复用 set_control_mode，CSP 下由 JTC 接管 8 个轮关节，切换与轨迹由 Motion 经 robot_interfaces 调用；限值与导航共用；切换参数用初始值、实机后再校
commits: [feature/arch-base-control-mode]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261107-01, chassis-control-20261022-01]
---

## 背景

底盘只有速度入口 `/rt/base/cmd_vel`：舵轮控制器写转向位置与行走轮速度，行走轮固定 CSV。Motion 需要在底盘静止时切到位置模式，直接给 8 个轮关节发位置轨迹。契约 robot_interfaces#19 新增 R-IN-23 `/rt/base/set_control_mode`、R-IN-24/25 底盘轮子轨迹与 `VELOCITY_READY=4`。

## 改动

- 行为：使能后底盘处于 `VELOCITY_READY`（舵轮控制器，行走轮 CSV），与现在相同。`/rt/base/set_control_mode` 只在已使能、8 个轮关节连续 20 个采样都静止时切换：
  - 切到 `FJT_READY`：先停舵轮控制器、启 `base_wheel_jtc`（以实测位置保持），再把 4 个行走轮的 0x6060 写为 CSP，等驱动器回读 CSP 后才返回成功；
  - 切回 `VELOCITY_READY`：先检查 JTC 输出与实测相差 ≤ 0.5°，把行走轮写回 CSV 并等回读，再停 JTC、启舵轮控制器；
  - 任一步 500 ms 内没有确认：底盘失能，返回 `TIMEOUT`/`drive_mode_not_confirmed`。`ROLLING_READY` 被拒绝。
- `ctrl_enable_manager`：新参数 `trajectory_controller`（有它时 `motion_controller` 运行在 `VELOCITY_READY`）、`mode_service`（服务名，来自 `interface_naming`）、`mode_switch_joints`（模式随控制器集合切换的驱动器）。只有已使能时才写 CSP；其余阶段每周期写回启动模式并清除请求，所以每次使能仍按固定启动模式确认。运行中这些驱动器的模式回读为启动模式或 CSP 都视为正常。
- `hw_ethercat`：Kinco 插件失能时输出启动模式（即使最后请求的是 CSP）；machine 模块的轴新字段 `mode_switching: true` 打开插件的运行时切换。渲染器导出各驱动器的位置范围与需要切换的关节。
- 组合：`chassis_only` 新增策略 `base_motion`（`policies/Alfa_V3_chassis_motion.yaml`）与控制器 `base_wheel_jtc`。JTC 的速度、加速度上限按舵轮导航配置换算（转向 35°/s、35°/s²；行走轮 2 m/s、2 m/s² 除以各模块轮半径，0.1 m 时为 20 rad/s、20 rad/s²），位置上下限取驱动器范围（转向 ±120°，行走 ±49000 rad），由补丁 0007 在准入时检查。新参数源 `joint_values`；`interface` 参数源可由策略选定名字。
- 双臂：模式服务名改由策略给出（`mode_service: arms_control_mode`），值仍为 `/rt/arms/set_control_mode`，行为不变。

## 验证

本机构建：`ctrl_enable_manager`（新增 `test_drive_mode_switch` 8 例：接口声明、配置拒绝、控制器集合映射、只在使能时写 CSP、运行中 CSP 只对切换驱动器有效、经假 controller_manager 的往返切换顺序、CSP 不确认时失能、拒绝 ROLLING_READY）、`hw_ethercat`（新增失能回启动模式）、`integ_composer` 229 项（新增底盘切换组合用例与按模块轮半径换算）、`integ_diagnostics` 全部通过。台架和实车都未验证；mock 硬件不会回读新模式，mock 下切换会超时失能。

## 待实机确认

研华底盘：未使能启动正常；使能后切 FJT_READY、发小幅轮子轨迹、切回 VELOCITY_READY；未停稳时切换被拒；行走轮 CSV→CSP 交接时插件要求位置指令与实测完全一致，需确认 JTC 保持位置能满足。
