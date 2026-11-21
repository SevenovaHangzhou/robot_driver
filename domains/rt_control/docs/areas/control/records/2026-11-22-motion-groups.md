---
id: control-20261122-01
area: control
title: enable_manager 支持多个运动组，机械臂组与底盘组各自切换模式
date: 2026-11-22
type: feature
trigger: 用户 2026-11-22 要求研华整机一个制品同时跑导航底盘与运控双臂/升降，并确认多运动组语义（ELECTRI-236）
commits: [feature/motion-groups]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [control-20261107-02]
---

## 背景

`ctrl_enable_manager` 只有一套运动组参数（一个主控制器、一个备用控制器、一个切换服务、最多 16 轴），一个制品里只能有一组带模式切换。整机需要机械臂组（轨迹 ↔ Rolling）与底盘组（速度 ↔ 轮子轨迹，行走驱动 CSV/CSP）同时存在。限制来自本包设计，与 EtherCAT、驱动器无关。

## 改动

- 参数：新增 `motion_groups`（组名列表），每组参数为 `<组>.motion_controller`、`rolling_controller`、`trajectory_controller`、`mode_service`、`motion_joints`、`mode_switch_joints`、`mode_switch_*`。原顶层同名参数删除，只保留分组一种写法。组数上限 4，每组最多 16 轴。
- 每组有自己的切换服务（`mode_service`）和静止确认服务 `~/<组>/confirm_standstill`（原 `~/confirm_standstill`）。
- 使能一次激活各组的默认控制器和伴随控制器；失能停掉全部。模式切换只改本组自己的控制器，不动伴随控制器和其他组。
- 切换只检查本组关节是否静止；同一时间只允许一个组切换（共用所有权），另一组请求返回 `busy`。
- 任一组切换失败或故障时整机失能；急停、失能、停用会中止所有组的切换。
- 状态接口：各组位置依次排在 CiA402 轴之后，共用一个 `ethercat_domain/process_data_age_ms`；命令接口：各组需要切换的驱动依次排在握手设备之后。
- 取消"固定启动模式与 Rolling 不能同时配置"的限制：Rolling 不改驱动模式，整机里双臂、升降、转向为 CSP，行走为 CSV。
- 不采用 ros2_control 的 `perform_command_mode_switch` 切驱动模式（Humble 中它不等驱动确认、失败只打日志），保留"写模式并等回读确认"。
- 组合器：控制器条目新增 `motion_group`（arms/base/suspension），`enable_manager` 条目新增按组的参数模板；组内有可激活控制器时才生成该组。`lifecycle.yaml` 的 `mode_controllers` 改为 `mode_controller_groups`（每组一列），`integ_diagnostics` 按组检查恰好一个激活。
- 头部 `head_only` 固定输出改为 `motion_groups: [head]`。
- 运行日志 trace 的 `control_mode` 字段记录第一组（机械臂组）的模式。

## 验证

- `ctrl_enable_manager`：原有 121 项与新增 8 项（两组接口布局与服务名、重叠/歧义配置拒绝、使能激活各组默认控制器与伴随、一组切换另一组不动且另一组运动不影响、切换中另一组 busy、一组切换失败整机失能、停用中止所有组、只允许被切换驱动处于 CSP）全部通过，连续 10 轮无失败。
- 组合器、诊断、bringup Python 测试：除依赖安装路径的 1 项（main 上同样失败）外全部通过；新增组参数生成与每组模式控制器集合测试。旧 scope 的金样只在 enable_manager 参数与 lifecycle 键上更新（manifest `motion_groups_regeneration`）。
- 未验证：mock 启动（T1）、研华实机（整机 scope 在 ELECTRI-239）。

## 结论与冻结事实

- F1: 一个 enable_manager 可有多个运动组，各自切换模式；整机使能状态、失能、急停与故障仍为一份并作用于所有组（用户 2026-11-22）。
- F2: 静止确认服务按组命名为 `~/<组>/confirm_standstill`。

## 遗留

- 操作网页调用的 `/<manager>/confirm_standstill` 需改为按组的路径（ELECTRI-237）。
- 软件急停响应仍只在机械臂策略打开；整机第一版不打开（用户 2026-11-22）。
