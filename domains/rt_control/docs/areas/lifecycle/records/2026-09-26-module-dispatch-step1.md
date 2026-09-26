---
id: lifecycle-20260926-01
area: lifecycle
title: 按功能模块分发使能/失能/复位（ELECTRI-150 第一步）
date: 2026-09-26
type: feature
trigger: BQ-154；ELECTRI-150；ELECTRI-109
commits: [feature/electri-150-enable-partition]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-154, BQ-151, ELECTRI-150, ELECTRI-109, release-deploy-20260919-01]
---

## 背景

BQ-154 允许在已启动的控制范围内按功能模块分别使能、失能、复位。当前 V3 中
`enable_manager` 只在机械臂运行环境里管理 `arms` 一个模块（14 CSP + 2 PP），头部由
`damiao_head_controller` 在 `/rt/head/*` 独立管理，升降/底盘/悬挂尚无运行环境。用户选定
分两步：本记录为第一步（模块分发），第二步（同一 `enable_manager` 内的按轴子集状态机）
待第二个模块有运行环境后实施。

## 改动

- `rt_control_interfaces/srv/RtEnable.srv`：请求增加 `string[] modules`（空 = 该服务可达的
  全部模块）；响应增加 `module_names`、`module_ok`、`module_stages`。旧字段不变。
- `enable_manager`：新增参数 `owned_modules`、`remote_module_name`、`remote_service_prefix`
  （默认均为空，即旧行为）；`/rt/enable|disable|reset_fault` 先做模块规划：
  - 空请求：运行本管理器全部轴；远端模块的服务存在时一并转调，不存在时跳过；
  - 只点名本管理器的一部分模块：拒绝，`module_partition_unsupported`，不改变任何控制字；
  - 点名未知或未加载模块：拒绝，`module_not_managed`；
  - 点名远端模块：在非实时服务线程转调 `<prefix>/enable|disable|reset_fault`；服务不存在
    `module_unavailable`，超过 `service_result_timeout_ms` 为 `module_service_timeout`；
  - 失能对所有选中模块都执行，即使前一个失败；总结果取第一个失败模块的 stage。
  - 纯规划逻辑独立为 `module_dispatch.hpp`；诊断增加 `modules`、`remote_module` 键。
- V3 机械臂运行环境与原位使能入口配置 `owned_modules: [arms]`、`remote_module_name:
  head_gimbal`、`remote_service_prefix: /rt/head`（名称与 `alfa_v3.yaml` 一致，有测试约束）。
- `/rt/head/*` 自身不解析 `modules`（单模块服务）；`/control/set_enabled` 仍发空请求，保持
  整机语义。

## 验证

T1（本机隔离工作区，无硬件，`ROS_DOMAIN_ID=150/151`、`ROS_LOCALHOST_ONLY=1`）：

- `colcon build`：rt_control_interfaces、enable_manager、damiao_head_controller、
  robot_hw_can、control_api_adapter 及依赖：PASS。
- `enable_manager`：84 tests / 0 failures（原有 65 项全部通过；新增 10 项纯规划单测与 7 项
  控制器测试：空请求旧行为、部分模块拒绝且控制字不动、全部模块、远端缺失、远端转调
  （仅头部/整机/失能部分失败）、远端超时、非法模块配置）；新增远端相关测试重复 5–10 次稳定。
- damiao_head_controller、robot_hw_can、control_api_adapter 测试通过（合计 150 项含上项）。
- `rt_control_bringup` 测试：与 `main@946ec11` 在同一隔离环境下失败集合完全一致（23 项，
  均为该环境缺少 EtherCAT 等依赖所致），本分支多通过 1 项新增测试。
- `tools/quality_gate.sh`、`check_v3_branch_contract.py`、`git diff --check`：PASS。
- 未验证：controller_manager 下的完整 Mock 运行环境（机械臂 + 头部同时加载目前没有启动入口）、
  实机。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `RtEnable` 请求 `modules` 为空时保持旧的整管理器语义；远端模块服务存在时一并转调。
- F2: 同一 `enable_manager` 内只选部分模块一律拒绝（`module_partition_unsupported`），不隐式扩大或缩小范围。
- F3: 头部由 `/rt/enable` 系列统一入口转调 `/rt/head/*`，转调在非实时线程，受 `service_result_timeout_ms` 约束。

## 遗留

- ELECTRI-150 第二步：同一 `enable_manager` 管理两个以上模块时的按轴子集使能/失能/复位与
  故障联动（BQ-151），需真实拓扑。
- 机械臂与头部同时运行的启动入口尚不存在；当前两者互斥启动，转调只在同时运行时生效。
- 操作台（ELECTRI-109）使能管理与故障诊断页面待接入。
