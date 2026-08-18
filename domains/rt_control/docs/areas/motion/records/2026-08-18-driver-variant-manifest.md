---
id: motion-20260818-01
area: motion
title: 驱动变体 manifest 与控制器投影（ELECTRI-94）
date: 2026-08-18
type: feature
trigger: ELECTRI-94
commits: [feature/driver-variant-config]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: ["driver variant tests: 65 passed", "quality gate: 260 passed", "Docker build: 28 packages PASS; tag rt-control:electri-94-final", "isolated no-device Mock startup/shutdown: PASS"]
supersedes: []
related: [lifecycle-20260818-01, contract-20260818-01, canopen-chassis-20260818-01, governance-20260818-01]
---

## 背景

ELECTRI-94 要求电机数量、型号与有效减速换算不再依赖跨包手工同步的常量副本。

## 改动

新增安装随包发布的 `driver_variant.yaml`，描述完整 0..15 EtherCAT ring、14 个
whole-body joint、2 个 CANopen track joint、drive profile、SI 有效换算、控制器角色、
JTC 起点容差、使能批次与生命周期策略。`controllers.yaml` 的 Ti5 失能例外改为显式
joint 列表。Robot Model 继续拥有 logical joint 名称与类型；离线投影同时校验其 active
joint 集合及 revolute/prismatic 对应的 SI 单位。phase 1 保留现有 Xacro/YAML/C++ 为
运行时输入，由离线投影门禁校验一致。

## 验证

`python3 -m pytest tools/tests/test_driver_variant_*.py -q`、统一投影 CLI 与
`tools/quality_gate.sh` 通过；manifest 已由 `rt_control_bringup` 的 config 目录安装规则
覆盖。完整生产 Dockerfile 构建 28 个包通过；无网络、无设备、无额外 capability 的
一次性容器检查确认入口、启动器、三个相关包及安装空间 manifest 均存在，且 manifest
保持 schema 1、16 个 ring member、16 个 logical joint 与 CAN Node 2/3。同一隔离条件
下的 Mock 启动确认 controller manager 250 Hz、预期 controller 状态；标准 TERM 先完成
`/rt/disable`、controller quiesce 与 mock hardware inactive，再以 0 退出。未执行手册中
需要获批测试网络和生产资源参数的完整 L1 Mock，也未做实机运行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `driver_variant.yaml` 是 actuator 到既有 logical joint 的驱动映射、总线地址、profile、有效 SI 换算、控制器角色、容差、使能批次与生命周期策略的版本化事实源；logical joint 名称与类型仍以 Robot Model 为权威。
- F2: 当前机型为 14 个 whole-body EtherCAT joint 加 2 个 CANopen track joint；EtherCAT ring 包含位置 0/13 两个 passive hub。
- F3: ELECTRI-94 phase 1 是构建期/评审期一致性门禁，不提供运行时热切换或自动写硬件配置。

## 遗留

offset、software zero、PDO/SDO、vendor identity 与现场标定仍由现有 profile/权威证据
管理；hub 0/13 的观测身份暂未提升为运行匹配约束。任何新机型仍须重新构建 Docker、
执行获批的 Mock 及按风险分级的实机 commissioning；静态一致与本次容器产物检查不能
证明物理方向、比例或停机反应。
