---
id: arm-control-20261014-01
area: arm-control
title: 显式使能与手臂容差策略文件
date: 2026-10-14
type: feature
trigger: 用户确认的四层解耦第二步
commits: [feature/kkozia-7-arm-composition]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [integration-20261014-01, protocol-20261014-01]
---

## 背景

使能批次归 enable_manager；运动容差和临时包络归机械臂控制 owner。

## 改动

安装 enable policy、arm tolerances、mock envelope 及 schema。
16 个单轴批次按环位，14 轴容差按原 right-first MOTION_JOINTS 顺序。
原位与正常入口的资源名和参数集合由显式 runtime view 表达。
批次展开仅为原 ROS 参数格式，不重新分组或推导数值。

## 验证

pytest 比较策略与旧 _controller_config 数值；六份旧输出规范化后零差异。
enable_manager、rolling_trajectory_controller 既有测试通过；未改 C++ 控制行为。
细节见 [integration 记录](../../integration/records/2026-10-14-arm-offline-composition.md)。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 时序、容差、guard、批次均取现有源码值；不使用 composer 常量生成策略。
- F2: 当前两个旧入口 service_result_timeout_ms 均为 90000。
- F3: envelope 仍为 ESTIMATED_NOT_MEASURED；它不构成实机运动认证。

## 遗留

原位完整 T1、目标机/实机包络与安全批准未完成。
