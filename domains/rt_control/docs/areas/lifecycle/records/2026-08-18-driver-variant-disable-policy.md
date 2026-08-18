---
id: lifecycle-20260818-01
area: lifecycle
title: 驱动变体按轴失能终态策略（ELECTRI-94）
date: 2026-08-18
type: feature
trigger: ELECTRI-94
commits: [feature/driver-variant-config]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: ["enable_manager build PASS", "enable_manager GTest: 42/42 PASS", "Docker build: 28 packages PASS", "isolated Mock orderly shutdown: PASS"]
supersedes: []
related: [motion-20260818-01, governance-20260818-01, BQ-115, BQ-126]
---

## 背景

Ti5 四轴的 `ReadyToSwitchOn` 失能终态原按 C++ 轴号硬编码，改变轴数或型号会静默把
例外策略套到错误电机，掉电恢复脚本还维护另一份名单。

## 改动

`enable_manager` 从 `controllers.yaml` 读取 joint 名列表，在 configure 时一次性映射为
固定大小 bool mask；未知或重复 joint 返回 ERROR，250 Hz update 只做数组读取。
恢复检查脚本使用同名语义常量，并由 manifest 源码投影门禁约束。

## 验证

原生构建与完整生产 Dockerfile 的 28 包构建通过；42 个 GTest 覆盖全部轴的标准终态、
仅配置轴的例外终态、未知/重复配置拒绝、configure 后参数不改变冻结 mask，以及混合
终态的成功/不提前成功路径。无网络/设备 Mock 容器确认 `enable_manager` active；标准
TERM 触发内部 `/rt/disable`，返回 already-disabled 成功后 quiesce controller、deactivate
mock hardware 并 clean exit 0。未访问真实总线、未调用 reset/enable、未驱动实机。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `ReadyToSwitchOn` 失能终态例外必须按 joint 名配置，并在 configure 时冻结为预分配 mask。
- F2: 所有轴接受 `SwitchOnDisabled`；只有 manifest 指定轴接受 `ReadyToSwitchOn`，未知/重复配置 fail-closed。

## 遗留

未在真实 Ti5/XMC 上重复失能与掉电恢复；BQ-115/BQ-126 的既有实机边界不因本次
静态重构自动扩大，新 motor profile 必须重新提供终态证据。
