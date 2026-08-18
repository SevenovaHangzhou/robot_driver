---
id: canopen-chassis-20260818-01
area: canopen-chassis
title: 履带节点与模式安全投影（ELECTRI-94）
date: 2026-08-18
type: feature
trigger: ELECTRI-94
commits: [feature/driver-variant-config]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: ["four ros2_canopen patches apply to pinned source", "source projection mutation tests PASS", "Docker build: 28 packages PASS", "isolated Mock startup/shutdown: PASS"]
supersedes: []
related: [motion-20260818-01, governance-20260818-01]
---

## 背景

Node 2/3 与 PV mode 3 曾散落在 EMCY、activate、completeness、rollback 和 deactivate
分支；只改 bus/Xacro 会让部分安全路径仍使用旧节点。

## 改动

冻结上游补丁新增唯一 `TrackNodeConfiguration` constexpr 表，EMCY filter、activation
允许集与完整性、rollback/deactivate 均消费该表；补丁显式包含 `<array>`。manifest
投影器精确校验 node ID 与 operation mode，并禁止旧的 2/3 分支重新出现。

## 验证

四个 ros2_canopen 补丁按顺序对冻结 SHA 执行 `git apply --check`/应用通过；node、mode、
重复 ID、marker 与遗留消费者 mutation 测试通过。宿主缺少 `lely_core_libraries`，但
完整生产 Dockerfile 已构建全部 28 个包，无网络/设备容器检查确认 `robot_hw_canopen`
已安装；GenericSystem Mock 完成加载与有序停机。未创建 CAN 流量，未启动真实驱动器。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前履带安全表恰为 Node 2/3、PV mode 3，ros2_canopen 全部生命周期与 EMCY 安全路径共同消费同一表。
- F2: 更换 CANopen 驱动型号不是替换 DCF：必须同时验证 EDS/profile、PDO/OD、单位换算、mode、heartbeat/EMCY 与驱动侧停机反应。

## 遗留

LD2 在断链/NMT Stop 下清除已锁存目标的机械停车证据仍属既有 commissioning 缺口；
新的 profile 未获得该证据前不得进入 production variant。
