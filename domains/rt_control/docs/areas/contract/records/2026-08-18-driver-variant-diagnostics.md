---
id: contract-20260818-01
area: contract
title: 驱动变体诊断投影与 readiness 完整性（ELECTRI-94）
date: 2026-08-18
type: feature
trigger: ELECTRI-94
commits: [feature/driver-variant-config]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: ["status_adapter pytest: 12/12 PASS", "control_api_adapter and rt_diagnostics build PASS", "Docker build: 28 packages PASS", "isolated Mock ROS startup/shutdown: PASS"]
supersedes: []
related: [motion-20260818-01, governance-20260818-01]
---

## 背景

readiness 过去只比较诊断数量，正确数量的重复/未知 ID 可能被误判健康；CANopen
诊断还用 `node + 2` 和数组下标隐式假设节点连续。

## 改动

status adapter 从 manifest 受控 ID 常量派生名称并保留整批实际 ID 序列；重复、未知、
缺失或换序时返回 NOT_READY 与明确 fault。拓扑和组件缓存作为不可变快照原子替换，
节点回调串行化，避免发布线程混用不同批次。rt_diagnostics 使用显式 ring/joint/node
表、查找 node index，并从完整 ring 派生 expected responder 数量。

## 验证

status adapter 12 个 pytest 全过，包含重复/未知 EtherCAT 与 CANopen ID 的 fail-closed、
合法批次恢复、无关 diagnostics 保留和不可变批次更新；`control_api_adapter` 与
`rt_diagnostics` 原生构建通过，完整生产 Dockerfile 的
28 包构建通过。无网络/设备 Mock 容器内 `rt_diagnostics` 与 adapters 启动并 clean exit；
这只证明 ROS graph 加载/收尾，不证明真实 EtherCAT/CANopen diagnostics。未访问总线。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: readiness 必须匹配 manifest 派生的完整 EtherCAT/CANopen diagnostic ID 集合；重复、缺失、未知或换序均不 ready 并报告 incomplete set。
- F2: CANopen 诊断归一化按显式 node-ID 表查找，禁止依赖连续节点编号算术。

## 遗留

尚未在 Mock/生产 graph 中观测完整 diagnostics 流；严格顺序属于当前生产发布约束，
若未来上游允许无序诊断集合，需先裁决并同步测试语义。
