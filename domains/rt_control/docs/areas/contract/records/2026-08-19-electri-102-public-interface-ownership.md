---
id: contract-20260819-01
area: contract
title: ELECTRI-102 rolling 公共接口所有权裁决
date: 2026-08-19
type: decision
trigger: ELECTRI-102 / 用户授权 Codex 完成剩余裁决（2026-08-19）
commits: [功能/视觉伺服-ELECTRI-102]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: [domains/rt_control/docs/electri-102-clarification-decisions.md, robot_interfaces-local-9cc9379]
supersedes: []
related: [contract-20260814-01, contract-20260817-01, ELECTRI-102]
---

## 背景

ELECTRI-102 的 rolling 五端点跨越 Motion 与 RT-Control。接口落地时，现行
`robot_interfaces` 门禁暴露了早期“全部放 RT 包”的表述与 provider-owned IDL 规则冲突。

## 改动

主决策文档明确按 endpoint 提供方拆包：Motion update batch/point 属 Motion 包；RT-Control
三个 service、state 及成员类型属 RT 包；QoS 属共享包。update 登记为 M-09，R-IN-08 留空。

## 验证

外部接口功能分支 `9cc9379` 的契约闭合、错误码、changelog、生成视图及 54 项工具测试通过；
ROS 2 Humble 下六个接口包全量构建通过。driver 尚未消费该提交，故本记录仍为 T0/UNVERIFIED。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: Rolling 五端点是 Motion/RT-Control 跨域公共接口，禁止复制进 driver 私有接口包。
- F2: M-09 update 的 batch/point 归 Motion 包；其余四端点类型归 RT 包；QoS 归共享包。
- F3: 接口 PR 必须等待 ELECTRI-102 完整 mock/fake 软件门，不因 IDL 单独编译通过而提前提出。

## 遗留

driver 尚未完成控制器、模式切换、mock 长稳和 public-IDL-only producer；接口仓库没有 PR、
release tag 或最终 main SHA，driver 的 vendored pin 保持不变。
