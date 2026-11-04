---
id: contract-20260919-01
area: contract
title: V3 rolling 接口 pin 与轴集合身份
date: 2026-09-19
type: feature
trigger: ELECTRI-102；V3 Motion/RT-Control 需要可抓取的 rolling wire schema
commits: [db2960b725e45b49e8c4694a3e8d95ef82eecd6b]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [contract-20260918-01, motion-20260919-01]
---

## 背景

V3 原 pin 不含 rolling schema，二代 axis hash 不能表达双七轴机械臂。

## 改动

pin 升级为可抓取的 `robot_interfaces@9aa2693d7d3235958369272b7ce8c48592dd7e83`；
新增域内 `JointControlModeResult`，V3 axis hash 按每轴 `<name>:rad:rad/s\n` 计算。

## 验证

公共接口四包构建通过，私有接口、rolling 和消费者编译通过；质量门禁 291 passed、
13 skipped、83%。本记录不授权使能或运动。

## 结论与冻结事实

- F1: V3 axis hash 为 `f4c8ff8a32183d1733032494600c9f5d1e7e625673a4ad51556b3ac6c35adee4`。
- F2: `JointControlModeResult` 仅为 RT-Control 域内交接，不是新的跨域 endpoint。
- F3: Motion 与 RT-Control 必须原子使用同一 public schema SHA 和 V3 axis hash。

## 遗留

上游消息注释仍描述二代轴序；正式跨域发布前必须在 `robot_interfaces` 提交 V3 语义修订。
