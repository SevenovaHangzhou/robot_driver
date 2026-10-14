---
id: protocol-20261014-01
area: protocol
title: EtherCAT 离线 renderer 与原位只读预检迁移
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
related: [integration-20261014-01, BQ-154]
---

## 背景

协议 profile、私有校验、渲染和只读预检回到协议 owner。

## 改动

新增 Mapping API 1 renderer、两份私有 schema、两个 drive profile、coupler identity 和族清单。
现行 V2 families/slaves 未修改。运行 PDO 使用旧运行入口实际的两项 RxPDO 模板，
不把三项 RxPDO draft 当作当前运行输出。PP 仍只有 control_word writer。

## 验证

受影响包构建和既有 C++ 测试通过。合成 conformance 验证离线无写入、
不读取归档、不 import composer、输出侧两向换算与注入的预检拒绝。
整体结果与限制见 [integration 记录](../../integration/records/2026-10-14-arm-offline-composition.md)。
未执行真实 EtherCAT CLI 或设备操作。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: command/feedback 因子仅使用 output encoder counts，不乘 reduction_ratio。
- F2: 原位 6041/603F/6040、身份、PREOP 和分支检查沿用；0x730F 由 single_turn profile 数据允许。
- F3: 运动入口没有新增总线预检；未知设备额定参数未冒充真实事实。

## 遗留

真实设备、完整拓扑与 BQ-154 未验证；其它协议族在后续步骤实现。
