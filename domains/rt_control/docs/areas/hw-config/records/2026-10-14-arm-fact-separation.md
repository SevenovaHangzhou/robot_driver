---
id: hw-config-20261014-01
area: hw-config
title: 手臂设备、机型与单机事实分离
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

硬件配置只保存有来源的事实与明确缺项，不提供代码 API。

## 改动

迁移 device、machine、unit、只读 evidence 文件。旧文件保留。
逐字段来源和 TBD 清单见 [来源明细](../../integration/arm-composition-sources.md)。
不猜 eRob 型号到关节的绑定；未分配型号的文件仅承载已使用的输出计数口径。
台架腕传感 2/4 保留并标明适用范围，全机拓扑不据此确认。

## 验证

当前新数据文件通过九份共享 schema 的校验；零位和 evidence 原始内容对齐。
safety_policy 的 fault_dependencies 原样迁移，人工 PR 批准仍未发生。
此处验证不等于实机确认。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 单机 16 轴零位捕获来自原文件，方向来自原机型暂定标定，型号映射仍未知。
- F2: 正常主环 1000 Hz；原位 scope 保留 250 Hz 与 BQ-154。
- F3: 其余模块仅有迁移占位信封，没有 runtime-ready 资格。

## 遗留

型号/额定参数、实际运动方向与单机确认、完整传感拓扑及用户安全策略批准待完成。
