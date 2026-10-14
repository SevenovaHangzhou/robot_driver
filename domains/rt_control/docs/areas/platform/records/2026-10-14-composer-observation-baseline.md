---
id: platform-20261014-02
area: platform
title: 四层迁移观察指标与冻结V2周期例外
date: 2026-10-14
type: investigation
trigger: 用户批准第五步默认迁移与工程观察指标登记
commits: [feature/kkozia-19-composer-defaults]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [platform-20261014-01, integration-20261014-04]
---

## 背景

架构迁移的观察指标必须从实际合并后的main开始，不能用候选分支SHA冒充。

## 改动

观察基线：pending_merge。合并后填实际main完整SHA。
窗口为后续20个合并PR；指标为其中触碰src/rt_control/rt_control_bringup的比例，目标≤30%。
本指标用于观察，不作此次验收，不由agent承诺完成日期。

## 验证

仓库扫描只剩一条迁移baseline：冻结V2 Xacro中的control_frequency 1000。
保留原值与精确baseline；该文件未修改。其余旧V3字面量/跨层依赖baseline已移除。
工具门禁的未豁免违规为0；不等于baseline为空。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 观察指标起点尚待合并，不能使用候选分支SHA代替。
- F2: V2保护优先，单条周期baseline公开保留；A3/清空baseline验收不能宣称完成。

## 遗留

合并后登记main SHA，再统计20个合并PR。若要求清空唯一V2例外，必须先裁定冻结资产的验收范围；
不擅自改V2或把旧例外换名隐藏。
