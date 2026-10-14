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

用户批准唯一冻结V2 Xacro排除本次V3频率验收，保留原control_frequency 1000与原文件。
V3迁移baseline_violations已清空；公开scope exclusion带路径、SHA-256和用户批准理由。
若内容改变，冻结哈希与R6检查失败；只有原字节可以排除周期检查，其它规则不排除。
工具门禁违规为0。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 观察指标起点尚待合并，不能使用候选分支SHA代替。
- F2: 用户已批准冻结V2文件排除V3频率验收；V3基线为空，原文件哈希必须保持不变。

## 遗留

合并后登记main SHA，再统计20个合并PR。冻结文件的内容若需修改，必须经独立任务评审；
当前排除不允许该文件的任何字节改动。
