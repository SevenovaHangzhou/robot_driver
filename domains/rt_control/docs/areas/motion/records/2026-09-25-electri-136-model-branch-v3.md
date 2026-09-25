---
id: motion-20260925-04
area: motion
title: ELECTRI-136 模型分支重命名为V3
date: 2026-09-25
type: decision
trigger: 用户要求将feature/electri-136-inertial-import改名为V3
commits: [feature/ELECTRI-136-gravity-ff]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-136, motion-20260925-03]
---

## 背景

用户明确要求重命名robot_description的惯性参数分支为`V3`。

## 改动

通过GitHub分支rename接口将`feature/electri-136-inertial-import`改名为`V3`，本地分支及
upstream同步。驱动source-lock和对应校验使用`V3`；模型提交仍为
`11f6d906dcb4cd5abba7cd4693afc9bb34c6a81e`，模型内容与运行行为不变。

## 验证

远端读回`V3`指向相同SHA，旧远端分支名不存在。`tools/quality_gate.sh`通过（288 passed、
13 skipped），`python3 tools/check_v3_branch_contract.py`通过。Git工作树与暂存区差异检查通过，
仅修改来源分支元数据、对应测试及记录；未重新运行无变化的模型/硬件测试。
域依赖方向及模型内容不变，提交/推送沿用用户已有授权。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 模型来源分支现为大小写精确的`V3`，完整SHA锁定及未实机验证状态保持。

## 遗留

台架标定和前馈准入条件不变；历史记录中的旧分支名保留为当时来源，不追改历史。
