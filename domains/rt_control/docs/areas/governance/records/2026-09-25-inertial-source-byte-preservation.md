---
id: governance-20260925-02
area: governance
title: ELECTRI-136 新机械惯性来源沿用精确哈希格式检查
date: 2026-09-25
type: fix
trigger: ELECTRI-136；新惯性模型包含4份保持原始字节的CAD导出
commits: [feature/ELECTRI-136-gravity-ff]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [governance-20260925-01, motion-20260925-03]
---

## 背景

新机械参数合入模型后，构建副本同时导入其4份无末尾换行的原始JSON/URDF。

## 改动

沿用既有精确路径＋SHA-256校验机制，登记arm_inertial_20260924的4份原件，总数由8变为12。
没有增加目录级例外；原文改动、敏感内容及未登记文件仍按原检查失败。

## 验证

`tools/quality_gate.sh`：288 passed、13 skipped，门禁覆盖率84%，PASS。
既有回归测试覆盖全部12份原件、内容变动及敏感内容拒绝；模型原始文件哈希保持一致。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本次4份机械原件仅以匹配精确路径及哈希的方式保持字节，其他规则不放宽。

## 遗留

本机无ShellCheck，CI仍强制；完整CI未执行。无硬件行为变化。
