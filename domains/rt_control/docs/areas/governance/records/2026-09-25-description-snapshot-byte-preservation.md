---
id: governance-20260925-01
area: governance
title: ELECTRI-136 保留公共模型哈希冻结的 CAD 原件字节
date: 2026-09-25
type: fix
trigger: ELECTRI-136 V3.1.1 模型迁移；上游8份原件没有末尾换行
commits: [feature/ELECTRI-136-gravity-ff]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [motion-20260925-02]
---

## 背景

公共 robot_description@ec69ca0 对8份 CAD 原始导出保存了 SHA-256，而文件没有末尾换行。
自动补换行会破坏上游来源校验和精确模型副本身份。

## 改动

`tools/repository_gate.py` 仅在这8个精确路径及 SHA-256 同时匹配时允许缺少末尾换行。
修改其内容即报 immutable source hash mismatch；未登记文件继续执行原格式规则。
敏感内容、冲突标记、其他空白、语法及架构检查继续执行，没有排除整个 model_sources 目录。
哈希来自随公共提交导入的两个 integration manifest，源文件保持原文。

## 验证

新增两项回归测试，覆盖8份原件通过、补换行也判定被修改、伪造内容/敏感内容不被例外遮蔽、
其他文件仍检查末尾换行。`tools/quality_gate.sh`：288 passed、13 skipped，门禁覆盖率84%，PASS；
ShellCheck 本机缺失，仍由 CI 强制。实际8份原件与公共模型 Git blob 逐字节一致。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: CAD 原件格式兼容只针对固定路径与固定内容，不形成可被新文件继承的目录级例外。
- F2: 更新公共模型 pin 时需同步审查这些原件身份；不能通过补换行伪造原文不变。

## 遗留

无新增硬件事实或运行行为；完整 CI 尚未执行，未提交/推送。
