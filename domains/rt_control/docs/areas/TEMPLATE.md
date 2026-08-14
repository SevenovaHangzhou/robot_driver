---
# id 格式：<area>-<YYYYMMDD>-<两位序号>，与文件名对应
id: <area>-20260813-01
# 八区之一：ecat-axes | canopen-chassis | lifecycle | motion | io-power |
#           contract | realtime-host | release-deploy | governance
area: <area>
title: <一句话标题>
date: 2026-08-13
# feature=新能力 | fix=缺陷修复 | corrective=对已交付结论的纠错 |
# commissioning=实机调试/验收 | decision=裁决落地 | investigation=调查结论
type: feature
# 起因：Linear issue（如 ELECTRI-74）/ BQ 编号 / 上一条记录 id / 口头需求（写明谁提出）
trigger: <来源>
# 合并后填完整 40 位 SHA；PR 未合并时先填分支名，合并后由同 PR 的后续提交补齐
commits: []
# 本次验证发生在哪个环境
env: native | docker | both | none
# 本次验证实际达到的层级：T0 静态 | T1 mock 运行 | T2 实机只读 | T3 生命周期 | T4 功能运动
risk: T0
# 本次操作实际执行了什么写动作；与测试体系授权位同义
writes: { reset: no, enable: no, motion: no, plc: no }
# PASS | FAIL | PARTIAL | UNVERIFIED（只有代码/静态检查、无运行证据时必须写 UNVERIFIED）
verified: UNVERIFIED
# 证据文件路径列表（日志、截图、报告 JSON）；没有写 []
evidence: []
# 本记录推翻/覆盖的旧记录 id 列表；没有写 []
supersedes: []
# 关联的 BQ 编号、测试用例 ID、其他记录 id
related: []
---

## 背景

为什么做这件事，一段话。引用 trigger 即可，不重复其内容。

## 改动

改了什么：包/文件/配置项。关键数值必须写明（旧值 → 新值）。不贴 diff，diff 在
commit 里。无代码改动的记录（如 commissioning、investigation）写"无代码改动"。

## 验证

跑了什么、在哪一层（对应 frontmatter 的 risk）、命令是什么、量化结果是多少。
已验证和未验证必须分开列；从源码检查推导的结论不算验证。

末尾必须有一行授权声明，二选一：

- 本记录不授权使能或运动。
- 本次实机操作经现场授权，操作人：<姓名>，确认短语入口：<脚本/命令>。

## 结论与冻结事实

本记录确立了什么。每条冻结事实单独一行、以 F 编号，供测试用例与后续记录引用
（引用坐标格式：`<记录id>#F1`）：

- F1: <事实一>
- F2: <事实二>

没有新冻结事实时写"无新增冻结事实"。

## 遗留

没做完的、有风险的、下一条记录该接的。没有写"无"。
