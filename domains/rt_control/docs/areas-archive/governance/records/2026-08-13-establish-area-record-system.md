---
id: governance-20260813-01
area: governance
title: 建立功能区开发记录体系
date: 2026-08-13
type: decision
trigger: ELECTRI-74（测试体系设计中的追溯与记录组织部分；用户口头裁决：放弃重建冻结 REQ 表，改为按功能区从零建立开发记录）
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-74]
---

## 背景

冻结 REQ 表原件确认不在任何本地 checkout、git 历史或外部文档目录中，仅存
33+ 个编号的散落引用；从引用重建的草稿经用户评审后被否决。用户裁决：不再恢复
REQ 表，改为按功能区从零建立开发记录体系，作为后续"什么已被决定、什么已被验证"
的唯一事实源，并为发布前测试体系（ELECTRI-74）提供追溯坐标。

## 改动

无代码改动。新增 `domains/rt_control/docs/areas/`：

- `README.md`：9 功能区划分（ecat-axes、canopen-chassis、lifecycle、motion、
  io-power、contract、realtime-host、release-deploy、governance）、归属规则与
  记录规则；
- `TEMPLATE.md`：记录格式（YAML frontmatter + 背景/改动/验证/结论与冻结事实/
  遗留五段）；
- 9 个功能区 `README.md` 骨架，各含冻结事实表、记录索引和历史锚点；
- `domains/rt_control/AGENTS.md` 第 4 节新增开发记录要求；
- `domains/rt_control/PROGRESS.md` 追加时间线降级说明与本条索引行。

## 验证

- 已验证（T0）：`tools/quality_gate.sh` 在本改动工作区通过（95 项策略测试、
  repository gate、shell 语法门禁）。
- 未验证：记录体系对日常开发流的适用性只能由后续真实记录检验；本条即首个样例。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: RT-Control 开发记录按 9 个功能区组织，归属规则以 owner 包判定、每条记录
  唯一归属；划分与规则见 `docs/areas/README.md`。
- F2: 记录格式固定为 `TEMPLATE.md`（frontmatter 字段 + 五段正文）；记录合并后
  不可修改，推翻结论须写新记录并用 `supersedes` 指向旧记录。
- F3: `PROGRESS.md` 自 2026-08-13 起降级为纯时间线索引；其历史表格冻结，
  不迁移、不修改。
- F4: 冻结 REQ 表原件丢失且不再重建；需求/事实追溯坐标改用 `<记录id>#F<n>`，
  测试用例与后续裁决据此引用。

## 遗留

- 尚未在 `repository_gate.py` 中加"改动 src/rt_control 的 PR 必须附带对应区
  记录"的机器检查，先靠 AGENTS.md 约定执行，跑顺后再固化。
- ELECTRI-74 测试用例模板的 `covers`/追溯字段需按 F4 的坐标格式对齐。
- 各功能区"冻结事实"表为空，随后续记录逐步填充；历史锚点仅供检索，不回填。
