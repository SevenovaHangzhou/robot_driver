---
id: governance-20260813-02
area: governance
title: 首版发布前测试用例目录（7 分类 33 用例）
date: 2026-08-13
type: feature
trigger: ELECTRI-74
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-74, ELECTRI-75, governance-20260813-01]
---

## 背景

ELECTRI-74 要求建立可重复执行的发布前测试体系：统一用例模板、风险分层、执行
授权边界、七类测试分类、机器可读报告与 baseline。本记录交付首版用例目录，
供人工评审后拆分子任务实现 runner。

## 改动

无代码改动。新增 `domains/rt_control/testing/`：

- `README.md`：T0–T4 风险分层（映射既有确认短语授权机制，不新设授权）、七分类、
  用例 YAML schema、报告三态与 V0.10 最小门禁定义、baseline 登记口径；
- `cases/`：7 个分类文件共 33 个用例（静态 5、只读运行 7、生命周期 6、功能运动 4、
  实时性能 4、长稳 2、异常恢复 5）。

## 验证

- 已验证（T0）：`tools/quality_gate.sh` 通过（含新增 YAML 语法检查）。
- 未验证：用例可执行性未逐条实跑；`status: automated` 项沿用既有 CI 证据，
  `manual/planned/blocked` 项待 runner 子任务与现场执行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 测试风险分层为 T0 静态 / T1 mock / T2 实机只读 / T3 生命周期 / T4 功能运动；
  发布门禁自动部分只到 T2，T3/T4 由现场人工授权执行、机器记录。
- F2: 用例 ID 规则为 `TC-<ST|RO|LC|FM|RT|LS|AR>-<NN>`，ID 不复用，作废置
  `status: retired`。
- F3: V0.10 最小门禁 = 全部 `v010_gate: run` 用例在冻结候选上执行通过 +
  全部 `v010_gate: reference` 用例给出历史证据坐标与差异说明。
- F4: 已知阻塞项显式入册：TC-AR-03（BQ-132）置 blocked，TC-AR-05（BQ-121）登记
  部署约束；阻塞用例不得从门禁清单中静默移除。

## 遗留

- runner、JSON 报告生成与 baseline delta 工具未实现（拆子任务）。
- TC-RO-01/02 待 launch_testing 自动化；TC-ST-04 六元组比对待脚本化。
- delta 阈值、TC-LS-02 漂移阈值为 TBD，待裁决。
- 增量执行（covers 字段驱动的 scope resolver）未实现。
