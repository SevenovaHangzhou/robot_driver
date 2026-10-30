---
id: governance-20260813-07
area: governance
title: BQ-134 性能阈值裁决落地（ELECTRI-80 裁决部分）
date: 2026-08-13
type: decision
trigger: ELECTRI-80（用户 2026-08-13 裁决四项阈值）
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/quality_gate.sh 全 PASS（124 tests）, pytest 24/24]
supersedes: []
related: [ELECTRI-80, BQ-134, governance-20260813-06]
---

## 背景

ELECTRI-80 的阈值 TBD 项由用户裁决（记 BQ-134）：CPU14 绝对上限 10%/40%、
RSS 700 MiB、静置漂移 0.5°/5mm、零容忍项非零记 REVIEW 不自动 FAIL。

## 改动

- BLOCKED-questions.md 追加 BQ-134；新增 testing/thresholds.yaml 固化阈值。
- compute_delta 支持 thresholds（FAIL/OK/REVIEW 判定），delta 子命令加
  --thresholds，有 FAIL 时退出码 1；新增 2 个测试。
- TC-RT-02/03、TC-LS-02 判据 TBD 全部消除；testing/README 同步。

## 验证

- 已验证（T0）：quality_gate 全 PASS，24/24 runner 测试，33 用例 validate 通过。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 性能判定阈值以 BQ-134 + thresholds.yaml 为唯一事实源，调整须新 BQ。

## 遗留

- ELECTRI-80 的脚本化部分（cyclictest/资源窗口/总线增量采集脚本）未做。
