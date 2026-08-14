---
id: governance-20260813-08
area: governance
title: 增量测试 scope resolver（ELECTRI-81）
date: 2026-08-13
type: feature
trigger: ELECTRI-81（用户报告小改动触发全量测试拖慢内环）
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/quality_gate.sh 全 PASS（131 tests），scoped plan 冒烟输出]
supersedes: []
related: [ELECTRI-81, governance-20260813-02]
---

## 背景

小改动被 AI 保守兜底成全量测试；"受影响包"此前无机器可执行定义。

## 改动

- 新增 tools/scoped_tests.py + tools/run_scoped_tests.sh：从 git 改动集
  （含已暂存与未跟踪）推导最小验证集——纯文档→repository_gate；tools/**→
  quality_gate；单包→colcon test --packages-above（colcon 自算反向依赖）；
  src/interfaces/**→三接口包及全部下游；patches/deps.repos/versions.env/
  docker/CI→提示真·全量。受影响目录用例按 covers 字段聚合列出（信息性，
  非第二份硬编码表）。
- 域 AGENTS.md 5.1 增加执行策略：内环 scoped、CI 全量兜底、发布门禁走
  release_test_runner，不得以增量结果替代。
- 9 个单测（TDD）；开发中修正一个真 bug：初版漏收已暂存改动。

## 验证

- 已验证（T0）：quality_gate 全 PASS（131 tests）；真实分支 diff 冒烟分类正确。
- 未验证：colcon --packages-above 动作在有 ROS 环境的机器上的实际执行
  （本机无 colcon 工作区）。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 内环验证入口 = tools/run_scoped_tests.sh；全局影响路径清单见脚本
  FULL_SUITE_PATHS，扩充须同步用例 covers 口径。

## 遗留

- scoped_tests.py 覆盖率 86%（informational 可见）；--run 实际执行分支未全测。
