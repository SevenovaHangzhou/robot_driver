---
id: governance-20260813-05
area: governance
title: 修复门禁测试收集缺口并修订覆盖率口径（ELECTRI-78）
date: 2026-08-13
type: fix
trigger: ELECTRI-78
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/quality_gate.sh 本地输出（98 passed，门禁集 85%，全 PASS）]
supersedes: []
related: [ELECTRI-78, governance-20260813-03]
---

## 背景

`quality_gate.sh` 用 `unittest discover` 收集 `tools/tests`，而
`test_rt_io_integration.py` 的 15 个测试是模块级 pytest 函数，自入库起从未被
CI/pre-commit 执行且失败不会报错。同时协作规范写"`tools/**` 覆盖率 80%"，实际
门禁只含 2 个文件；实测全量口径仅 78%（`rt_control_thread_affinity.py` 0%、
`diff_legacy.py` 63%、`rt_control_axis_state_check.py` 69%），直接扩大门禁会
误伤，补测试属独立工作量。按 ELECTRI-78 预留的第二条路径处理：修收集 + 规范
口径修订 + 未门禁部分显式可见。

## 改动

- `tools/quality_gate.sh`：收集改为 `python3 -m pytest tools/tests -q`
  （coverage `--source=tools` 全量采样）；门禁报告保持
  repository_gate + pr_contract_gate 合并 ≥80%；新增 NOTICE 段打印其余三个
  脚本的覆盖率（不设阈值）。
- `docs/collaboration-and-commit-standards.md` 9.5 节：覆盖率口径由
  "`tools/**` 80%" 修订为"门禁集 ≥80% + 其余脚本打印可见暂不设阈值"，
  写入 2026-08-13 实测基线数值与"提升覆盖属独立任务"；明确收集器为 pytest。
- `.github/workflows/rt-control-ci.yml` governance job：apt 增加
  `python3-pytest`。

## 验证

- 已验证（T0）：`tools/quality_gate.sh` 全 PASS——98 个测试收集执行
  （83 unittest 类 + 15 此前静默的模块级函数），门禁集 85%（repository_gate
  87%、pr_contract_gate 72%），NOTICE 段输出 63%/69%/0% 三项。
- 未验证：CI 容器内的 `python3-pytest` 安装未实跑（下次 push 触发 governance
  job 即验证）。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `tools/tests` 收集器为 pytest；新增测试可用 unittest 类或模块级函数，
  两者都会被收集，静默漏收路径已封死。
- F2: 覆盖率门禁口径 = 门禁集（repository_gate + pr_contract_gate）合并 ≥80%；
  未门禁脚本覆盖率必须在门禁输出中可见，纳入门禁前须先补测试。

## 遗留

- `pr_contract_gate.py` 单文件 72%（低于 80，被合并平均掩盖）与
  `rt_control_thread_affinity.py` 零测试是已知欠账，候选后续任务，不阻塞本项。
- CI 内 pytest 安装待下次 push 验证。
