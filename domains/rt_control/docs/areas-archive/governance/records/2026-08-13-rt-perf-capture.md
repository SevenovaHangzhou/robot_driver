---
id: governance-20260813-09
area: governance
title: T2 性能采集脚本化（ELECTRI-80 脚本部分）
date: 2026-08-14
type: feature
trigger: ELECTRI-80（BQ-134 裁决后剩余的采集脚本化）
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/quality_gate.sh 全 PASS（184 tests），rt_perf_capture 覆盖 95%]
supersedes: []
related: [ELECTRI-80, BQ-134, BQ-064, governance-20260813-07]
---

## 背景

BQ-134 阈值已裁决，缺一条命令产出候选指标 YAML 的采集工具（TC-RT-01..03）。

## 改动

- 新增 tools/rt_perf_capture.py：纯函数解析器（mpstat/pidstat/free/ip -details/
  cyclictest 尾行/wc_error 差值）+ CLI（capture / wc-error / cyclictest / parse）。
  输出键名与 baseline v0 严格同名。cyclictest 模式只打印 runbook 8.1 注入命令
  供人工执行（--print-command），解析 --from-log，Max>100us 退出码 1（BQ-064）；
  脚本自身绝不调 docker。
- 新增 53 个 pytest（TDD），fixture 精确复现 baseline v0 数值（4.945/23.230/577.3）。
- TC-RT-02/03 entry 更新为脚本入口；quality_gate NOTICE 清单加入本模块。

## 验证

- 已验证（T0）：quality_gate 全 PASS（184 tests），新模块覆盖 95%，33 用例
  validate 通过。
- 未验证：目标机实采（本机无 sysstat/rt-tests/ROS）；pidstat/mpstat 列格式为
  按文档数值重构，首次实采需对表确认；CAN berr 块、/diagnostics 实际 echo
  格式同样待首采确认。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: cpu14_busy_* 是 mpstat 核口径（100−%idle），非 pidstat 进程口径；采集
  工具两者都采，核口径入判定键，进程口径仅信息性——防止 baseline 含义漂移。

## 遗留

- TC-RT-01 首次在当前主机通过后补登记 cyclictest baseline（governance-20260813-06#F3）。
