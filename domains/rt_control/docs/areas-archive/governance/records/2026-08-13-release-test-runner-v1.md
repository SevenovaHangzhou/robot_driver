---
id: governance-20260813-06
area: governance
title: 首版测试 runner、JSON 报告与 baseline v0 登记（ELECTRI-76）
date: 2026-08-13
type: feature
trigger: ELECTRI-76
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/quality_gate.sh 本地输出（120 passed，runner 覆盖 82%），/tmp 冒烟输出（validate 33 用例、plan 24 选集、run 报告 INCOMPLETE、delta REVIEW/MISSING）]
supersedes: []
related: [ELECTRI-76, ELECTRI-80, governance-20260813-02]
---

## 背景

按 ELECTRI-76 实现测试执行与报告层。TDD 流程（先写测试确认 RED 再实现），
测试驱动过程中发现并修正两处既有错误（见"结论"F4/F5）。

## 改动

- 新增 `tools/release_test_runner.py`：子命令 validate（schema 校验）/
  plan（选集 + T3/T4 人工执行卡）/ identity（发布身份六元组 + 一致性异常，
  TC-ST-04 脚本化）/ run（执行 auto 用例 + 合并人工结果目录 + JSON 报告 +
  人类摘要）/ delta（候选 vs baseline）。安全边界硬编码：仅自动执行
  risk∈{T0,T1,T2} 且 writes 四位全 no 的用例，违规声明抛 AutoExecutionRefused；
  绝不代替输入确认短语。
- 新增 `tools/tests/test_release_test_runner.py`：22 个用例，覆盖目录校验、
  选集、自动执行安全边界（含 T3/带写位拒绝）、发布身份采集与异常发现
  （fixture git 仓库）、reference 结果坐标强制、门禁三态判定、执行卡不含
  确认短语断言、delta 计算与 CLI 全流程。
- 新增 `domains/rt_control/testing/baselines/v0.yaml`：登记 180 s 性能窗口
  六项指标（来源 docker-deployment-performance-summary.md，当前主机）；
  **cyclictest 显式排除**——当前主机 30 分钟运行被断电中止（BQ-105，T-009
  未完成），host-setup-record 中的完整记录属已退役 alfa-two 主机。
- `testing/README.md`：新增 `auto` 字段定义与 Runner 用法/结果回填约定；
  Baseline 节改为如实描述 cyclictest 缺口。
- `testing/cases/`：TC-ST-01、TC-RT-04 增加 `auto` 命令；全部 `v010_gate: no`
  加引号（修 YAML 1.1 布尔化陷阱）。
- `host-setup-record.md` Host note 修正（cyclictest 归属 alfa-two）。
- `quality_gate.sh` NOTICE 覆盖率清单加入 release_test_runner.py。

## 验证

- 已验证（T0）：quality_gate 全 PASS，120 个测试（含 22 个新增）通过；
  runner 覆盖率 82%；端到端冒烟：validate=33 用例 0 错误、plan --gate v010
  =24 条选集+执行卡、run 产出报告且正确判 INCOMPLETE / 捕获 dirty 工作区
  异常、delta 输出 REVIEW/MISSING/NEW 三类行。
- 未验证：`--execute` 实跑 TC-ST-01 全门禁链（会递归调 quality_gate，留待
  干净候选 commit 上执行）；`--image` 路径的 docker inspect 分支（本机无候选
  镜像）；目标机上 TC-RT-04 auto 执行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: runner 自动执行安全边界 = risk∈{T0,T1,T2} 且 writes 全 no，由代码强制
  且有拒绝路径测试；执行卡与 runner 输出不得包含任何确认短语明文。
- F2: 报告 schema `rt-control-release-test-report/v1`：发布身份六元组 +
  identity_findings + 逐用例三态 + gate_verdict(PASS/FAIL/INCOMPLETE)。
- F3: baseline v0 只含当前主机 180 s 性能窗口指标；**当前主机没有有效
  cyclictest baseline**，TC-RT-01 首次通过后以新版本补登记（不改 v0）。
- F4: 用例总数勘误：33（非此前记录的 28；分项 5+7+6+4+4+2+5 正确，总和加错。
  governance-20260813-02 标题与正文、Linear 评论已同步勘误）。
- F5: 用例 YAML 中 `no` 类枚举值必须加引号，防 YAML 1.1 布尔化；已全量修正。

## 遗留

- delta 阈值仍 TBD（ELECTRI-80 裁决），verdict 固定 REVIEW。
- TC-ST-02/03 的 auto 化需参数注入（colcon 环境、legacy checkout 路径），随
  CI 集成再做。
- 人工结果目录 `results/` 的样例文件未提供，首次实机执行时补。
