---
id: governance-20260813-03
area: governance
title: 文档矛盾修正与重复测试清理（ELECTRI-89 高优先项）
date: 2026-08-13
type: fix
trigger: ELECTRI-89（重复性审查发现权威内容复制后漂移出矛盾、tools/tests 与包内双份维护重复用例）
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/quality_gate.sh 本地输出（83 tests OK，全 PASS）]
supersedes: []
related: [ELECTRI-89, ELECTRI-74, governance-20260813-02]
---

## 背景

跨区文档卫生清理，按"主要改动落区"归 governance（文档一致性约束本身是治理职责）。
两类问题：① 四份文档对 `docker/compose.yaml` 的描述停留在 BQ-128（2026-07-31，
Domain 0 + `rmw_fastrtps_cpp` 显式固定、无 DDS XML 挂载）之前，其中 runbook 还把
已关闭的"RMW 未固定"缺口列为生产前硬门禁——文档在制造不存在的发布阻塞项；
② `tools/tests` 中 12 个用例与 `control_api_adapter` 包内测试双份维护，且包内已
演进（新增 4 个测试未回流），tools 侧为旧版本。

## 改动

- `deployment-operations-runbook.md` 七处：Mock Domain 说明（42→0）、mock 模板
  （去 CycloneDDS 挂载、显式 `RMW_IMPLEMENTATION`）、生产配置核对清单
  （cap 补 `NET_RAW`、Domain 0、RMW 固定）、Phase H 门禁重写（保留网络暴露核对，
  移除虚假"RMW 未固定"阻塞）、故障表 RMW 行、`ros2 service call` 示例旧包名
  `robot_control_interfaces`→`robot_rt_control_interfaces`、证据清单标注
  ethercat commissioning 为历史拓扑。
- `onboarding-knowledge-map.md` 两处：已知差距表 CYCLONEDDS 行改为 BQ-128 现状
  （风险 高→中）；"验证边界"措辞不再把历史拓扑记录列为现行权威。
- `docker-deployment-performance-summary.md` 一处：DDS 配置行改为 compose 现状。
- `integration-readiness-summary.md`：删除第 2 节 19 行接口副本表（已漂移出
  源码不存在的旧包名），改为指向 `docs/cross-domain-interfaces.md` 的单一链接。
- `one-command-start.md`：5 处旧包名修正；契约版本 0.5.0→0.6.0。
- `tools/tests/test_control_api_adapter.py` 整文件删除（6/6 与包内
  `test_control_enable_adapter.py` 同名同逻辑，git 证实同批 commit 双份维护）；
  `test_vacuum_and_status_contract.py` 由 8 个裁至 2 个 tools 层独有断言
  （IDL 固定通道/无压力字段、PLC DI bit0/bit1 解码），6 个包内重复项删除。
  适配器逻辑覆盖归 `colcon test`（CI build job）。
- PROGRESS.md 历史段与 BLOCKED-questions.md 中的旧包名不修改（不可变历史记录）。

## 验证

- 已验证（T0）：`tools/quality_gate.sh` 全 PASS；tools/tests 83 个用例 OK
  （95−12，数目与删除项一致）；全仓库 grep 确认无残留旧包名/过时 CycloneDDS
  论述（保留项均为正当：native 清 env 行为、负例夹具、"不得设置"断言）。
- 未验证：runbook mock 模板修改后的容器命令未实机执行。

本记录不授权使能或运动。

## 结论与冻结事实

无新增冻结事实——本次是让文档回归既有裁决（BQ-128）与源码现状，不产生新决定。

## 遗留

- ELECTRI-89 中优先项未做：follow-up-modification-plan / plc-bms-handoff /
  baseline_report 的 historical 标注，域 README 导航瘦身，根 docs/README 改写，
  recover-power-loss 三处并存收敛，host-setup-record 分段声明。
- `docker/cyclonedds.xml` 文件本体已无任何消费者（compose 与 runbook 模板均不再
  挂载），是否删除待 ELECTRI-89 处理时裁决。
- `onboarding-knowledge-map.md:112` 仍把该孤儿文件列为配置位置，随上项一并处理。
