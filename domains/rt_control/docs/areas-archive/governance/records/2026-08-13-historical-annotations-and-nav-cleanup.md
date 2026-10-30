---
id: governance-20260813-04
area: governance
title: ELECTRI-89 中低优先清理：historical 标注、导航收敛与孤儿文件删除
date: 2026-08-13
type: fix
trigger: ELECTRI-89（承接 governance-20260813-03 遗留项）
commits: []
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/quality_gate.sh 本地输出（83 tests OK，全 PASS）]
supersedes: []
related: [ELECTRI-89, governance-20260813-03]
---

## 背景

完成 ELECTRI-89 剩余的中低优先清理项。核心动作是把"已过时但仍被当作现行权威"
的文档显式标为 historical、把并行维护的文档目录收敛到 areas/ 索引，并删除已无
消费者的孤儿配置文件（用户裁决删除）。

## 改动

- 删除 `docker/cyclonedds.xml`（compose 与 runbook 模板均已不再挂载，无任何
  消费者；native 启动器清 `CYCLONEDDS_URI` 环境变量的行为不受影响）；
  `onboarding-knowledge-map.md` 配置位置表同步去引用。
- historical 标注三份：`baseline_report.md`（T-001 已关闭，注明 REQ 表不可恢复、
  追溯改用 areas 记录坐标）、`follow-up-modification-plan.md`（两项修改已核实在
  main 源码，状态表为 07-29 时点快照）、`plc-bms-merge-hardware-handoff.md`
  （工作区快照与两阶段提交流程已被 T-REL-010 取代；第 3/8 节仍有效）。
- `plc-bms-merge-hardware-handoff.md` 第 2 节 BMS/PLC 映射副本删除，改为指向
  `plc-bms-integration.md`（后者为当前权威）。
- 域 `README.md` 导航从 15 条收敛为 8 条核心 + areas/testing 指针（专题与历史
  文档归各功能区"历史锚点"索引，已核对覆盖完整）；Domain records 块补
  areas/ 与 testing/ 两行。
- 根 `docs/README.md` 重写为准确的 2 条目录清单，并写入"接口清单唯一副本"
  规则；根 `README.md` 仓库结构注释对 `docs/` 的定位同步修正（消除
  robot-wide vs 域内的自相矛盾）。
- `recover-power-loss` 三处并存收敛：runbook 恢复段注明完整序列以
  one-command-start.md 为准；native-development-workflow.md 注明原生序列多出的
  第 5、9 两步 CPU14 pin 门禁是有意差异。
- `host-setup-record.md` 文首新增 Host note：声明 "Prior target retained for
  audit: alfa-two" 起的后 2/3 属已退役主机与已放弃迁移尝试，当前主机事实
  （含 30 分钟 cyclictest）仅在其上方段落。
- `testing/cases/lifecycle.yaml` TC-LC-05 traces 更新：现行序列指向
  native-development-workflow.md，follow-up plan 降为 historical 来源论证。

## 验证

- 已验证（T0）：`tools/quality_gate.sh` 全 PASS（83 tests OK）；用例 YAML 解析
  通过；全仓库 grep 确认 cyclonedds.xml 无残留引用（保留项均为正当断言）。
- 未验证：无（纯文档与死文件清理，无运行时行为变更）。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `docs/` 目录只保留唯一接口实现视图与协作规范两类仓库级文档；接口清单
  以 `docs/cross-domain-interfaces.md` 为唯一副本，其他文档只链接不复制。
- F2: 全量文档索引的唯一维护点是 `docs/areas/` 各区（域 README 只保留最短
  上手路径）。

## 遗留

- ELECTRI-89 全部清理项至此完成，可随本分支 PR 一并关闭。
