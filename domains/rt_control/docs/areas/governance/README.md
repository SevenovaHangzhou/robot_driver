# governance — 门禁、CI、测试体系与协作规则

**范围**：仓库门禁与策略测试、CI 工作流、pre-commit、协作与提交规范、
发布前测试体系（用例模板、runner、报告、baseline）、本开发记录体系自身。
**Owner 包/资产**：`tools/quality_gate.sh`、`tools/repository_gate.py`、
`tools/pr_contract_gate.py`、`tools/diff_legacy.py`、`tools/tests/`、
`.github/workflows/`、`.pre-commit-config.yaml`、各级 `AGENTS.md`、
`collaboration-and-commit-standards.md`、`domains/rt_control/docs/areas/`。

不属于本区：具体功能的验证记录（归各功能区，即使证据由门禁产生）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| F1 | 开发记录按 9 功能区组织，规则见 areas/README.md | [governance-20260813-01](records/2026-08-13-establish-area-record-system.md)#F1 | 有效 |
| F2 | 记录格式为 TEMPLATE.md：frontmatter + 五段正文，合并后不可改，推翻用 supersedes | [governance-20260813-01](records/2026-08-13-establish-area-record-system.md)#F2 | 有效 |
| F3 | PROGRESS.md 自 2026-08-13 起降级为时间线索引，历史段冻结 | [governance-20260813-01](records/2026-08-13-establish-area-record-system.md)#F3 | 有效 |
| F4 | 冻结 REQ 表原件确认丢失，不重建；需求追溯坐标改用 `<记录id>#F<n>` | [governance-20260813-01](records/2026-08-13-establish-area-record-system.md)#F4 | 有效 |
| F5 | 测试风险分层 T0–T4，发布门禁自动部分只到 T2 | [governance-20260813-02](records/2026-08-13-test-case-catalog-v1.md)#F1 | 有效 |
| F6 | 用例 ID 规则 `TC-<分类码>-<NN>`，不复用 | [governance-20260813-02](records/2026-08-13-test-case-catalog-v1.md)#F2 | 有效 |
| F7 | V0.10 最小门禁 = v010_gate run 全过 + reference 全部给出历史坐标 | [governance-20260813-02](records/2026-08-13-test-case-catalog-v1.md)#F3 | 有效 |
| F8 | 根 docs 必须为空，协作规范位于仓库根，接口视图来自 vendor | [governance-20260814-01](records/2026-08-14-root-docs-and-vendor-governance.md)#F1 | 有效；取代 governance-20260813-04#F1 |
| F9 | source-lock 发布身份必须与 deps.repos 的 robot_interfaces pin 一致 | [governance-20260814-01](records/2026-08-14-root-docs-and-vendor-governance.md)#F2 | 有效 |
| F10 | 已归档 records 保留旧路径作为历史坐标 | [governance-20260814-01](records/2026-08-14-root-docs-and-vendor-governance.md)#F3 | 有效 |
| F11 | quality_gate 无条件运行驱动变体三层校验，两个入口命令均受 repository policy 保护 | [governance-20260818-01](records/2026-08-18-driver-variant-gate.md)#F1 | 有效 |
| F12 | 工具覆盖率硬门禁扩展为 repository、PR contract 与三个 driver-variant 模块合并 ≥80% | [governance-20260818-01](records/2026-08-18-driver-variant-gate.md)#F2 | 有效；取代 governance-20260813-05#F2 |
| F13 | manifest 变更触发全量；投影消费者变更触发 quality gate 与所属包测试 | [governance-20260818-01](records/2026-08-18-driver-variant-gate.md)#F3 | 有效 |

## 记录索引（倒序）

- 2026-08-18 [驱动变体三层一致性门禁（ELECTRI-94）](records/2026-08-18-driver-variant-gate.md) — feature，PASS（T1）
- 2026-08-14 [清空根 docs 并建立公共接口 vendor 门禁](records/2026-08-14-root-docs-and-vendor-governance.md) — decision，PASS（T0）
- 2026-08-14 [T2 性能采集脚本化（ELECTRI-80 脚本部分）](records/2026-08-13-rt-perf-capture.md) — feature，PASS（T0）
- 2026-08-13 [增量测试 scope resolver（ELECTRI-81）](records/2026-08-13-scoped-tests.md) — feature，PASS（T0）
- 2026-08-13 [BQ-134 性能阈值裁决落地](records/2026-08-13-bq134-thresholds.md) — decision，PASS（T0）
- 2026-08-13 [首版测试 runner、JSON 报告与 baseline v0 登记（ELECTRI-76）](records/2026-08-13-release-test-runner-v1.md) — feature，PASS（T0）
- 2026-08-13 [修复门禁测试收集缺口并修订覆盖率口径（ELECTRI-78）](records/2026-08-13-pytest-collection-and-coverage-scope.md) — fix，PASS（T0）
- 2026-08-13 [ELECTRI-89 中低优先清理：historical 标注、导航收敛与孤儿文件删除](records/2026-08-13-historical-annotations-and-nav-cleanup.md) — fix，PASS（T0）
- 2026-08-13 [文档矛盾修正与重复测试清理](records/2026-08-13-doc-contradiction-and-test-dedup.md) — fix，PASS（T0）
- 2026-08-13 [首版发布前测试用例目录（7 分类 33 用例）](records/2026-08-13-test-case-catalog-v1.md) — feature，UNVERIFIED→随 PR 合入生效
- 2026-08-13 [建立功能区开发记录体系](records/2026-08-13-establish-area-record-system.md) — decision，UNVERIFIED→随 PR 合入生效

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-GOV-001/002、T-DOC-001/002
- `collaboration-and-commit-standards.md`（第 9.5 节五层测试要求、第 10 节 CI）
- 相关 Linear：ELECTRI-74（测试体系设计）
