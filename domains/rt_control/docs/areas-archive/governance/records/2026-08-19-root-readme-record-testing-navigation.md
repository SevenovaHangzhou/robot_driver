---
id: governance-20260819-01
area: governance
title: 根 README 补齐记录与测试体系导航
date: 2026-08-19
type: fix
trigger: ELECTRI-74（从未合并提交 f3efbb40 重新落地仍有效的根 README 导航）
commits: [fd24f9f4d46582d2c429a588a4a1f1d08ebc2f84]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: []
supersedes: []
related: [ELECTRI-74, governance-20260813-01, governance-20260813-08, governance-20260814-01]
---

## 背景

PR #4 评审期产生的提交 `f3efbb40` 补充了根 README 中的功能区记录体系和发布前
测试入口，但该提交未合入 `main`。此后公共接口权威源、vendor 契约视图与根目录
治理已经更新，旧提交不能直接 cherry-pick，需要在最新 `main` 上重新落地仍有效的导航。

## 改动

- 根 README 的仓库结构与文档入口补充 `domains/rt_control/docs/areas/` 和
  `domains/rt_control/testing/`。
- 本地质量门禁区分 scoped 内环、仓库级 quality gate，以及发布候选用例目录校验和
  V0.10 执行计划；完整 identity、run、delta 用法继续由 testing README 维护。
- 保留独立 `robot_interfaces` 权威源，以及 `deps.repos`、vendored 契约视图和
  `src/interfaces/source-lock.yaml` 的身份锁定链，不恢复已删除的根 `docs/`。
- 仅更新导航和治理记录；不修改测试 schema、用例、runner、公共接口或运行行为。

## 验证

- `tools/run_scoped_tests.sh`：PASS；4 个文档文件均路由到 repository gate。
- `tools/quality_gate.sh`：PASS；190 项策略测试通过，门禁覆盖率 83%，EtherCAT 停机与
  current-IPC 策略检查通过。本机没有 ShellCheck，PR CI 必须补跑。
- `python3 tools/release_test_runner.py validate`：PASS；33 个用例有效。
- `python3 tools/release_test_runner.py plan --gate v010`：PASS；V0.10 计划正常生成。
- 本地 Markdown 链接检查：PASS；4 个变更文档中的 50 个本地链接均存在。
- `git diff --check` 与 `git diff --cached --check`：PASS。

本记录不授权使能或运动。

## 结论与冻结事实

无新增冻结事实。

## 遗留

- 首个实现提交 SHA 在同一 PR 内回填；GitHub CI 仍需执行强制 ShellCheck 和完整构建。
- 本次是 T0 文档变更，未执行且不需要容器、Mock、HIL 或实机操作。
