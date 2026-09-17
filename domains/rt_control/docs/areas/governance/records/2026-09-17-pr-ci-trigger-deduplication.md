---
id: governance-20260917-01
area: governance
title: PR CI 触发去重与描述编辑隔离
date: 2026-09-17
type: decision
trigger: 用户决定 PR 描述编辑不再触发完整 ROS 构建，并取消 PR 合并后的重复 main 构建
commits: [ci/pr-trigger-policy]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - "repository gate 与 PR contract 回归：22 passed"
  - "tools/quality_gate.sh：283 passed、13 skipped、策略覆盖率 84%"
  - "GitHub API: main Branch not protected (HTTP 404)"
supersedes: []
related: [TC-ST-01, TC-ST-02]
---

## 背景

原 `rt-control-ci` 同时监听 PR `edited` 和 `push(main)`。修改 PR 描述会取消同一 PR
正在运行的完整构建并重新执行约十五分钟的 ROS 闭包；PR 已通过相同完整门禁后，合并
产生的 main push 又执行一次同等构建。仓库规则禁止直接 push main，因此用户决定去除
这两类重复计算，同时保留合并前的强制源码门禁。GitHub API 实查显示 main 尚未配置
分支保护，因此该规则目前缺少平台强制执行。

## 改动

- `rt-control-ci` 只监听 PR `opened`、`synchronize`、`reopened`，继续顺序执行完整
  `governance -> build`，不再监听 `edited` 或 `push(main)`。
- 新增 `pr-metadata-governance`，只监听 PR `edited`，只运行 `governance` job 中的
  PR contract 校验；并发组按 PR number 隔离，不会取消代码事件触发的完整构建。
- repository gate 和策略测试固定上述事件、权限、job 和并发边界，防止后续回退。
- 协作规范与域 README 明确：取消 main 重复构建依赖禁止直接 push 和合并前
  `governance/build` 检查持续生效；分支保护仍需管理员配置。

## 验证

- `python3 -m pytest -q tools/tests/test_repository_gate.py -k 'ci_workflow or pr_metadata'`：
  2 passed。
- `python3 -m pytest -q tools/tests/test_repository_gate.py tools/tests/test_pr_contract_gate.py`：
  22 passed。
- `tools/quality_gate.sh`：283 passed、13 skipped，策略覆盖率 84%。
- workflow YAML 由 repository gate 使用 `yaml.BaseLoader` 解析并检查事件和 job 结构。
- `gh api repos/SevenovaHangzhou/robot_driver/branches/main/protection`：HTTP 404，
  `Branch not protected`。

未执行 ROS 构建：本次只修改 GitHub Actions 调度和只读策略检查，不涉及 ROS 源码、
依赖、Robot Model 或运行入口。远端 `edited`/PR 代码事件行为需由本 PR 验证。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 完整 ROS CI 只由 PR `opened`、`synchronize`、`reopened` 触发。
- F2: PR `edited` 只触发独立 contract governance，不得启动或取消完整构建。
- F3: PR 合并后的 `push(main)` 不重复全量构建；若允许直接 push 或绕过必选
  检查，必须先恢复 main 对应门禁。

## 遗留

本 PR 合入前需观察一次代码事件完整 CI，并编辑 PR 描述确认只出现轻量 governance。
GitHub main 当前未受保护；管理员仍需配置禁止直接 push、禁止 bypass，并要求
`governance` 与 `build`。workflow 不能代替这项平台设置。
