---
id: governance-20260920-01
area: governance
title: V3 完整门禁保留下的分层 CI 与依赖缓存
date: 2026-09-20
type: decision
trigger: 用户要求缩短 CI，增加固定 ROS/EtherLab 基础镜像、依赖缓存和 head-only 快速任务
commits: [feature/v3-head-can-integration]
env: both
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - "PR #44 run 35449639274: governance PASS 45 s, build PASS 12 min 1 s"
  - "tools/tests/test_ci_scope.py: 3 passed"
  - "tools/tests/test_repository_gate.py -k ci_workflow: 1 passed"
  - "tools/quality_gate.sh: 294 passed, 13 skipped, policy coverage 83%"
  - "docker buildx build --check: no warnings"
  - "GitHub workflow YAML BaseLoader parse: PASS"
supersedes: []
related: [governance-20260915-01, governance-20260917-01]
---

## 背景

V3 PR 的完整 ROS 闭包约需十余分钟，其中系统包安装、vendor 拉取、EtherLab 编译和
C++ 重编译存在重复开销。优化必须回答的核心问题是：缩短反馈时间时，不能让头部局部
改动绕过完整 V3 合并门禁，也不能把普通源码 PR 重新绑定到产品 Docker 封装流程。

## 改动

- 新增独立 `rt-control-ci-base` workflow 和 CI 专用 Dockerfile，在 `main`/`v3`
  对基础镜像输入变更时发布 ROS Humble、构建工具和固定 EtherLab userspace 镜像。
  镜像标签同时包含 EtherLab commit 与仓库 commit；主 CI 通过仓库变量
  `RT_CONTROL_CI_IMAGE` 选择该精确标签，未设置时保留公开 ROS 镜像回退路径。
- 完整 `build` 缓存 vendor checkout、apt archives 和 ccache，并保持
  `governance -> build`、ROS test dependencies、完整 V3 package closure、URDF 和
  分支合同检查不变。
- 新增可单测的 `tools/ci_scope.py`。只有头部 CAN hardware/controller 及五个专属
  bringup 文件组成的非空变更集才启动 `head-fast`；共享 machine profile、CI、文档
  或其他子系统变更均不得被归为 head-only。
- repository gate 新增约束：完整 V3 `build` job 不得带 `if` 条件。快速任务只提供
  更早反馈，不能替代合并前完整闭包。

CI 基础镜像仅用于构建环境复用，不是运行产品镜像，不改变
`governance-20260915-01` 确立的人工产品 Docker 封装策略。

## 验证

- `python3 -m pytest -q tools/tests/test_ci_scope.py`：3 passed，含 workflow 同款
  stdin/stdout CLI 调用。
- `python3 -m pytest -q tools/tests/test_repository_gate.py -k ci_workflow`：1 passed，
  并用 RED 用例确认带条件的完整 build 会被拒绝。
- `tools/quality_gate.sh`：294 passed、13 skipped，策略覆盖率 83%。
- `docker buildx build --check ... -f docker/rt-control-ci/Dockerfile .`：通过，
  no warnings。
- 两个 GitHub workflow 使用 `yaml.BaseLoader` 解析通过，`git diff --check` 通过。
- PR #44 在 vendor cache 边界修正后，回退镜像路径的完整 build 为 12 min 1 s，
  相比本 PR 首轮 15 min 46 s 缩短 3 min 45 s。

当前提交的远端完整 CI、基础镜像实际发布和设置仓库变量后的计时仍待完成，因此本记录
保持 PARTIAL。本记录不授权使能或运动。

## 结论与冻结事实

- F1: 所有代码 PR 的完整 V3 `build` 都是无条件合并门禁，不因 head-only 分类跳过。
- F2: `head-fast` 只作头部专属改动的提前反馈，不替代 `governance` 或完整 `build`。
- F3: CI 基础镜像必须用包含 EtherLab commit 与仓库 commit 的精确标签；变量未配置时
  必须保留可工作的公开 ROS 镜像回退路径。
- F4: vendor、apt 和 ccache 可以跨 CI run 复用，但依赖源身份仍由 `deps.repos` 和
  `versions.env` 的完整 commit pin 决定。

## 遗留

合并后运行 `rt-control-ci-base`，确认 GHCR 包可读，再把其精确标签写入仓库变量
`RT_CONTROL_CI_IMAGE`。随后分别记录基础镜像冷/热缓存耗时，并由仓库管理员确认
`governance` 与 `build` 仍为 `v3` 的 required checks。
