---
id: governance-20260814-01
area: governance
title: 清空根 docs 并建立公共接口 vendor 门禁
date: 2026-08-14
type: decision
trigger: BQ-136
commits: [feature/robot-interfaces-vendoring]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [quality_gate 187/187, repository gate PASS, release catalog 33/33]
supersedes: [governance-20260813-04]
related: [BQ-136, BQ-137, contract-20260814-01, TC-ST-01, TC-ST-04]
---

## 背景

`docs/` 同时保存本地接口视图、目录入口和唯一协作规范，导致本地文档与上游契约形成
第二事实源。BQ-136 裁决清空该目录，但协作规范必须搬迁而不是删除。

## 改动

协作规范迁到仓库根 `collaboration-and-commit-standards.md`；删除根 `docs/README.md`
与本地跨域接口视图，并迁移所有未归档引用。仓库门禁把根协作规范列为必需文件并拒绝
任何 `docs/` 跟踪路径，同时禁止 `src/interfaces` 出现公共包。

新增 `source-lock.yaml` 与 `deps.repos` 的路径、URL、完整 SHA、契约版本和三个 vendored
包一致性检查；CI 与原生 bootstrap 均显式导入依赖并只构建 RT-Control 包闭包。

## 验证

RED 阶段的治理门禁测试先按预期失败。实现后 `tools/quality_gate.sh` 全绿：repository
architecture/file-hygiene gate PASS，187 个工具测试通过，repository gate 覆盖率 85%；
`python3 tools/release_test_runner.py validate` 返回 `OK: 33 cases valid`；`git diff --check`
和 staged diff 检查均通过。草稿 PR 首轮 governance job 通过；build job 首次真正导入
EtherCAT vendor 后，按预期暴露 CI 镜像缺少 `/usr/local/etherlab`，形成失败证据。workflow
corrective 改为用 native bootstrap 在隔离 vendor 根应用/核验冻结补丁，并按 `versions.env`
构建固定 IgH 用户态库；repository gate 新增两项不可退化断言，CI 重跑待提交后触发。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 根 `docs/` 必须保持为空；唯一协作规范位于仓库根，接口视图来自固定 SHA 的
  `robot_interfaces` vendor。本事实取代 `governance-20260813-04#F1`。
- F2: `src/interfaces/source-lock.yaml` 是发布身份元数据，必须与 `deps.repos` 中
  `src/vendor/robot_interfaces` 的完整 SHA 严格一致。
- F3: 已归档 records 中的旧路径引用保持历史原样；迁移事实只在本记录和当前索引中表达。

## 遗留

上游 PR squash/merge 后必须更新最终 pin；BQ-137 解除前不得合并或发布。Docker 完整闭包
已构建为本地测试镜像，GitHub CI 尚未形成证据。
