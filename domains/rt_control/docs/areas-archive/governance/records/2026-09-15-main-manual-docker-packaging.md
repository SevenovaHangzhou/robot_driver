---
id: governance-20260915-01
area: governance
title: main 稳定源码与 Docker 人工封装解耦
date: 2026-09-15
type: decision
trigger: 用户决定 main 不再强制 Docker，源码稳定后人工封装
commits: [work/electri-117-swerve-module-binding]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: []
supersedes: []
related: [BQ-148]
---

## 背景

原规则把每个进入 `main` 的 RT-Control 源码 PR 与生产镜像构建、容器启动证据绑定，
导致功能开发必须同步承担封装成本。用户决定将稳定源码集成和发布封装拆成两个阶段：
先稳定 `main`，再人工选择明确 SHA 封装。

## 改动

- 根/域 AGENTS、协作规范和 README 改为：普通 `main` 源码 PR 不强制 Docker。
- `native -> main` 不再以补齐容器为前置条件，仍禁止个人路径和不可复现临时修改。
- Docker/Compose 资产继续保留；Docker 资产变更和人工封装/正式发布仍要求镜像、
  容器 Mock、启动/停机、身份和 source SHA 证据。
- 发布测试 TC-RO-04 改为人工封装产物验证，不再表述为所有 main PR 的 smoke。
- 增加策略回归测试，防止旧“main 必须容器证据”措辞重新进入权威规则。

## 验证

- 对权威规则执行旧强制措辞扫描和新人工封装口径断言。
- 策略回归测试：3 passed。
- `tools/quality_gate.sh`：280 passed、13 skipped，策略覆盖率 83%；
  本机缺少 ShellCheck，保留 CI 检查。
- `python3 tools/release_test_runner.py validate` 与 `git diff --check`
  在最终差异上执行。

这是治理规则修改，不涉及运行代码、镜像构建或容器启动。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: `main` 是稳定源码基线；普通源码 PR 不因目标分支自动触发 Docker 门禁。
- F2: Docker 由人工从选定的 `main` source SHA 单独封装；只有完成镜像身份和
  容器验证记录的产物才是部署候选。
- F3: 本裁决只移除分支级封装强制，不降低源码、接口、实时、安全、Mock、
  生命周期、硬件授权或正式发布门禁。

## 遗留

人工封装任务的具体时间、source SHA、镜像标签和发布版本由用户在源码稳定后指定。
现有 Docker 资产是否对应届时的 `main` 必须在封装任务中重新核对，不能从本裁决推断。
