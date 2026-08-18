---
id: governance-20260818-01
area: governance
title: 驱动变体三层一致性门禁（ELECTRI-94）
date: 2026-08-18
type: feature
trigger: ELECTRI-94
commits: [feature/driver-variant-config]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: ["quality_gate: 260 tests PASS", "five-module hard coverage: 87%", "Docker build: 28 packages PASS; tag rt-control:electri-94-final", "isolated no-device Mock startup/shutdown: PASS"]
supersedes: ["governance-20260813-05#F2"]
related: [motion-20260818-01, lifecycle-20260818-01, contract-20260818-01, canopen-chassis-20260818-01]
---

## 背景

轴拓扑、profile、换算、控制器、诊断和安全生命周期存在多份事实副本；单份 schema
不能阻止运行时资产继续漂移，且过去 scoped tests 对普通包配置只跑包测试。

## 改动

新增严格 manifest parser、结构化 YAML/Xacro/profile 投影器与 AST/C++ marker/patch
源码投影器；重复 YAML key、未知字段、路径越界和不完整投影均聚合失败。
`quality_gate.sh` 在 repository gate 后无条件运行统一检查，repository policy 固定两个
命令各一次及顺序。scoped resolver 将 manifest 判全量，将投影消费者判 quality gate
加所属包测试；三个新模块进入合并 branch coverage 80% 硬门禁。CI 新增由 governance
前置的生产镜像构建与无设备 Mock 启停 job，并由 repository policy 防止删除或降级。

## 验证

260 个 tools pytest 全过；五个硬门禁模块合并 branch coverage 87%，三个新模块分别
90%/86%/88%；`tools/quality_gate.sh` 全 PASS。mutation 覆盖 schema、映射、换算、
Robot Model active joint/类型与 SI 单位、Xacro namespace/参数穿透、controller safety
参数、AST 常量、C++ marker 与 patch 消费。
完整生产 Dockerfile 构建 28 个包通过；无网络、无设备、无额外 capability 的容器检查
确认入口、启动器、相关包及安装空间 manifest，并完成 Mock ROS 启动、预期 controller
状态、`/rt/disable` 优先的有序停机与 exit 0。未执行需要获批测试网络和生产资源参数的
完整 L1 Mock 或任何实机动作。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: quality gate 必须先运行 repository gate，再运行统一 driver-variant 三层投影检查；两个命令都必须恰好一次。
- F2: 覆盖率硬门禁集扩展为 repository gate、PR contract gate 与三个 driver-variant 模块，合并 branch coverage 不低于 80%。
- F3: manifest 变更触发全量验证；任一投影消费者变更至少触发 quality gate 与所属包测试。

## 遗留

phase 1 校验现有投影而不自动重写；未来若引入 generator，必须保持确定性、reviewable
输出并继续 fail-closed。ShellCheck 本机不可用；新增的强制 ShellCheck、生产镜像构建
及 Mock 启停 CI job 尚待本 Draft PR 首次运行。完整 L1 Mock 与实机 commissioning
仍须按各自授权和风险门禁执行。
