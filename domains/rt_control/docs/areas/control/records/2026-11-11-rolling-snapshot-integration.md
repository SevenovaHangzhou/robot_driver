---
id: control-20261111-01
area: control
title: Rolling 完整状态摘要接入保持指令查询与拒绝诊断
date: 2026-11-11
type: fix
trigger: 用户批准 PR 260 两版本取舍，任务 arch-flexible-startup
commits: [feature/arch-flexible-startup]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [outputs/module-loop/pr260-integration/]
supersedes: []
related: [control-20261109-01]
---

## 背景

PR #260 的保持指令查询、拒绝诊断与主线完整 RT 状态摘要修改重叠。按用户确认的取舍合入 main@358ea0e，保留主线快照交换与准入保护，再接入功能分支的非实时服务和诊断。

## 改动

- 补窗、公开状态与保持指令查询共用三槽完整摘要；移除旧版本号重读及等待逻辑，不增加实时分配或锁。
- 保持指令查询只接受当前 epoch、未过期、状态与当前 RT 一致、零速度且硬件反馈新鲜的摘要；返回保持命令位置。
- 补窗保留主线的最新执行时间、替换边界、已消费 generation 与停车时间增长余量；过期拒绝原因使用 `snapshot_stale`，网页同步识别。
- 保留主线七项并发/生命周期回归与功能分支保持指令、诊断及连续 1 kHz 更新覆盖，新增旧保持摘要及过期查询回归。
- 用 clang-format 14、ruff 0.11.13 修正本分支相对主线的格式；25 个 Python 文件的纯格式部分通过 AST 一致性核对。

## 验证

本地 ROS Humble 隔离构建目录 `work/pr260-validation`，使用既有依赖与固定公共接口 SHA 78f02ffe5b8f40a9f57bf9405cd00e007914fc84 的 Rolling 客户端。controller_manager 2.54.2 使用既有独立安装；未改系统安装或生产目录。

- CMake 构建 Rolling、网页、组合器、启动包、头部驱动、SocketCAN 与力传感广播器及 Rolling 客户端：通过。
- CTest：Rolling 70、网页 382、启动包 33、组合器 253、头部驱动 30、力传感广播器 15、客户端 15，共 798 项通过，0 失败、0 跳过。
- 网页传输测试使用 `ROS_LOCALHOST_ONLY=1`、隔离 ROS 域 219，并加载本次编译的扩展；未接入硬件。
- `tools/quality_gate.sh`（提供固定格式工具及 jsonschema 3.2）：通过；`git diff --check` 与暂存区检查通过。
- 初次运行遇到本地依赖路径、旧动态库优先级、未安装插件和回环网络限制；补齐隔离环境后重新运行上述全部相关测试。没有修改生产逻辑来绕过失败。

未验证：合并结果的实机部署、总线时序、使能与运动、完整容器/全仓构建。前置柔性启动的实机与上游补丁证据仍按原记录边界解读；本次结果不替代生产验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 保持指令查询与补窗消费同一套完整 RT 摘要，不使用独立版本号重试机制。
- F2: 过期摘要不能作为补窗或保持指令查询的依据；补窗拒绝诊断可报告 `snapshot_stale`。

## 遗留

实机与完整容器验证待后续授权；保持已有模块及故障联动策略。
