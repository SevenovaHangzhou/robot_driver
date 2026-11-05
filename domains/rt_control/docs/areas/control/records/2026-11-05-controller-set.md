---
id: control-20261105-02
area: control
title: 统一控制器启停集合，使头部与机械臂模式切换兼容
date: 2026-11-05
type: behavior
trigger: 用户选择 B2，并确认 motion_controller、rolling_controller、companion_controllers 三个配置字段
commits: [feature/arch-controller-set]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261105-01]
---

## 改动

- 使能控制器的配置收敛为 `motion_controller`、`rolling_controller`、`companion_controllers`。默认运动控制器为空且无伴随控制器时只使能硬件；存在 rolling 控制器时开放模式切换。`enable_only`、`motion_mode_switching`、重复的控制器名列表从参数与策略文件删除。
- 使能、失能、故障停机和模式切换共用控制器集合及请求构造。使能激活默认控制器与伴随控制器；模式切换只改变互斥的机械臂控制器，已激活的头部保持运行；失能停止集合中所有 active 控制器。
- 所有启停都读取 controller_manager 当前状态、发送一次 STRICT 请求、回读确认。空请求也验证目标状态。伴随控制器失活或缺失时不能报告模式切换成功；不确定状态仍按既有流程失能并要求重启。
- `bench_dual_arm_head` 增加 `dual_arm_rolling`，保留 JTC-only 配置选择；readiness 沿用 lifecycle 的互斥机械臂控制器对与共同启停的头部要求。
- 参数迁移需重新 compose artifact；不接受继续沿用旧参数名作为运行配置。

## 验证

本机隔离构建：`colcon build --packages-up-to ctrl_enable_manager integ_composer integ_diagnostics` 通过。
`colcon test --packages-select ctrl_enable_manager integ_composer integ_diagnostics` 及 `colcon test-result --verbose`：358 项，无错误、失败或跳过。新增 arms+head 的 Rolling 与 JTC-only compose 用例另跑 2 项通过。
用例覆盖头部随使能启动、两方向模式切换不重启头部、失能停止剩余头部及 Rolling、缺失伴随控制器拒绝成功、原状态机故障与超时路径。
`tools/run_scoped_tests.sh --base HEAD`：6 个受影响及下游包通过；累计 `colcon test-result` 658 项，无错误、失败或跳过。
`tools/quality_gate.sh` 通过（工具测试 191 项通过、11 项按环境跳过）。
先前 `test_config_contract` 的 CLI 失败是 schema 环境变量指向源码目录；按安装目录运行已通过，未修改该测试。

本记录不授权使能或运动。

## 遗留

尚未在实机执行新集合的使能、切换、故障停机和正常失能。当前台架 TF 部署未替换为本分支。
