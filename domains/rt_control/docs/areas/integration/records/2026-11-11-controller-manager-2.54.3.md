---
id: integration-20261111-01
area: integration
title: 核验 controller_manager 2.54.3 并修复 CI 版本准入
date: 2026-11-11
type: fix
trigger: 用户提供 PR 260 的失败构建 42984753273，任务 arch-flexible-startup
commits: [feature/arch-flexible-startup]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [outputs/module-loop/pr260-integration/cm-2.54.3/]
supersedes: []
related: [integration-20261022-01]
---

## 背景与改动

CI 安装了 controller_manager 2.54.3，旧版本白名单在 integ_bringup 配置阶段拒绝它。核对官方源码后仅增加 2.54.3；保留其他版本的拒绝，不修改控制循环、上游源码或补丁。

## 验证

- 官方 2.54.2（e822ad539633d1d286d1a85ab4e0390c79f218e7）、2.54.3（65d389596953cb4656c4216a3f0bc08a73230739）及仓库 ros2_control_node.cpp 逐字节一致，SHA-256 均为 ca6192e765b4ad433e9cd9053ca8209982f6367e301afd7f99620baa3c4eed92。
- 现有 overrun 补丁对两个版本均以零 fuzz 应用，补丁后源码一致。
- 使用本机实际安装的 controller_manager 2.54.3 独立 CMake 构建：通过；链接解析到 /opt/ros/humble/lib/libcontroller_manager.so。
- integ_bringup CTest：33 项通过，0 失败、0 跳过。质量门禁通过。

未验证：本次提交的完整 CI、实机部署与周期时序。本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前保留的上游主程序及补丁适用于已核对的 2.53.1、2.54.2、2.54.3；后续版本仍须重新核验。

## 遗留

完整 CI 以本提交推送后产生的新运行结果为准；实机验证另行授权。
