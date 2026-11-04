---
id: governance-20260919-01
area: governance
title: V3 Robot Model CI 使用构建 overlay
date: 2026-09-19
type: fix
trigger: PR #39 完整构建通过后，Robot Model xacro 校验找不到刚构建的 robot_description 包
commits: [feature/v3-arm-motion-runtime]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [https://github.com/SevenovaHangzhou/robot_driver/actions/runs/35373311636]
supersedes: []
related: [release-deploy-20260919-01]
---

## 背景

CI 已完成完整构建和测试，但 Xacro 使用 `$(find robot_description)` 时只看到系统 ROS，
没有看到本 job 的安装结果。

## 改动

Robot Model 校验在 source `/opt/ros/humble` 后继续 source `install/setup.bash`；新增顺序回归测试。

## 验证

本地质量门禁和定向测试通过；PR #39 CI 重跑用于验证完整 overlay。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: CI 校验共享 Robot Model 前必须 source 当前构建的安装 overlay。
- F2: `xacro` 和 `check_urdf` 校验的是本 PR 构建产物，不是仅依赖系统 ROS 环境。

## 遗留

无功能代码风险；等待 PR #39 完整 CI 重跑终态。
