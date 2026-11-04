---
id: release-deploy-20260909-01
area: release-deploy
title: swerve_driver 构建依赖与增量容器验证
date: 2026-09-09
type: feature
trigger: ELECTRI-118
commits: []
env: both
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [motion-20260909-01, BQ-145]
---

## 背景

新舵轮包需要进入现有构建闭包，同时不能在硬件资料未齐时被生产 launch 自动加载。

## 改动

bringup 增加 swerve_driver 运行依赖，仓库治理登记新域包；安装 plugin XML、头文件、库、来源说明与
明确不可运行的 draft 参数模板。原有 launch、设备权限、CPU 设置和主站选择不变。

## 验证

- `colcon list --packages-up-to rt_control_bringup` 确认构建闭包包含 swerve_driver。
- 以本地既有 PP 验证镜像（digest `cfddfcb50391d8f1903242c146793bff9bcfde002ad0274694a5ad5a34b56443`）
  为基线，用 `docker build --network none` 编译新包并执行测试，64 条测试记录全通过。
- 新验证镜像 `rt-control:electri-118-swerve-offline` 的 digest 为
  `bc6cc03c2177087537437280566c7f831c56a22dfb7c449531da90f0be5a2cec`。
- `docker run --network none --cap-drop ALL` 运行共享 manager Mock 测试通过，无 EtherCAT/CAN 设备映射。
- 此镜像使用测试入口，仅用于软件验证；不是完整生产 Dockerfile 从零重建或 rt_control_start 交付验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 安装/构建新包不等于启动新控制器，生产默认仍走旧机配置。
- F2: 真实后端、模型/标定及控制范围组合未闭合前，alfa_v3 继续拒绝真实启动。

## 遗留

完整生产镜像、现场时序、真实后端与启动组合见 BQ-145；不授权以验证镜像替换生产服务。
