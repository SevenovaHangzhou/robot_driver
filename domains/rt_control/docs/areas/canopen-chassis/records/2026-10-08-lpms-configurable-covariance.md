---
id: canopen-chassis-20261008-01
area: canopen-chassis
title: LPMS 姿态与角速度协方差支持 YAML 配置
date: 2026-10-08
type: feature
trigger: 用户要求将 orientation_covariance 和 angular_velocity_covariance 加入配置文件，默认零
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /tmp/lpms-covariance-build.log
  - /tmp/lpms-covariance-rebuild.log
  - /tmp/lpms-covariance-review/build/test_results/lpms_nav3_can
  - /tmp/lpms-covariance-quality.log
supersedes: []
related: [ELECTRI-105, BQ-150]
---

## 背景

用户要求通过配置文件填写 IMU 姿态和角速度协方差，未标定时默认保持零。

## 改动

新增两个启动时只读 double 数组参数，每个恰好九个有限数，按 3×3 矩阵行序写入 Imu 消息；默认九个 0.0。保留原单参数消息转换函数，增加协方差重载。更新包 YAML、README 和当前 /tmp 台架 YAML。

## 验证

隔离 CMake build/install（/tmp/lpms-covariance-review）通过，ctest 五个目标通过（20 GTest + 9 pytest）。覆盖默认零、非零矩阵复制、只读参数以及长度/NaN/Inf 拒绝。COVERAGE_FILE=/tmp/lpms-covariance-review/quality.coverage bash tools/quality_gate.sh 通过（207 passed、13 skipped，策略覆盖 83%）。git diff --check 通过。本轮未重启实机节点或进行协方差标定。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: orientation_covariance / angular_velocity_covariance 支持 YAML 启动配置，默认全零表示未知；修改后需使用新构建重启节点。
- F2: 参数按行排列，每个九个有限数；单位分别为 rad² / (rad/s)²。不从 CAN 报文读取，也不自行估算或替用户标定。

## 遗留

厂家/标定协方差和现场应用验收未完成；未标定时继续保留零。原始 CAN 映射、共线准入及坐标定义仍受 BQ-150 约束。
