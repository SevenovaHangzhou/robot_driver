---
id: contract-20260828-01
area: contract
title: ROS Domain 由部署显式配置
date: 2026-08-28
type: decision
trigger: 用户正式裁决取代 BQ-128 的固定 Domain 0 策略
commits: []
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - "native launcher focused tests: PASS (51 tests)"
  - "Compose/IO focused tests: PASS (18 tests)"
  - "tools/quality_gate.sh: PASS (197 tests, gated coverage 83%)"
  - "candidate image build: PASS (28 packages)"
  - "image id sha256:af24d7e5f1a5b32846b6e60e0992bcf6812a83438d194ce8cfe906e0e9f63ea4"
  - "Domain 12 hardware-free Docker Mock: PASS (devices=[], exit=0)"
supersedes: [BQ-128-fixed-domain-zero]
related: [BQ-128, BQ-141]
---

## 背景

BQ-128 为当时同机联调冻结 Domain 0，并固定 Fast DDS 的 RMW 和传输策略。现场现需按
机器实例与联调环境选择 ROS Domain，用户因此正式裁决取代“必须为 0”的部分。

## 改动

- Native 支持 `--ros-domain-id N`，优先级为 CLI、`RT_CONTROL_ROS_DOMAIN_ID`、`ROS_DOMAIN_ID`、`0`。
- Docker Compose 使用 `RT_CONTROL_ROS_DOMAIN_ID`，未设置时默认 `0`。
- 两条路径都在硬件访问前拒绝非十进制值和 `0..232` 之外的值。
- `ROS_LOCALHOST_ONLY=0`、`rmw_fastrtps_cpp`、Fast DDS 默认 UDP+SHM 与无 DDS XML 挂载保持不变。

## 验证

执行 native/Compose 参数的 RED/GREEN 测试，覆盖默认 `0`、显式 Domain 以及负数、前导零、
超过 `232` 和非数字输入。Native 51 个聚焦测试、Compose/IO 18 个聚焦测试、完整仓库门禁
197 个测试与 83% 门禁覆盖率通过。main 候选镜像完成 28 包构建；Domain 12 的无硬件
Docker Mock 无设备映射、控制器状态正确并有序退出 `0`。
Mock 不是真实总线或多域联调证明。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: ROS Domain 为部署输入，合法范围 `0..232`，默认 `0`。
- F2: 同一机器实例的 RT-Control、Motion、Navigation、Perception 和 Autonomy 必须使用同一 Domain；
  Domain 不构成安全隔离。
- F3: RMW 继续固定为 `rmw_fastrtps_cpp`，传输继续使用 Fast DDS 默认 UDP+SHM。

## 遗留

每个四域/五域联调发布清单必须记录实际 Domain，并完成 topic/service/action 双向发现 smoke。
现有生产 release 在重建和受控切换前仍保持其已部署的 Domain。
