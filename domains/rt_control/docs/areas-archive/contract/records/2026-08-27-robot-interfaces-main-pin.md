---
id: contract-20260827-01
area: contract
title: robot_interfaces 权威 SHA 同步至 92d6ff2
date: 2026-08-27
type: corrective
trigger: 用户要求目标工控机 RT-Control 更新到最新公共 interface
commits: []
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - robot_interfaces main 92d6ff2ed0b45684d7da2170d96703ca8be569f4
  - focused pytest 66 passed
  - quality_gate 193 passed, gated coverage 83%
  - "rt-control:ipc-main-sync-20260828 image build: PASS (28 packages)"
  - "image id sha256:d26242edc6f3d5dce1ea08e3c9d99f4d10ca035c10a6567baaa34c7b01bed8ab"
related: [BQ-137]
---

## 范围

目标机 `/home/ar/rt-control-dev/robot` 的 `deps.repos` 与
`src/interfaces/source-lock.yaml` 从
`f699f45972ad15bbbbbb3da1a4894faf209144c9` 更新到权威 `robot_interfaces/main`
`92d6ff2ed0b45684d7da2170d96703ca8be569f4`，并将干净 vendor checkout 切到同一 SHA。

## 兼容性审查

上游两个 SHA 之间只修改 `robot_motion_interfaces/action/ExecuteMotionStage.action`
及 Motion/Autonomy 契约视图，新增独立 TURN 与 NAMED_JOINT_POSE 阶段。RT-Control
直接构建的 `robot_rt_control_interfaces`、`robot_system_interfaces` 和
`robot_interfaces_qos` 三个目录 tree object 均未改变。因此本次没有 RT wire schema
变化，但统一权威 SHA 仍要求 Motion 与 Autonomy 同批升级，旧/新 M-08 Action 不得混跑。

## 构建与验证

- 重建三项 RT 公共包及 `bms_node`、`control_api_adapter`、`enable_manager`、
  `rt_diagnostics`、`rt_control_bringup` 依赖闭包；colcon 17 packages PASS。
- `bootstrap_native_dev.sh doctor` PASS，manifest/source-lock/vendor 三方 SHA 完全一致，
  vendor 工作树干净。
- QoS 五个命名 profile 运行 smoke PASS。
- focused pytest 66 passed；完整 `tools/quality_gate.sh` 193 passed，门禁覆盖率 83%。
- 最新 `main` 基线上的完整镜像构建 PASS，28 个包完成；构建日志确认 vendor
  `robot_interfaces` 精确为 `92d6ff2`。同一候选镜像的无设备 Domain 142 Mock 启停和有序退出 PASS。

## 边界

本次只启动无设备映射的 RT-Control Mock；未启动真实硬件栈、未访问或写入 EtherCAT/CAN、
未执行 reset/enable/运动/PLC 输出。
ShellCheck 在目标机不可用，仍由 CI 强制执行。目标机工作树已有其他未提交任务，本次没有
清理、覆盖或提交它们。

## 冻结事实

- F1: RT-Control 当前权威 `robot_interfaces` SHA 为 `92d6ff2`。
- F2: `f699f45→92d6ff2` 不改变 RT 三个公共包；变化只影响 Motion/Autonomy M-08。
