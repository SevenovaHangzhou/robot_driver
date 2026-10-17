---
id: chassis-control-20261017-04
area: chassis-control
title: 底盘运行故障锁存、轮速归零后有序禁能
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 决定：控制器故障=轮速归零后有序禁能；CAN 编码器重启按控制器故障处理，恢复后手动重新使能
commits: [feature/electri-127-motion-fault]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261017-03]
---

## 背景

主线 swerve_controller 在反馈故障、舵角超差或限位时只在实时层停止输出，驱动器保持使能，
反馈恢复后收到新指令还会继续运动，没有人收尾。

## 改动

- swerve_controller：`feedback_fault`、`steering_hold_fault`、`steering_limit` 锁存为运动故障，
  直到下次激活；锁存期间不接受指令。全部车轮实测轮面速度 ≤ `fault_stop_speed`（0.01 m/s）
  持续 `fault_stop_hold`（50 ms）后，或轮速不可测、或超过 `fault_stop_timeout`（2 s），
  在 `~/fault_stop`（可靠、transient_local，20 ms 周期）发布 true。
- enable_manager：`motion_fault_topic` 订阅；仅在 Enabled 阶段受理。worker 走与 `/rt/disable`
  相同的顺序（停运动控制器 → 0x0007 → 0x0006 → 开关禁止），记录 `motion_fault` 并进入 FAILED。
  不走快停。重新使能只能由操作员发起（保留一次自动复位）。
- composer：使能参数 `motion_fault_topic=/<motion 控制器>/fault_stop`。
- CAN 编码器重启：上一变更作废样本 → 反馈无效 → 运动故障 → 本流程；恢复后由操作员重新使能。

## 验证

- `colcon test --packages-select enable_manager`：85 tests，0 failures（新增运动故障有序禁能用例）。
- `colcon test --packages-select swerve_driver`：164 tests，0 failures（新增“轮速未归零不请求、归零后请求、
  锁存期间不恢复”用例；编码器丢失用例改为“恢复后不动作，重新激活才可动作”）。
- composer：42 passed；quality_gate PASS。

离线测试，不授权使能或运动。

## 遗留

- 运行入口与监督进程（重启判断）见后续变更。
