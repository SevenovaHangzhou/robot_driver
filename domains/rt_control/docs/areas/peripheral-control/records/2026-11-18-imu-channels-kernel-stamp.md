---
id: peripheral-control-20261118-01
area: peripheral-control
title: LPMS IMU 接入 chassis_only 生产启动，通道 7–9 改为零偏校准陀螺，/rt/imu/data 用内核接收时间戳
date: 2026-11-18
type: fix
trigger: ELECTRI-227；用户 2026-11-18：IMU 配置按 feature/electri-127-chassis-imu，IMU 无磁力计；导航逐轮融合口径 v0.1 §7
commits: [feature/wheel-observations]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - 研华 ~/Robot/rt-control/tasks/ELECTRI-227-wheel-observations/runs/imu-20261118T072530Z/（只读运行，summary.txt）
  - 研华 ~/Robot/rt-control/records/imu-calibration/rotation-fullturns-20261020T224112/can.log（只读离线分析）
  - 研华 ~/Robot/rt-control/records/imu-sdo-targeted-20261017T075351Z/diagnosis.json
supersedes: []
related: [peripheral-control-20261021-01]
---

## 背景

main 把 0x280/0x380 的通道 7–9 当磁场解码并发 `~/mag`，时间戳是发布时刻。未合并分支
`feature/electri-127-chassis-imu`（peripheral-control-20261021-01）记录的现场映射是通道 7–9 为
静止零偏校准陀螺、没有磁力计，并改用内核接收时间戳。

## 改动

- `periph_imu_lpms`：通道 7–9 解码为 `angular_velocity_bias_corrected_radps`（0.1 °/s/LSB），
  发 `~/gyro_bias_corrected`（只含角速度，姿态与加速度协方差[0] = −1）；删除 `~/mag`。
- `/rt/imu/data` 时间戳取样本首帧（0x180）的 SocketCAN 内核接收时间（`SO_TIMESTAMPNS`，CLOCK_REALTIME），
  `/rt/sensors/state` 标 `TIME_SOURCE_HOST_RECEIVE`；无内核时间时退回发布时刻，标 `PUBLICATION` 并计数
  `publication_stamped_samples`。不做安装旋转，`frame_id` 仍是传感器自身坐标。
- `infra_socketcan`：新增 `enable_receive_timestamps()` 与带内核时间的 `receive()`。
- 生产启动：单机部署新增 `optional_peripherals`（`units/Alfa_V3-Advantech-01/deployment.yaml`），
  参数文件 `lpms_nav3.yaml`（can3、节点 1、`imu_link`、研华现场角速度协方差）。composer 只在实机组合里把它写入
  `lifecycle.yaml` 与 `peripherals/lpms_nav3.yaml`，scope launch 按此启动节点；节点退出或设备离线不影响底盘启动与就绪。
  `integ_bringup` 增加 `periph_imu_lpms` 运行依赖。

## 验证

- 离线（研华原始 CAN 日志，未访问总线）：2026-10-20 原地整圈记录 36,188 个样本，通道 7–9 与 4–6 相关
  0.95/0.97/1.00，Z 积分 −389.5° 对 −390.1°；静止时通道 7–9 约为 0。确认为陀螺而非磁场。
  通道 1–3 静止 Z ≈ +1.003 g，保留重力。
- 设备回读：NAV3-CAN 口只发数据，10-17 的 SDO 回读无应答，厂家上位机手册（V3-DOC-2026-022）要求 TTL；
  映射与滤波模式本次未能回读。
- T0 本机：`periph_imu_lpms` 32、`infra_socketcan` 9 项测试通过（无 vcan，带时间戳的收发用例跳过）；
  `integ_composer` 新增实机/mock 外设组合用例、`integ_bringup` 新增 launch 用例通过。
- T2 研华只读（2026-11-18，用户授权，底盘未使能）：本分支构建的 IMU 节点单独在 can3 只收运行 20 s，
  隔离 ROS 域（231，仅本机）。1,929 个样本，100.003 Hz，间隔 9.63–10.36 ms（P99 10.20 ms），无重复或倒退时间戳；
  消息时间戳与 candump 记录的 0x181 内核接收时间相差中位 0.5 µs、最大 1.2 µs；`/rt/sensors/state` 时间来源 2
  （HOST_RECEIVE），`publication_stamped_samples` 为 0；静止加速度 Z ≈ +9.84 m/s²。
- 未验证：整栈启动中 IMU 节点与 CANopen 主站共存（同日整栈启动被 EtherCAT 自检拒绝，见 control-20261118-01）。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 研华 LPMS-NAV3 的 CAN 输出没有磁力计通道；通道 7–9 是静止零偏校准陀螺（离线 CAN 证据）。

## 遗留

- 现场接 TTL 后只读回读 CAN 映射与滤波模式。
- `ARCHITECTURE.md` §2.1 对照表没有"台套启动外设节点"一行，需按 `feature/arch-*` 流程补表。
- `imu_link` 在 Robot Model 中偏航为 0，与实测安装 +90° 不一致，需 robot_description 修正或由导航外参覆盖。
- 研华 ELECTRI-181 报告的安装偏航 +90.0° ± 0.5°、IMU 比轮式晚约 8 ms、X/Y 角速度方差约 1.1e-6/1.0e-6 (rad/s)²
  属标定制品，交导航使用；Z 方差静止不可测。
