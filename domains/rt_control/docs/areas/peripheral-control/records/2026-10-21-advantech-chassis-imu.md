---
id: peripheral-control-20261021-01
area: peripheral-control
title: 研华底盘接入 LPMS-NAV3 IMU（/imu/data，内核接收时间戳）
date: 2026-10-21
type: feature
trigger: ELECTRI-127；用户要求底盘 IMU 接入并为轮式 + IMU 融合提供数据
commits: [feature/electri-127-chassis-imu]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - 研华 records/imu-calibration/20261020T213915-stage1-static-disabled/stage1-report.json
  - 研华 records/imu-calibration/底盘里程计融合测试报告-20261020.md
supersedes: []
related: [chassis-control-20261021-01]
---

## 背景

研华底盘的 LPMS-NAV3 IMU 挂在 `can0`（500 kbit/s，节点 1），与舵轮外置编码器共用总线。
导航需要 `/imu/data`（契约 R-OUT-22）用于轮式 + IMU 融合。

## 改动

- `lpms_nav3_can`：
  - TPDO 通道 7–9 改名为 `angular_velocity_bias_corrected_radps`；`/imu/data` 使用通道 4–6。
  - 原 `~/mag` 话题改为 `~/gyro_bias_corrected`（`sensor_msgs/Imu`，只填角速度）。独立磁场不公开。
  - 时间戳取第一帧 TPDO 的内核接收时间（`SO_TIMESTAMPNS`），时间来源为 `TIME_SOURCE_HOST_RECEIVE`。
    它不是设备采样时间。
  - 新增 `publication_stamped_samples` 计数和 `allow_shared_interface` 参数（只接收，不发送）。
- `rt_control_composer`：新增 `chassis_imu_sensor()`；部署 schema 增加 `chassis_imu`；
  产物生成 `sensor_chassis_imu.yaml` 并加入生命周期 `sensor_nodes`。
- `rt_control_bringup`：范围运行时启动 `sensor_nodes`；`exec_depend lpms_nav3_can`。
- `Alfa_V3-Advantech-01/deployment.yaml`：`chassis_imu {lpms_nav3_can, can0, node_id 1, imu_link,
  shared_bus_receive_only}`。
- 舵轮控制器保持 `imu_enabled: false`。IMU 只作为观测输出，融合由导航 EKF 负责。

## 验证

已验证（T1，本地 Native）：

- `lpms_nav3_can` 35 项测试通过；组合与集成相关 12 项测试通过。

已验证（T2，研华现场，底盘未使能静止 10 分钟）：

- `/imu/data` 100.002 Hz，最大间隔 10.93 ms。
- 4 组 TPDO 各 60,201 帧，格式错误和丢弃组包为 0。
- 消息时间戳与 CAN 内核接收时间 100% 在 2 µs 内一致。

已验证（T4，研华现场，操作人驾驶，见 2026-10-20 报告）：

- 安装偏航 +90.0° ± 0.5°；Z 角速度比例误差 0.01–0.03%；IMU 比轮式角速度晚约 8 ms。

未验证：

- `/imu/data` 的协方差字段当前为全 0，待标定后填写。
- 主线代码的研华重新部署（现场为同等内容的补丁构建）。

本记录不授权使能或运动。现场运动均由操作人在网页驾驶。
