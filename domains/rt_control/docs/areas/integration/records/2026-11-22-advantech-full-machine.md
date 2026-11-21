---
id: integration-20261122-01
area: integration
title: 研华整机组合 advantech_full：底盘、升降、双臂、腕部力、编码器与头部同一 controller_manager
date: 2026-11-22
type: feature
trigger: 用户 2026-11-22 要求研华只跑一个整机制品，导航驱动底盘的同时运控控制双臂与升降（ELECTRI-239）
commits: [feature/advantech-full-machine]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [control-20261122-01, control-20261122-02, protocol-20261122-01, protocol-20261122-02, hw-config-20261122-03]
---

## 背景

研华 EtherCAT 实扫 27 个从站：0 主分支器、1 升降、2–9 底盘、10 副分支器、11–17 左臂、18 左腕力、19–25 右臂、26 右腕力；外置编码器与 IMU 在 can3，头部在 can2。原有组合都只覆盖其中一部分，且底盘环位仍是单独接线时的 0–7。

## 改动

- 新 profile/scope `advantech_full`：一个 `ecat_arms` 主站系统（底盘、升降、双臂、腕部力）加 CANopen 编码器与达妙头部；分阶段激活顺序编码器 → 头部 → EtherCAT。
- 运动组：机械臂组 `arms_jtc`（15 轴：双臂 + 升降，整组下发）↔ `arms_rolling`（14 轴）；底盘组 `swerve_controller` ↔ `base_wheel_jtc`；头部为伴随控制器；腕部力与逐轮观测广播器常驻。
- 使能：底盘转向、行走（两两一批）→ 升降 → 双臂逐关节 → 头部握手；双臂不确认启动模式（无 0x6061），底盘与升降确认；软件急停响应关闭（用户 2026-11-22）。
- 新模块 `advantech_dual_arm`（台架双臂环位 +10，分支挂在副分支器 10 的端口 3/1）与单机零点（沿用台架零点）；`swerve_chassis` 环位 2–9；`advantech_arms_motion` 策略（升降起点容差 5 mm、静止 2 mm/s、接管 2 mm 为待实机确认的初值）；Rolling 沿用台架双臂运动策略；`arm_acceleration_limits` 增加升降 0.4 m/s²。
- hw_ethercat：分支可声明所属分支器 `esc_ring_position`，预检读该分支器的端口；Kinco 导出 `joint_names`。组合器：scope 绑定可对一个参数列多个来源（`/joint_states` 依次追加底盘与头部关节）。
- 单机部署：IMU 外设也在 `advantech_full` 启动。

## 验证

- 新增 `test_advantech_full.py`：mock 与真实组合下两组参数、使能批次与模式确认、15/14 轴控制器与限位、`/joint_states` 25 个关节、单一 EtherCAT 系统覆盖 0–26、分支拓扑、唯一主站健康组、激活顺序。组合器、诊断、bringup、hw_ethercat、tools 测试除 1 项环境相关外通过。
- 未验证：mock 启动（T1）、研华不使能只读（T2）、使能与运动（T3/T4）。

## 结论与冻结事实

- F1: 研华整机 EtherCAT 拓扑 27 从站如上；双臂分支在副分支器（环位 10）端口 3 → 11、端口 1 → 19，副分支器在主分支器端口 2（2026-11-22 只读实扫）。
- F2: 研华双臂沿用台架零点（用户 2026-11-22）。

## 遗留

- 旧 scope 与台架配置删除（ELECTRI-240）；网页机械臂页改发 15 轴（ELECTRI-237）。
- 升降三项容差为初值，需实机确认。
