---
id: chassis-control-20261017-01
area: chassis-control
title: 研华底盘台架已验证参数与行为修复迁入组合架构
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-16/17 要求把实机验证过的底盘参数迁入新架构并移植保持、转向符号与编码器回绕
commits: [feature/electri-127-chassis-real-facts]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [integration-20261014-02, integration-20261015-01]
---

## 背景

研华工控机底盘（四转向 CSP、四行走 CSV、四个外置 CANopen 编码器）已用现场配置实机运行。
新组合架构里这些事实仍为 `TBD`，舵角保持、转向符号和编码器回绕修复也只在现场。
本次把它们迁入机型、单机和协议 owner 的数据，不迁固定模式插件、一键脚本、录波和 Web。

## 改动

- 轮组键 `front_left..rear_right` 改为 `module_1..4`，编码器资源名改为 `steering_encoder_1..4`。
  原因：车头已重新定义，前后左右标签不再正确。mock 模型关节名改为真实 `steerN_joint/wheelN_joint`。
- 八轴：环位偶数为行走 `wheel1..4_joint`、奇数为转向 `steer1..4_joint`；`direction` 全部 `-1`；
  电机计数行走 10000、转向 65536；转向零点 46212/-3394086/-3430317/2967074，行走 0。
  换算系数与现场 profile 逐轴相等。
- 编码器：节点 11..14；方向 -1；安装偏置按现场；65536 计数、16 圈。
- 单机：模块坐标 x `[-0.305,0.305,-0.305,0.305]`、y `[0.305,0.305,-0.305,-0.305]` m，
  坐标系 `base_footprint`；有效轮径 0.1 m（名义值，50 mm 实测 50.003 mm）。
- Kinco 协议 profile：身份 6820200/4604997/505，DC 1 ms、`0x0300`，看门狗读回 100 ms，
  反馈超时 0.2 s，停稳阈值 行走 0.1 rad/s（0.01 m/s）/转向 0.01 rad/s，模式确认超时 30 s。
- 组件最大速度按角色分开：转向 35°/s，行走 20 rad/s（2 m/s）；行走位置范围 ±49000 rad 仅为 int32 可表示范围。
- 控制器策略：线速度与轮面速度 2 m/s、轮面加速度 2 m/s²（单位为 m/s 与 m/s²），
  转向速度 35°/s、转向加速度 35°/s²；其余阈值沿用现场值。协方差和 IMU 参数为占位值，待标定。
- 删除机型数据中未被使用的 `module_coordinates`，坐标只由单机标定提供。
- swerve_driver：保持最后下发目标、超差锁存 `steering_hold_fault`；转向梯形加减速，
  零命令和命令超时时先减速再锁存，行走轮按加速度斜坡停止；新增 `steering_to_yaw_sign`，
  由渲染器从机型 `model_heading_sign` 导出并绑定（模型转向关节绕 X 翻转 π，heading = -q）。
- robot_hw_canopen：解码器处理 16 圈计数回绕；渲染器传递 `encoder_revolutions`；
  迁入 BRT EDS 厂家原件与兼容文件（`6502` 改 UNSIGNED32，五个 DS406 对象移入可选）及测试。
- KincoCyclicModeSlave：禁能时 `6061=0` 允许初始化，启动模式随使能写入；回读确认前保持原位/零速，
  超过模式确认超时锁存错误。

## 验证

- `colcon test --packages-select swerve_driver`：163 tests，0 failures。
- `colcon test --packages-select robot_hw_canopen`：17 tests，0 failures。
- `test_kinco_cyclic_mode_slave`：9/10 通过；插件注册表用例失败，原因是本地底层安装的插件 XML 损坏，与本次无关。
- robot_hw_ethercat Python 测试：54 passed。
- composer `test_chassis/test_boundary_bindings/test_compose/test_head_force` 与 EDS 测试（源码 PYTHONPATH）：41 passed。
- 真实路径离线组合只剩 `verified` 与背隙缺项。

均为离线测试，不连接总线。本记录不授权使能或运动。

## 结论与冻结事实

- F1: 上述环位、方向、计数、零点、坐标、编码器参数来自研华部署 profile（base 946ec110）与用户确认。
- F2: 控制器速度/加速度参数是轮面 m/s 与 m/s²，不是关节 rad/s。

## 遗留

- `verified:false` 保留；背隙与 IMU/里程计方差待迁移完成后统一标定。
- 驱动器 PDO 需一次性改为主线插件布局并写入不易失内存，属实机写操作，另行授权执行。
- Robot Model 仍锁定 `17f5bdc`；部署模型切换与夹爪入口取舍另立变更。
- 底盘使能链路、Web 与一键启动未迁移。
