---
id: ecat-axes-20260916-01
area: ecat-axes
title: Kinco 四舵轮 CSP/CSV 接口合同与八轴 draft
date: 2026-09-16
type: feature
trigger: ELECTRI-127
commits: [work/electri-117-swerve-module-binding]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-117, ELECTRI-127, BQ-144, BQ-145, motion-20260915-01]
---

## 背景

main 已有协议无关的 `swerve_driver/SwerveController`，但
`robot_hw_ethercat` 只登记旧机和双臂 family/profile，没有四转向 CSP、
四驱动 CSV 所需的 ros2_control 合同。实际 Kinco ESI identity、环位和 PDO/SDO
尚未确认，本轮只建立不触发运行的合同和 draft。

## 改动

- 在 family registry 增加未被任何 production family 引用的
  `swerve_steering_csp` 和 `swerve_drive_csv` interface contracts。
- 转向合同导出 `position/control_word` 命令以及
  `position/status_word/mode_of_operation_display` 状态。
- 驱动合同导出 `velocity/control_word` 命令以及
  `position/velocity/status_word/mode_of_operation_display` 状态。
  `control_word/status_word` 保持 enable_manager 所有权；swerve controller
  只 claim position/velocity 并读取状态。
- 新增 `alfa_v3_swerve_chassis.draft.yaml`：八轴按四转向 FL/FR/RL/RR、
  四驱动 FL/FR/RL/RR 排列，模式固定 8/9，单位固定为输出轴 rad 和轮侧
  rad/rad/s，共享 `ecat_arms` master 0 和 1 kHz 请求周期。
- identity、joint name、ring position、family、profile、DC 周期支持和 PDO watchdog
  全部为 `TBD`。validator 拒绝局部猜值；draft 放在 config/machines，
  不进入 `variants`，因此不会生成 production EtherCAT system。
- alfa_v3 machine manifest 引用该 `verified: false` profile，runtime gate 继续关闭。

## 验证

- TDD RED：两个合同不存在、draft 文件不存在时 3 个定向测试失败；实现后通过。
- EtherCAT variant + machine-profile 完整配置回归：97 passed。
- CMake `validate_ethercat_variants` 和 `validate_kinco_swerve_draft`
  两个目标通过。
- `robot_hw_ethercat BUILD_TESTING=ON` 包构建尝试被本机缺少
  `gripper_controllers` 阻塞；`BUILD_TESTING=OFF` 又发现现有安装 underlay
  未含当前源码需要的 ICube 0013 PDO cycle hook ABI。未通过删除 override 或混用
  旧 ABI 绕过，因此不把包级二进制构建写成通过。
- CANopen/EtherCAT/machine/governance 合并定向回归：155 passed；
  发布用例目录 33 cases valid。
- `tools/quality_gate.sh`：282 passed、13 skipped，策略覆盖率 83%；
  本机缺少 ShellCheck，保留 CI 检查。
- `git diff --check`、两个 draft validator、0006 `git apply --check`
  和 native bootstrap shell 语法均通过。

这是配置和合同验证，没有创建实际 slave profile、启动 IgH、读写 SDO、复位或使能。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: Kinco 舵轮继续使用现有 generic EcCiA402Drive 和整机唯一 EtherCAT master，
  不引入独立 Kinco/IgH 控制循环。
- F2: 四转向为 CSP mode 8、四驱动为 CSV mode 9；命令/状态接口对象已冻结，
  control_word 仍由 enable_manager 管理。
- F3: 控制器看到的转向单位是输出轴 rad，驱动位置/速度是轮侧 rad/rad/s；
  原始计数、电子齿轮、减速比、方向和零偏必须在实际 slave profile 确认换算。
- F4: 未确认 identity、环位、PDO/SDO、DC/watchdog 和机械标定前，
  不创建 production family/profile/variant，runtime-ready 必须拒绝。

## 遗留

需要厂家 ESI、Vendor/Product/Revision、实际 PDO/SDO、0x6064/0x606C 单位、
电子齿轮、转向/驱动方向、chassis_only/full_robot 环位、DC 支持、PDO watchdog
和 0x605E 停止行为。资料确认后才能创建 Kinco family 与 CSP/CSV slave profiles，
再处理 enable_manager 多运动控制器和 chassis_only 联合 Mock。
