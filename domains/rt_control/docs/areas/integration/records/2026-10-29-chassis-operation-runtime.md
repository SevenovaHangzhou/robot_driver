---
id: integration-20261029-03
area: integration
title: 复用导航标定接通作业模式运行配置
date: 2026-10-29
type: feature
trigger: 用户要求复用导航参数和相关代码使作业实机模式具备运行条件，未知运动限制允许估计
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: ["6 affected packages built", "773 colcon test results passed", "247 quality tests passed, 13 skipped", "production artifact configured without devices", "isolated ROS NAV-OP-NAV smoke passed"]
supersedes: []
related: [integration-20261029-02, BQ-145]
---

## 背景

验收项 integration-20261029-03：配置生成同时接通控制器相对运动、IMU实际话题和四驱动
CSV/CSP切换；复用导航已验证的硬件身份、PDO、逐轮几何与单位换算，运动限制估计有来源；
编译及真实控制器+cyclic adapter无设备联合测试通过，生成带校验和的候选部署。
范围为RT-Control域内composer、硬件renderer、swerve控制器及操作站，公共接口不变。
硬件重启、使能和实机模式切换另行确认；先完成可审查代码和候选产物。

## 改动

1. 单机部署显式选择 `chassis_operation` 模板；composer 从已验证导航配置推导四轮
   rad/s、rad/s²上限、转向限制、反馈时效和停车时限。半径不同也分别换算；公共停车速度
   取最大轮径对应的更严格轮侧值。未验证标定、缺少 IMU 或缺少交接接口拒绝生成。
2. 硬件 renderer 只为四个 drive 配置 `ec_module.mode_switching=true`；启动仍 CSV=9，
   四个 steering 继续 CSP=8。PDO、身份、方向、减速比、计数换算、编码器偏移未改变。
3. 控制器新增可配置 `imu_topic`，连接现有 `/imu/data`、`imu_link`；NAV 状态发布实际 IMU
   有效性和停车确认，网页/RT 两级拒绝无有效 IMU 的作业准入。导航速度不因该准入失效。
4. 修复 controller-manager 自动声明 YAML 后 relative 参数重复声明的初始化异常；
   保留原 YAML 覆盖值。作业切换只在 session 无故障且四轮交接确认时成功。
5. 模板初始作业速度 0.05 m/s、转动 0.1 rad/s；加速度、容差及部分超时为用户允许的
   调试估计，注释列出单位与来源。`profile_verified` 表示复用已有 NAV 硬件/标定配置，
   不代表这些估计已通过动态验收；CSP 状态位12、实际IMU轴向/时效和低速运动仍须现场验证。
6. 保留 enable_manager 控制字所有权、真实反馈确认、超时故障及旧速度失效机制。
   已重新生成正式 `generated/chassis_only`，下次 `chassis start` 使用新配置；原产物备份
   `v3/deployments/chassis-before-operation-20261029T162936`。未启停总线、使能或发送实机模式请求。

## 验证

- 六包构建：swerve_driver、robot_hw_ethercat、rt_control_bringup、rt_control_composer、
  rt_control_machine_config、rt_control_operator_web 通过。
- `colcon test-result`：swerve_driver 187、robot_hw_ethercat 148、bringup 45、composer 225、
  operator_web 168，均零错误/失败（colcon 汇总包含 CTest 条目）。新增 YAML 参数覆盖、IMU
  丢失拒绝、非等轮径换算、超时不可行拒绝及 opt-out 测试。机器配置包无测试，不以此作证据。
- 真实 SwerveController + Kinco adapter 的8个无设备 synthetic-PDO 联合场景通过，覆盖
  NAV停车/预装/四轮确认/相对保持/CSV返回/新指令、部分模式拒绝和缺发送ACK等失败路径。
  增量构建曾遗留旧 ABI 测试程序，bringup clean rebuild 后重新测试通过。
- 隔离本机 DDS domain203：真实 ROS 模式服务 NAV→OP→NAV、零位移 Action 和网页后端
  回读联调通过；不连接总线，不发送实机运动。
- 正式产物 checksum/重新组合校验通过；生产控制器离线 init/configure 接受该产物，
  声明24命令/73状态接口且不含控制字 writer。候选与原部署归一化路径比较只改
  controllers.yaml 和 robot_description.xml，8份 slave 配置均相同。
- `tools/quality_gate.sh` 通过：247 passed，13 skipped；`git diff --check` 通过。
- 实机进程和网页服务当前未运行；没有作业实机验收证据。

本记录不授权使能或运动。

## 结论与冻结事实

F1：单机部署作业显式启用，启动仍 NAV；四轮 mode=8/9 的真实新反馈和发送ACK决定交接成功。

F2：IMU 使用部署已有接收节点 `/imu/data`，不新增 CAN writer；无有效IMU拒绝进入作业。

F3：复用 NAV 硬件/几何与换算，估计作业参数不构成实机精度、停车距离或动态验收保证。

## 遗留

实机切换和低速运动尚未验证；不能以离线通过替代现场验收。

后续现场使能暴露启动PDO瞬态、年龄估计抖动与激活错误退回unconfigured问题，
诊断和源码修复见 integration-20261029-04；本记录的离线通过不代表这些实机问题已验收。
