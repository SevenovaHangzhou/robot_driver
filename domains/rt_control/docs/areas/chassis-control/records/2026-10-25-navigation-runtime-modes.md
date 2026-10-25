---
id: chassis-control-20261025-01
area: chassis-control
title: 导航连续与对齐模式本地集成及控制轴反馈
date: 2026-10-25
type: feature
trigger: 用户要求替换新版底盘驱动并同步仿真、导航与双模式验收
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - /home/robot/wheel_nav/log/chassis-modes-20261025/build-real-driver-06.log
  - /home/robot/wheel_nav/log/chassis-modes-20261025/tests-real-driver-03.log
supersedes: []
related: [ELECTRI-127]
---

## 背景

按用户提供的 chassis-mode-runtime 源码替换旧工作区，导航 FollowMPPI 对应连续模式，
CrabMPPI 对应对齐模式。用户授权源码修改和仿真测试，没有执行实机使能或运动。

## 改动

swerve_driver 启动 drive_mode 从 align_first 改为 steer_while_driving，运行切换仍复用
新源码原有 SetBool 服务及控制周期确认。新增只读 sensor_msgs/JointState
`/rt_control/chassis/joint_states`，使用控制轴角度和真实轮速，不使用 URDF 显示角符号。
消息最多 100 Hz，SensorDataQoS，ROS 时间戳，base_footprint 坐标系。
module_1..4 的 steering/wheel 名称与生产模块编号一致；角度 rad、角速度 rad/s。
预分配 RealtimePublisher 消息，不在实时周期分配向量；反馈失效时停止发布。
增加 Ruckig 库环境 hook，并在 wheel_driver 中提供独立依赖路径。
修复纯平移入口将已对齐的混合正反转布局强制改为共同分支的问题：仅在所有轮轴
已经位于释放角容差内时保持等价分支，未对齐时继续使用共同可行分支。
这避免小角速度经过零点时出现 180° 目标跳变、对齐门控停驱与 S 曲线反复制动。
仿真共用源码及导航生产快照一并更新，门控阈值、机械限位和轨迹速率限制不变。

## 验证

单包串行 colcon build 成功；source driver_env.bash 后运行
`ctest --test-dir build/swerve_driver --output-on-failure`，11/11 测试组通过。
包含实际模式握手、运动拒绝、默认连续、控制轴反馈单位/顺序和失效样本抑制。
新增 PureTranslationKeepsAlreadyAlignedEquivalentBranches：修复前复现失败，
修复后通过；已有 TranslationUsesOneFeasibleBranchForAllModules 仍通过。
未执行硬件总线、容器发布门禁、实机使能或运动；该结果只代表 mock 层。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本地启动默认连续，服务成功仍由控制周期确认，运动请求拒绝且不排队。
- F2: 新只读反馈为控制轴原始观测；分支连续性修复不改变轮式加 IMU 里程计或标定。
- F3: 公共新话题为本次本地集成适配，未修改冻结公共 IDL 或声明正式发布契约。

## 遗留

真实设备部署仍需目标工控机的生命周期和低速运动验收；双模式 box_room 仿真验收在导航日志单独记录。
