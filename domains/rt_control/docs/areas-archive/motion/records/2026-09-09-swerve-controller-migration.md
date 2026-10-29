---
id: motion-20260909-01
area: motion
title: 迁入 swerve_driver 算法与 CSP/CSV ros2_control 控制器
date: 2026-09-09
type: feature
trigger: ELECTRI-118，用户要求继续实际源码迁移
commits: []
env: both
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-144, BQ-145, motion-20260908-02, release-deploy-20260909-01]
---

## 背景

此前仅完成设计与静态配置。此轮按用户要求实际迁移源码，保留转向 CSP、驱动 CSV 和外置舵角观测，
不引入转向位置外环、在线补偿、rezero 或 MIT/力矩前馈。

## 改动

- 新增 `src/rt_control/swerve_driver`，从固定源 `7bd20cf21299c6b590e226550181037981d54ede`
  迁入运动学、舵角优化/斜坡、里程计、协方差及原有测试，保留 LICENSE/NOTICE 来源。
- 新控制器 `swerve_driver/SwerveController` 仅申请 4 个 position、4 个 velocity 命令接口；
  读取 8 个电机状态/模式与 4 个外置编码器，控制字仍由 enable_manager 管理。
- 新控制核心明确区分电机折算位置和外置实测舵角。零速度指令保持电机当前位置，反馈失效时
  使用最后有效测量保持，不把外置偏差或上一次规划目标变成隐含补偿。
- Twist 命令沿用 N-04 的本地接收间隔 500 ms。失活重新建立订阅并带会话标识，禁止旧队列命令重放。
- 私有 odom 发布实测速度/位置增量，支持可选 IMU 校验与无跳变切换，不发布 TF 或全局 joint_states。
- 无效反馈/模式、越界和旧 MIT 参数拒绝执行或配置；无自动降级续行、归零、故障恢复服务。

## 验证

- TDD：先复现缺失迁移代码，再让原有 29 项算法测试通过；另先复现零命令隐含转向、旧 DDS
  命令跨激活重放、NaN 电机反馈保持规划目标和旧参数静默接受等问题，再逐项修正。
- Native `colcon build/test`：64 条测试记录（58 个 GTest 用例及 6 个测试目标），0 错误/失败/跳过。
- 共享 controller_manager + GenericSystem Mock：加载、激活、接收 Twist、产生轮速、失活释放接口通过；
  实际验证控制器没有占用 control_word。该 Mock 不是实机使能。
- 侧向行驶的 ROS 消息/接口反馈/里程计闭环、命令超时、反馈中断、重激活、IMU 和配置拒绝测试通过。
- 算法/控制核心 ASan+UBSan：41 项用例通过。行覆盖率：控制核心 96.48%，控制器 89.19%。
- `tools/quality_gate.sh`：214 项工具测试通过，门禁覆盖率 83%；ShellCheck 本地缺失，留待 CI。
- 无网络、无设备映射的增量容器构建及同一套 64 条测试记录通过；独立容器启动 Mock manager 测试通过。
- 自审：域依赖方向、Robot Model/公共接口不变、旧机配置隔离 PASS；软件生命周期/接口回归 PASS；
  实机实时性与完整交付 UNVERIFIED。原有改动全部保留；无提交、推送或设备操作。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: swerve_driver 算法包和 ros2_control 插件已经迁入，不再只是设计名称；旧达妙协议代码未迁入。
- F2: 控制器输出 CSP/CSV 目标，不拥有总线、控制字、标定写入或设备恢复权限。
- F3: 外置编码器是独立舵角观测，不能覆盖电机反馈或形成运行中积分修正。
- F4: `update()` 使用固定大小数据和有界运算；ROS 实体创建/销毁属于生命周期阶段，需安排在受控无运动阶段，
  实机控制周期与切换抖动尚未验收。

## 遗留

真实 Kinco PDO/SDO、CANopen 编码器状态提供器（含 4 ms SYNC 和真实反馈年龄）、机械标定及
scope-specific 生命周期组合仍未接入。默认 launch 未加载新控制器，alfa_v3 runtime gate 继续关闭。
本轮未重建完整生产交付镜像，也未验证实机运动、同步精度或停车距离。
