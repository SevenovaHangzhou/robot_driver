---
id: motion-20260919-01
area: motion
title: ELECTRI-136 七轴 CSP 重力前馈离线库与控制器
date: 2026-09-19
type: feature
trigger: ELECTRI-136；用户批准离线阶段实现
commits: [feature/ELECTRI-136-gravity-ff]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-152, ELECTRI-135, MECHINE-35, ecat-axes-20260919-01]
---

## 背景

V3 自研双七轴保持 CSP 位置控制，ELECTRI-136 阶段一增加独立 0x60B2 重力前馈；当前
URDF 质量与力矩千分比换算均未闭合，因此只允许离线/Mock 和默认 shadow，不接生产启动面。

## 改动

- 新增纯 C++ `rt_arm_dynamics`，由原始 URDF 字符串构建 Pinocchio 单臂 reduced model；
  初始化时显式锁定非本臂关节、按请求顺序映射关节，并由输入端惯量和减速比计算 armature。
- 为绕过当前模型 link/joint 同名，只在内存把 link `left/right_joint1..7` 改为
  `left/right_link1..7`；源 URDF、关节名和公共模型均不修改。锁定的 continuous 关节使用
  Pinocchio 的 cos/sin 配置坐标；保留的七轴仍要求标量关节。Pinocchio 3/4 头文件由
  `__has_include` 兼容，运行函数使用预分配向量、错误码和 SI N.m。
- 阶段一 `gravity(q)` 已实现；`feedforward(q,v,a)` 仅在 v/a 精确为零时返回重力，非零输入
  返回 unsupported。负载按 frame 挂接且每次基于原始惯量重建，不累加旧负载。
- 新增 `gravity_ff_controller`，左右臂各可独立实例化。shadow 不认领命令接口，发布
  `gravity_nm`、原始 `torque_actual_permille`、模型/换算验证状态；active 只认领 effort，
  与 JTC position、enable_manager control_word 可同时激活。
- active 同时要求原始 URDF SHA-256/来源验证和全部逐轴换算验证；来源为 `TBD` 时即使
  `verified: true` 也拒绝配置；`max_effort_nm`、正常 slew 必须显式配置。
  shadow 不要求限幅或换算参数；active 可在 `torque_actual_interface` 为空时不认领
  可选 6077 状态，shadow 则要求原始 6077。scale 可在线设为 0..1，拒绝批量参数更新时
  不发生部分写入；其他策略参数配置后不可变。
- 初次使能前持续按实际位置预加载；正常失能时，仍有轴为 Operation Enabled 就继续计算，
  全组离开使能后的过渡状态保持最后值，全部到 Switch On Disabled/Not Ready 后恢复预加载。Fault Reaction Active 保持
  最后有效输出；Fault 立即归零并锁存；非法位置/状态/模型在最近有效状态为使能时按
  `fault_slew_nm_per_s` 归零，否则直接归零。锁存只在停用/重新激活时清除。
- `on_deactivate` 直接写零，带载使用前必须先把 scale 降到零并确认输出归零。
- Pinocchio 发行包固定为 `ros-humble-pinocchio=4.0.0-2jammy.20260606.100000`，通过
  `pinocchio::pinocchio` 链接；未 vendor 源码、未加入 deps.repos、未增加本机优化选项。

## 验证

- 无设备映射的开发容器内 69 个包全量编译通过。动力学库 13 个 gtest 通过（含 CTest
  包装共 14 条 test-result 记录），覆盖
  单/双摆解析重力、`rnea=M*a+C*v+g`、armature、负载 `-J_v^T m g_world`、
  同名改写、双七轴独立约简/顺序/锁定、非法输入、不累加负载和非零 v/a 拒绝；
  测试还同时启用 Eigen malloc gate 与 glibc 堆分配探针。
- 合成 2-DoF 基准只输出均值与最大值，不设阈值；不是七轴或实时性能结论。
- 控制器 10 个纯状态机、7 个直接插件、1 个 manager 集成 gtest 和 1 个配置 pytest
  全部通过（含 CTest 包装共 23 条 test-result 记录）。实际 JTC、enable_manager、active gravity 与 shadow 在一个 Mock manager
  中同时激活，三类命令接口无冲突；影子无力矩限幅准入、active 双门禁和可选 6077、
  原始诊断、scale、Fault/非法反馈斜坡及 deactivate 归零均有针对性覆盖。
- 硬件包 103 项通过；容器内 `tools/quality_gate.sh` 为 288 passed、11 skipped。
  `tools/run_scoped_tests.sh --base v3` 完成 69 包编译后跑全量测试，初次并行执行有 5 包
  失败；enable_manager、力传感器 broadcaster、Modbus 生命周期和 pinned JTC 的长时测试
  逐包串行复验通过。上游 GUI 测试 `rqt_joint_trajectory_controller` 缺少 `rqt_gui`，仍失败；
  全量测试门禁因此未通过，不能标记为 PASS。
- Dockerfile 镜像构建已尝试；官方 ROS apt 下载 `ros-humble-eigenpy` 长时间停滞，手动
  结束后状态为取消，未产出干净镜像或启动证据。以上不作为生产封装验证。
- 没有执行实机、总线、SDO/PDO 写、复位、使能或运动。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 重力模型输出与控制器输入统一为关节输出端 N.m；N.m 到 60B2 原始千分比只属于硬件映射层。
- F2: shadow 是默认模式且无命令接口；active 必须同时通过模型原文哈希与全部逐轴换算门禁。
- F3: 初次使能前及确认失能终态后预加载、失能过渡保持；Fault Reaction 保持、Fault 归零、非法反馈条件斜坡的语义按本记录冻结。
- F4: 当前 V3 模型/换算未验证，新增包未接入默认启动面，离线通过不构成 active 或实机准入。

## 遗留

BQ-152 全部事实、ELECTRI-135 预加载/抱闸时序和 MECHINE-35 质量模型未闭合。还需以正式七轴
模型重跑离线数值验证，完成只上行、shadow 标定、逐轴低 scale active 和故障反应分级实测。
