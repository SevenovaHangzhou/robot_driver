---
id: control-20261118-01
area: control
title: 新增逐轮观测广播器 /rt/base/wheel_observations，修正打滑诊断轮位标签
date: 2026-11-18
type: feature
trigger: ELECTRI-227；导航《逐轮观测融合：驱动输出接口口径 v0.1》（NAV-144）；用户 2026-11-18 确认轮位按研华单机标定、诊断标签直接修
commits: [feature/wheel-observations]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [hw-config-20261116-02, control-20261107-02]
---

## 背景

导航要做逐轮 EKF 融合。`/rt/base/odom` 只有融合后的底盘量，而且只在 `swerve_controller`
激活时发布：底盘失能和 `FJT_READY`（`base_wheel_jtc` 接管）时没有任何底盘观测。
另外打滑诊断按模块下标写 FL/FR/RL/RR，与研华单机标定（module_1 右前、2 右后、3 左前、4 左后）不符。

## 改动

- 新包 `ctrl_wheel_observation_broadcaster`：只读广播器，只认领状态接口，chassis_only 启动即激活。
  发布 R-OUT-29（robot_interfaces `feat/R-OUT-29-wheel-observations`，待合并后升 pin），100 Hz，`fast_state`。
  - 每轮三个通道：行走（0x6064/0x606C 同一 PDO）、转向电机（位置 × `steering_to_yaw_sign`）、外置编码器；
    两路转向角分开，无效不互相填补。
  - 时间：驱动器通道 `READ_COMPLETE`（周期时间 − `feedback_age_ms`）；编码器 `HOST_RECEIVE`
    （周期时间 − 编码器 `feedback_age_ms`，用户态回调时刻）。
  - 样本序号：驱动器用 `feedback_sequence` 变化；编码器无计数器，按接收年龄下降判定新帧（每 1 ms 周期运行）。
  - 有效性：未收到/超时（沿用 `swerve_controller.feedback_timeout` 0.2 s）/非有限/CiA402 故障；不以使能为条件。
  - 连续性：无效间隙后、计数回退、行走位置单步 > 1000 rad 时 epoch 加一；激活或 ROS 时间回退时换 session。
  - 轮位按标定坐标象限排成 FL/FR/RL/RR，并随消息给出转向轴坐标与轮半径。
- `calibration_id`：`infra_renderer.calibration_id()` 对渲染器实际使用的事实做 SHA-256；
  底盘取机型模块、单机标定、Kinco profile 与器件，编码器取逐节点换算参数。
- 组合：`controllers.yaml` 新条目、chassis_only 绑定与 active 列表；控制器 schema 的 yaml 参数源允许 `as`。
- `ctrl_swerve`：新增 `module_quadrant()`；`slipping_modules` 诊断改用标定坐标推出的轮位；README 更正模块顺序说明。

## 验证

- T0 本机：`ctrl_swerve` 97、`ctrl_wheel_observation_broadcaster` 11、`infra_renderer` 5 项测试通过；
  `integ_composer` 242 项中 239 通过，其余 3 项在 main 上同样失败（本机缺安装 share，与本改动无关）；
  `hw_canopen`/`hw_ethercat` Python 测试 22 项通过；`tools/quality_gate.sh` 通过。构建基于本地 robot_interfaces 分支。
- 未验证：T1 mock 启动（mock 下计数不前进，各通道应为 NOT_READY）、研华实机只读发布、频率/延迟/时差统计（需求 A5）、
  底盘失能与 FJT_READY 下持续发布。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 打滑诊断与逐轮观测的 FL/FR/RL/RR 由标定坐标象限推出，不由模块下标推出。
- F2: R-OUT-29 的转向角为航向（电机角 = q × `steering_to_yaw_sign`，编码器角原值），行走量为轮轴 rad、rad/s。

## 遗留

- robot_interfaces 已临时锁定到分支提交 `46a027308ca08b7931ae25eba5638957411e5279`；其 PR 合并后须换成 main 上的合并提交再合并本 PR。
- 接触点偏置、噪声、Kinco 速度反馈滤波（60F9.05/06）未知，需另行标定或只读回读。
- 行走电机为增量编码器，int32 计数约 ±49,100 rad 后回绕，靠 epoch 标出。
