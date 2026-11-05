---
id: control-20261105-01
area: control
title: 精简 rolling 协议：会话随模式切换，批次用 ROS 时间，限值取机器人模型
date: 2026-11-05
type: behavior
trigger: 用户 2026-11-04/05（代表运控评审）：运控是唯一客户端且同机；保留缓冲、拼接、限值与停车检查、断流刹停，砍掉协议外壳；状态 5 个字段；位置余量 0.05 rad；刹车减速度用加速度上限
commits: [feature/arch-rolling-slim]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261104-03]
---

## 改动

- 接口按 robot_interfaces#16：删除 open/close 服务；批次 `epoch`、`sequence`、`replace_from`（ROS 时间）、`points`；状态 `epoch`、`state`、`last_sequence`、`last_result`、`stop_reason`；`SetJointControlMode` 只收 `target_mode`。
- rolling 控制器：
  - 激活即 epoch 加 1 并保持；第一批（或刹停后的下一批）开启内部会话，实时线程在 `replace_from` 到达时开始跟随（`update()` 的 ROS 时间对比原子量），之后的批次按 `replace_from − 起点` 换算到会话时间。
  - 内部的 boot/session/client 标识、快照交接、缓冲、拼接、限值、停车可行性、刹停逻辑不变；对外不可见。
  - 删除预热超时与初始缓冲要求、测试用内置包络、限值来源三态、包络文件加载（yaml-cpp 依赖一并去掉）、SHA 解析、服务结果映射、模式结果订阅。
  - 限值参数由 composer `model_limits` 从 robot_description 填入：位置上下限（URDF）、速度（URDF，双臂 30°/s）、加速度（限位文件，60°/s²）；`position_margin: 0.05`（运动策略）；刹车减速度等于加速度上限。参数无代码默认值，缺失即配置失败。
  - 反馈新鲜度改用 `hw_semantic_components::feedback_fresh`。
- enable_manager：`SetJointControlMode` 去掉协议号、UUID、幂等缓存、`expected_mode` 和切换证据字段；删除 `/rt/internal/joint_control/mode_result` 及 `rt_control_interfaces/JointControlModeResult`；rolling 作为切换源时要求 `state == HOLDING`，接管容差只对 JTC 输出检查（rolling 激活自检接管，JTC 从实测起步）。
- 切换前静止判定改用驱动器实测速度（台架 2026-11-05：1 ms 位置差分把编码器量化放大到阈值以上，静止手臂被判为 SOURCE_MOVING；做法取自同事的 rolling 拆分）。实测速度由 hw_ethercat 在渲染了 `velocity` 状态接口时导出 `velocity_state_interfaces`，enable_manager 参数 `motion_velocity_interfaces`；没有时仍用位置差分。
- readiness：可切换的运动控制器（JTC 与 rolling）由 lifecycle `mode_controllers` 列出，要求恰好一个 active；之前两者都要求 active，切换作用域使能后永远 `controller_inactive`（台架 2026-11-05 发现）。
- composer：删除 `policy_file` 参数源、`digest_as` 与 `rolling_axis_set_sha256`；删除 `Alfa_V3_arm_envelope.mock.yaml`。
- 运控对接说明 `v3-arm-motion-integration.md` §6、§7.2 按新协议重写。

## 已知行为

缓冲耗尽时 RT 在剩余缓冲小于保护时间之和时开始刹停（`STOP_BUFFER_END`），以静止点收尾的运动需在末点后再留一段静止（不短于保护时间之和），否则停点差最后一小段（测试中约 2e-4 rad）。原 rolling 行为相同。

## 验证

本机：ctrl_rolling_trajectory 9 个测试程序全部通过（新 `test_controller_batches` 9 例，重复 20 次无失败）；ctrl_enable_manager 95 项；integ_composer 236 项（`test_cli_uses_installed_share_and_reports_rejection` 为既有环境问题）。未上台架。

## 遗留

- 台架：与 JTC 流式同条件 joint7 对比。
- robot_interfaces#16、robot_description#9 合入后改锁定版本。
