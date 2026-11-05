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
- 切换前静止判定改为窗口位移：20 个连续采样（约 20 ms）首尾位移 ÷ 时长低于阈值（运动策略 `mode_switch_stable_interval_count: 20`）。原来逐个 1 ms 间隔差分，一个编码器计数就超过阈值，静止手臂被判为 SOURCE_MOVING（台架 2026-11-05）；窗口内一个计数只相当于 0.0006 rad/s。整机与台架同一逻辑，不依赖 TxPDO 是否映射实测速度（用户 2026-11-05 选方案 A）。
- readiness：可切换的运动控制器（JTC 与 rolling）由 lifecycle `mode_controllers` 列出，要求恰好一个 active；之前两者都要求 active，切换作用域使能后永远 `controller_inactive`（台架 2026-11-05 发现）。
- 拼接（含首个批次）改为从缓冲状态连续接续：新批次首点只用于检查是否在拼接容差内，指令不再在拼接点跳变；拼接容差随之放宽到 0.05 rad / 0.1 rad/s（用户 2026-11-05）。首个批次的首点同理，轨迹从保持位置精确起步（客户端只能从反馈得知保持位置；原协议由 open 返回精确值）；首批开始前到达的批次接续首批，不重开会话。
- composer：删除 `policy_file` 参数源、`digest_as` 与 `rolling_axis_set_sha256`；删除 `Alfa_V3_arm_envelope.mock.yaml`。
- 运控对接说明 `v3-arm-motion-integration.md` §6、§7.2 按新协议重写。

## 已知行为

缓冲耗尽时 RT 在剩余缓冲小于保护时间之和时开始刹停（`STOP_BUFFER_END`），以静止点收尾的运动需在末点后再留一段静止（不短于保护时间之和），否则停点差最后一小段（测试中约 2e-4 rad）。原 rolling 行为相同。

## 验证

本机：ctrl_rolling_trajectory 9 个测试程序全部通过（新 `test_controller_batches` 9 例，重复 20 次无失败）；ctrl_enable_manager 95 项；integ_composer 236 项（`test_cli_uses_installed_share_and_reports_rejection` 为既有环境问题）。未上台架。

## 台架（2026-11-05，用户批准的计划内对比测试，`~/rt-control-arch-20261105`，bench_dual_arm，域 78）

只动两侧 joint7：偏移 0.05·(1−cos ωt)，周期 6 s，峰值 0.052 rad/s；同一部署、同一运动，先 rolling 后 JTC 流式。

| 项 | rolling（30 Hz，0.1 s 节点，0.5 s 窗口） | JTC 流式（50 Hz，两点窗口） |
| --- | --- | --- |
| 模式切换 | FJT→rolling、rolling→FJT 均 accepted | — |
| 流式 19.5 s | 569 批 0 拒绝；实测与计划最大差 0.067°（含时间对齐误差） | 跟踪误差 0.060° |
| 断流 | 更新超时刹停（`STOP_UPDATE_TIMEOUT`），刹停后多走 0.0028 rad，保持 3 s 漂移 3.6e-5 rad | 走到窗口停止点 0.0159 rad，漂移 3.6e-4 rad |
| 超速 0.03 rad / 5 ms | `LIMIT_EXCEEDED`，未动 | 拒绝，未动 |
| 回起点 | 103 批 0 拒绝，残差 5e-5 rad | FJT 残差 1e-4 rad |

全程 readiness HEALTHY、无驱动器故障；去使能后有序停机，18 PREOP。过程中发现并修正：静止判定被编码器量化误判（台架时临时用实测速度验证，之后按用户决定改为窗口位移，待复测）、readiness 要求两个可切换控制器都 active、首批首点要求与保持位置逐位相等、首批开始前的批次重开会话。

## 遗留

- 台架：与 JTC 流式同条件 joint7 对比。
- robot_interfaces#16、robot_description#9 合入后改锁定版本。
