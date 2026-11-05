---
id: control-20261104-03
area: control
title: JTC 流式轨迹话题与 URDF 速度上限准入
date: 2026-11-04
type: behavior
trigger: 用户 2026-11-04：遥操作、阻抗控制、视觉伺服、在线重规划只有运控一个客户端，断流后保持即可；限值由实时侧 JTC 补丁检查（方案 B），上限只用 URDF
commits: [feature/arch-arm-joint-stream]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [arm-control-20261019-01, integration-20261104-01]
---

## 背景

补丁 0001 的起点检查要求首点距实测位置 ≤ 1°，启用后直接关闭 JTC 的 `~/joint_trajectory` 话题，流式下发只能走 FJT action 或 Rolling。流式下发时首点在未来几十毫秒，按 0001 会被误拒。

## 改动

新增 `patches/ros2_controllers/0006-stream-topic-velocity-admission.patch`，接入 bootstrap 和 Dockerfile：

- 话题入口恢复，名字由新参数 `trajectory_topic` 给出（R-IN-22 `/rt/arms/joint_trajectory`，单臂 R-IN-22L/R），QoS `Q_CONTROL`。
- action 与话题共用一条准入规则：`|首点 − 实测| ≤ 容差 + 速度上限 × 到首点的时间`；每点速度与每段平均速度不超过 `joint_velocity_limits`。首点时间为 0 时与 0001 原行为一致。
- 话题轨迹被接受时中止正在执行的 action 目标（ABORTED，`INVALID_GOAL`），以最后到达者为准。
- `joint_velocity_limits` 在启用起点检查时必填，由 composer 新参数源 `model_limits` 从模型 URDF `<limit velocity>` 按策略关节顺序填入；策略不再携带速度数值。
- 运动策略 `allow_nonzero_velocity_at_trajectory_end: false`（schema 固定为 false）：每条轨迹必须以 0 速度结束，断流时在最后一点平滑停住，不会出现速度阶跃。

## 验证

本机打补丁构建 JTC：`test_trajectory_controller` 501、`test_trajectory_actions` 114、`test_tolerances`、`test_trajectory` 全部通过，其中新增 5 例（必填校验、起点按速度放宽、点/段超速拒绝、话题准入与替换、话题中止 action 目标）。`test_load_joint_trajectory_controller` 因本机 test_assets 插件缺失失败，与本改动无关。integ_composer 236 项通过（`test_cli_uses_installed_share_and_reports_rejection` 在 main 上同样失败，源码树运行环境问题）。未上台架。

## 台架（2026-11-04 22:03，用户授权，bench_dual_arm，域 78）

只动两侧 joint7（±0.05 rad 正弦，峰值 0.052 rad/s），motion_guard 3°。

- 流式 18 s（50 Hz、两点窗口、末点 0 速度）：最大跟踪误差 0.163°，无故障。
- 断流：保持 3 s 漂移 8.4e-5 rad，无故障；断流后 0.5 s 内走了 0.034 rad，大于预期（约 0.01 rad），原因未查明。
- 超速拒绝：0.03 rad / 5 ms 的单点窗口**被接受**（规则允许 1° + 3.14 rad/s × 5 ms = 0.033 rad），两侧 joint7 报 0x8400 速度误差超限，enable_manager 急停，全部轴去使能。
- action 交替：因上一步故障未执行。

结论：只限速度不能防止速度阶跃；1° 静态容差加在首点时间 > 0 的轨迹上等于允许 1° 瞬跳；`open_loop_control: false` 时每条新轨迹从实测位置、零速度（只有位置状态接口）起步。修正方案待定，本 PR 保持 draft。

## 结论与冻结事实

- F1: 流式与 action 共用 JTC，准入规则相同，速度上限唯一来源是模型 URDF。
- F2: 断流后 JTC 走完最后一条轨迹并保持；轨迹末点速度必须为 0。

## 遗留

- 台架：低速流式下发、断流保持、流式与 action 交替。
- 上游 Humble `preempt_active_goal()` 对执行中的目标调用 `setCanceled`，属于非法状态转换（本次测试中观察到 `invalid transition from state EXECUTING with event CANCELED`），新 action 抢占旧 action 时旧目标不会正常结束；本补丁只在话题路径改用 ABORTED，action 路径未改。
- 段内样条插值可能略超过平均速度；加速度未检查。
- robot_interfaces#14 已合入（15525d1），`deps.repos` 与 `src/interfaces/source-lock.yaml` 已锁定。
