---
id: arm-control-20261017-01
area: arm-control
title: 单臂台架运动 scope 与关节网页操作台
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 要求单臂台架网页操作台：关节滑块、URDF 可视化、拖动预览后确认发送、命名位姿保存与到达、休闲/运动两档、S 型曲线
commits: [feature/electri-127-bench-left-arm-console]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no, esc_diag_counters: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [ecat-axes-20261017-01]
---

## 结论

- 新增 `bench_left_arm` 运动 scope：7 轴 CSP，左臂轨迹控制器 `left_arm_jtc`，使能成功后由
  enable_manager 激活；`/rt/capabilities` 发布 `arm_joint_trajectory`。
- 新增 `rt_control_arm_console`：网页操作台，只用公共接口。
- 真实模式仍被渲染器拒绝：台架 7 轴方向为 `TBD`，`arms.yaml` 为 `verified: false`。
  方向由用户后续补齐。本记录不含任何实机运动。

## 运动规则（用户决定）

| 档位 | 速度 | 加速度 | 加加速度 |
| --- | --- | --- | --- |
| 休闲 | 10°/s | 20°/s²（速度的 2 倍） | 80°/s³ |
| 运动 | 30°/s | 60°/s²（速度的 2 倍） | 240°/s³ |

- 加加速度不是用户给定值：取加速度爬升时间 0.25 s。可在 `config/console.yaml` 修改。
- 轨迹由操作台生成：7 段 S 型曲线，关节空间直线，各轴同时到达，移动量最大的关节用满档位上限，
  每 10 ms 一个点（位置、速度、加速度），JTC 在点间做五次样条插值。
- 单点目标交给 JTC 时，JTC 的样条不受速度/加速度上限约束，所以不采用单点目标。

## 时间基准

- 每条轨迹带 `header.stamp`（发送后 50 ms）。操作台与控制器使用同一时间线。
- 停止轨迹沿用原轨迹的 stamp：前段为原轨迹自身的点（从当前时刻前一个采样点开始），20 ms 后开始
  按档位加速度和加加速度减速。若原轨迹剩余部分停得更早，保留原轨迹剩余部分，不越过原目标。
- 原因（Mock 实测）：停止轨迹第一点在未来时，JTC 先输出接收时刻状态一个周期（速度 0），
  再追赶第一点，出现 52°/s、4164°/s² 的尖峰。改为上述做法后无尖峰。

## 验证

本地 WSL 主机 DDS 发现不可用（talker/listener 也不通），ROS 链路在台架主机上隔离验证：
`ROS_DOMAIN_ID=91`、`ROS_LOCALHOST_ONLY=1`、`mock_components/GenericSystem`、
台架工作区的已打补丁 JTC（起点一致性检查开启），独立目录，未触碰 domain 77 的实机运行栈。

| 项目 | 结果 |
| --- | --- |
| 组合器测试 | 214 通过，1 失败（`test_cli_uses_installed_share_and_reports_rejection`，安装布局环境问题，main 同样失败） |
| 操作台单元测试 | 27 通过 |
| quality_gate | PASS |
| 运动档单段运动（J4 实测 `/joint_states`） | 最大速度 30.00–30.03°/s，最大加速度 60.4–61.1°/s²（10 ms 差分） |
| 运动中停止（巡航段） | 30.03°/s、60.7°/s²，无速度跳变（>3°/s） |
| 接近终点停止 | 保留原减速段，终点等于原目标 |
| 页面全流程（Chromium） | 登录、控制权、使能、拖动预览、确认、到达、保存位姿、停止均通过；JTC 无拒绝 |

## 未验证

- 实机：方向未验证，scope 真实模式不可组合；未发送任何实机轨迹。
- 实机跟踪误差下的起点一致性：停止轨迹第一点落后当前时刻一个采样（10 ms），
  30°/s 时约 0.3°，加上跟踪滞后须小于 JTC 1° 容差，需实机确认。
- J3 单圈回绕：模型限位 ±180°，操作台再收 2°；实际 J3 运动范围仍待确认。
- 操作台端口和令牌的现场部署方式（systemd 等）未定。
