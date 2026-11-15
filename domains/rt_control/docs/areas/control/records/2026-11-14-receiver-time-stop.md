---
id: control-20261114-02
area: control
title: 可选的执行周期制动保持位置、速度和加速度连续
date: 2026-11-14
type: fix
trigger: 用户要求修复 J6 plan-stop 延迟接管跳变并部署验证
commits: [codex/j6-stop-continuity]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - Dynamic_Parameter_Identification/artifacts/j6_stop_repair_20261114/release/tests03/test.log
  - Dynamic_Parameter_Identification/artifacts/j6_stop_repair_20261114/release/compose.log
  - Dynamic_Parameter_Identification/artifacts/j6_stop_repair_20261114/release/target_mock-remote.log
supersedes: []
related: [control-20261114-01]
---

## 背景

网页提前规划停止曲线时，实际接管时间仍可能晚于预测时间。复现用例在原轨迹
235 ms 请求、预测 255 ms 接管、实际 285 ms 接管，出现约 1.2°/s² 的参考加速度差。
保留首点加速度只能修复规划接点，不能解决消息晚到后的接管差异。

## 改动

增加可选 `ctrl_stoppable_trajectory`，继承冻结并带本仓库补丁的 JTC；域内私有
`StopJointTrajectory` 携带当前 FJT UUID 和档位速度、加速度、jerk 上限。
消费请求的实时周期从原轨迹实际位置/速度/加速度生成有界制动，不预测接管时刻。
制动参考到静止且满足既有跟踪容差后才应答，原 FJT 以 ABORTED / INVALID_GOAL 结束。
重复请求、过期 UUID、反馈过期、时钟倒退和生命周期中断分别拒绝或返回失败。

网页显式配置 `stop_backend: controller` 才使用新服务；失败不替换为普通 FJT cancel，
控制权交接仍等待终态之后的新鲜静止证据。默认仍为 `planned`，现有 scope 不选新插件。
原规划路径同时保留切点的完整状态和 jerk 分段边界，对无法满足编码精度的短段拒绝发送。
进度条在控制器接管期间显示等待确认，不显示预测停止时长。

## 验证

- 基于主线 `73cdce7cf71e95f793666b3c7f0252b7d685dc34`：13 个包构建通过；实际 `colcon test`
  覆盖接口、新控制器、enable manager、rolling、网页、bringup、composer、head 和 diagnostics，
  `colcon test-result` 为 982 tests、0 errors、0 failures、0 skipped。
- `python3 tools/arch_check.py compose --against HEAD`：27 个既有选择的生成结果不变。
- 工控机候选目录使用现场 JTC 头文件和库重新编译接口及插件，7 组真实 ROS/service/update
  组件测试通过。延迟接管的 q/v/a 连续性、重复/竞争请求、反馈损失、生命周期和缺失接口均覆盖。
- 工控机 T1：独立网络、IPC、挂载空间，仅回环 DDS、私有 `/dev/shm`，所有硬件为 GenericSystem。
  实际安装版 `rt_control_start` 加载双臂、头部、六维力发布、enable manager 和 Runtime-LOG；
  候选网页的轨迹生成器及停止客户端完成 FJT→停止服务→原目标 ABORTED 的调用；
  新控制器→rolling→新控制器切换通过；TERM 有序退出、硬件 shutdown、主进程 exit 0。
- 现场旧 `scope_capabilities` 在退出时报告 `ExternalShutdownException`，exit 1；
  controller manager 与硬件均已正常退出。此既有退出提示不视为新控制器运行故障，也不隐去。
- 未验证：实机 T3/T4、物理停止曲线、生产容器镜像。T1 的 GenericSystem 生命周期和虚拟
  轨迹不构成使能或运动实机的授权。插件必须针对目标 JTC ABI 重编译，不能跨版本拷贝二进制。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 默认 scope 与 `planned` 后端仍可用；新停止路径须显式选择插件并设置网页配置。
- F2: 新路径从消费请求的实际周期接管参考状态，响应只确认参考制动完成，不证明物理停稳。
- F3: 私有接口、插件、controller 选择、网页 Python 和进度显示须作为配套候选部署；回退须同时恢复。

## 遗留

候选部署和隔离 T1 已准备；正式控制栈切换、未使能启动检查和受控实机停止测试仍待执行。
