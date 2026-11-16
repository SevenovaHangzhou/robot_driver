---
id: integration-20261116-02
area: integration
title: 头部与双臂独立运行及网页路由
date: 2026-11-16
type: commissioning
trigger: SEV-74；用户要求独立 head_only 与双臂制品，并将运行及网页改动单独上库
commits: [feature/head-arm-independent]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no, disable: yes, bus_start_stop: yes}
verified: PARTIAL
evidence:
  - /tmp/head-arm-independent-20261116/current-build.log
  - /tmp/head-arm-independent-20261116/current-test-results.log
  - /tmp/head-arm-independent-20261116/current-quality.log
  - /tmp/arm-sev74-t1-20261116-current/t1-wrapper-result.json
  - /tmp/arm-sev74-t1-20261116-current/web-route-verification.json
  - /tmp/head-arm-independent-20261116/final-evidence.json
  - /home/ar/rt-control-arch-20261106/releases/head-arm-independent-20261116/real-verification.json
  - /home/ar/rt-control-arch-20261106/releases/head-arm-independent-20261116/t1-wrapper-result.json
supersedes: []
related: [SEV-74, "#281"]
---

## 背景

[SEV-74](https://linear.app/sevenova/issue/SEV-74/确保arm模块可以独立运行)：
[GitHub #281](https://github.com/SevenovaHangzhou/robot_driver/issues/281)。
头部不经 EtherCAT，使用独立 head_only 制品；双臂保留 bench_dual_arm 控制栈。
网页仍在同一端口显示两区，但使能、失能、复位和实例退出分别路由到各自控制栈。

## 改动

- composer 的可选 --instance-file 校验绝对命名空间和精确 ROS 路由，将其保存到制品，
  在生成阶段限定控制器参数节点名。旧的根命名空间制品保持原输出。
- 通用 launch 将制品中的实例配置显式传递给全部节点，包括延迟启动的 spawner；
  启动不重写配置。scope_shutdown 自动读取同一实例配置，避免头部退出调用双臂失能服务。
- 台套 head_instance.yaml 保存现场读回的路由；网页 --head-instance-file 读取制品中的
  instance.yaml，仅影响头部桥接。独立头部使能/复位确认显示“头部”，机械臂确认不再固定写“左臂”。
- 不包含墙钟、线程 CPU、调度切换、缺页计数及相关 trace 补丁；这些诊断候选继续独立保留。

## 验证

本次提交的优化版本基于 main `2129bc6496a32f21b69d37fa779cfe29e632d34e`，未重新部署到实机：

- T0：`colcon build` 完成 5 个受影响包；`colcon test` 覆盖受影响包及下游，
  699 项测试，0 error、0 failure、0 skipped。`tools/quality_gate.sh` 通过
  （185 passed、11 条条件跳过），`ruff format --check`、`ruff check` 和 `git diff --check` 通过。
- T1：192.168.100.40 上使用独立临时目录、ROS domain 216、GenericSystem mock，
  经 `rt_control_start` 并行启动双臂及头部；双臂配置 14 关节、头部配置独立硬件握手，
  两侧运动控制器均 inactive。头部有序退出后双臂使能管理器仍 active，运动控制器仍 inactive；
  随后关闭双臂，两个启动器退出码均为 0。证据路径 `/tmp/arm-sev74-t1-20261116-current/`
  位于工控机，未连接真实总线。
- 网页：实际 JS 模块离线验证头部/合并作用域确认文案、取消时零请求以及头部 WebSocket；
  ROS 图确认头部桥接客户端为 `/head/rt/enable`、`/head/rt/disable`、
  `/head/rt/reset_fault`，没有根命名空间的对应客户端，控制调用次数为 0。

此前候选的现场证据（路由相同，参数当时在启动阶段加命名空间）：

- T1：两实例并行 mock，关闭头部后双臂仍运行，再关闭双臂，两个启动器均退出 0。
- T3：有序失能并停止旧合并栈，启动双臂与独立头部；最终两个使能管理器均 IDLE，
  运动控制器 inactive。双臂 14 个驱动为 switch_on_disabled；头部 phase=0、
  motion_allowed=0、fault_code=0，关节及内部反馈持续更新。
- 网页：实际 JS 模块离线验证取消不发请求、确认使用头部 WebSocket；未发送实机使能。
- 初次部署缺共享库链接及一次头部查询超时已记录并修复打包、重启核验；
  偶发 DDS 查询超时根因尚未确定。

本记录不授权使能或运动。

## 结论与冻结事实

- F1：头部和双臂使用独立 controller_manager 与 enable_manager；总线锁仍阻止重复占用。
- F2：登录、审计和操作权协调沿用现有逻辑；同一作用域内部的使能/故障联动保持原定义。
- F3：头部桥接的使能、失能、复位服务使用独立实例路由；动作接口仍沿用公共头部契约。

## 遗留

优化版本未重新部署实机；实机独立使能、运动、故障注入和完整生产验收未验证。
现有部署仅有一个头部动作服务，本轮未验证同域多个头部动作服务共存。
