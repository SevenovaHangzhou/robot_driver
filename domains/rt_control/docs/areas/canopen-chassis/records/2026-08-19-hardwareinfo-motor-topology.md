---
id: canopen-chassis-20260819-01
area: canopen-chassis
title: CANopen 硬件包接管变体 Xacro 与电机拓扑
date: 2026-08-19
type: feature
trigger: ELECTRI-94
commits: [feature/rt-control-硬件配置分层组合]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [final image rt-control:electri-94-franka-layering-final-bbe5 sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4, ros2_canopen 0005 SHA-256 bbe5fddd32760e06eda7c558984647990fbe5bbfb629753e3bebeeaafe315b0a, pristine upstream CANopen 8-package build and 178 tests, focused Python 165/165, quality_gate 196/196 at 83%, Mock container a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580 exit 0]
supersedes: []
related: [BQ-027, BQ-122, BQ-132, ecat-axes-20260819-01, lifecycle-20260819-03, contract-20260819-06, release-deploy-20260819-01]
---

## 背景

ELECTRI-94 要求 CANopen 与 EtherCAT 作为两个独立 hardware component 由统一
bringup 组合。原先在上游插件内固定 Node 2/3 的做法会把机型拓扑泄漏进通用适配层，
也无法让硬件包独立拥有自己的 real/mock 描述。

## 改动

- `robot_hw_canopen` 公开 `rt_control_canopen_system` 分发宏，并以
  `variants/alfa_v1.yaml` 注册 system 及 joint/node/mode/side/profile。公开宏加载所选
  descriptor 生成 real/mock joint topology；真实分支继续由 `Cia402System` 根据
  `bus.yml` 动态导出接口，Mock 明确提供 velocity command 与 position/velocity state，
  bringup 中原有的重复 Mock schema 已删除。
- `alfa_v1` 仍配置 `left_track_joint=Node 2`、`right_track_joint=Node 3`、Profiled
  Velocity mode 3，保持 bus/EDS/DCF、换算、方向和 joint/interface ABI 不变；未知
  变体通过 `xacro.fatal` 拒绝。
- configure/build 门禁严格要求 descriptor node 名集合、Node ID 和 operation mode 与
  `bus.yml` 完全一致；生成 `.bin` 文件名从已验证 descriptor 派生，不再在 CMake 中
  固定左右节点清单。当前只准入 `ld2_drive` 与 mode 3。
- pinned `ros2_canopen` 补丁在 `on_init` 从每个 joint 的 `HardwareInfo` 读取
  `node_id` 与 `operation_mode`，要求 Node ID 在 `1..127`、唯一且仅接受当前可安全
  预置零速度目标的 mode 3。EMCY 组停与 activate/rollback/deactivate 顺序遍历该冻结
  拓扑，不再在插件代码中固定 Node 2/3。
- 这只建立受控机型接缝；首个生产配置仍是两节点，未声称任意电机数量已经通过
  controller、生命周期或实机验收。

## 验证

- 最终 `ros2_canopen` 0005 补丁 SHA-256 为
  `bbe5fddd32760e06eda7c558984647990fbe5bbfb629753e3bebeeaafe315b0a`；在冻结上游的
  pristine 工作树完成 8-package 构建并通过 178 项测试。focused Python 165/165、
  `tools/quality_gate.sh` 196/196（门禁覆盖率 83%）通过。
- 最终镜像 `rt-control:electri-94-franka-layering-final-bbe5`（ID
  `sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4`）成功构建
  29 个包。精确该镜像的无设备 Mock 容器
  `a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580` 在
  `network=none`、private IPC、无 device/capability/bind 的条件下得到预期 controller
  状态，controller manager 为 250 Hz，SafetyState 保持 `NOT_READY` 并 fail closed；
  BQ-122 顺序成立，容器退出 0，日志未出现 `ERROR`、`FATAL`、
  `UNCLEAN_SHUTDOWN` 或 `SIGSEGV`。
- 本记录的 `PASS/T1` 只表示最终镜像内的软件构建、契约和无设备 Mock 通过。未执行
  CANopen 报文/boot、EMCY fault injection、HIL 或实机生命周期；未访问 CAN 接口或
  设备，未发送 NMT/SDO、reset、enable、运动或 PLC 输出。本机没有 ShellCheck，仍由
  CI 补跑。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: CANopen descriptor 是 joint/node/mode/side/profile 的 owner-local 注册点，驱动
  real/mock topology、diagnostics node 派生和生成文件清单；真实运行配置仍由同包
  `bus.yml`/EDS 持有并与 descriptor 严格对齐。
- F2: 当前唯一允许的 CANopen 变体是 `alfa_v1`，生产映射保持 Node 2/3、mode 3
  和既有 joint/interface ABI；未知变体必须 fail closed。
- F3: `Cia402System` 的电机集合来自严格校验后的 `HardwareInfo`，插件不再把生产
  Node 2/3 写死；这不等于任意节点数已获得生命周期或实机准入。

## 遗留

- BQ-027 的驱动侧失联反应实机证明与 BQ-132 的现场无报文前置条件仍阻塞相应
  powered 验证，不因配置解耦而解除。
- 引入首个非两节点 CANopen 生产变体前，必须补完整 activate 部分失败、rollback、
  deactivate、EMCY/失联 fault injection 和 HIL 证据。
- CANopen 当前只准入 Profiled Velocity mode 3。新 profile 至少需要 EDS/bus/profile
  准入与 descriptor 注册；修改 Node ID 仍需同时修改 descriptor 和 `bus.yml`。
