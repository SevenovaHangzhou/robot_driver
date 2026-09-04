---
id: ecat-axes-20260819-01
area: ecat-axes
title: EtherCAT 硬件包接管变体 Xacro 与 HardwareInfo 拓扑
date: 2026-08-19
type: feature
trigger: ELECTRI-94
commits: [feature/rt-control-硬件配置分层组合]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [final image rt-control:electri-94-franka-layering-final-bbe5 sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4 with 29 packages, focused Python 165/165, quality_gate 196/196 at 83%, Mock container a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580 exit 0]
supersedes: []
related: [BQ-122, canopen-chassis-20260819-01, lifecycle-20260819-03, contract-20260819-06, release-deploy-20260819-01]
---

> 历史说明：本记录来自当时未合并的 `67ca53b` 分支，所述 16 位环已被 PR #16 的
> 18 位拓扑取代。当前实现与证据以 `release-deploy-20260903-01` 为准。

## 背景

ELECTRI-94 要求按 Franka 的分层与组合方式解耦硬件配置，同时保留 EtherCAT 与
CANopen 两个独立硬件插件。EtherCAT 总线局部事实因此应由 `robot_hw_ethercat`
拥有，而不是由 bringup 维护另一份 real/mock schema。

## 改动

- `robot_hw_ethercat` 公开 `rt_control_ethercat_system` 分发宏，并以
  `variants/alfa_v1.yaml` 注册 system、extra responders 及 14 个轴的 joint/family/
  ring/profile/mode。公开宏加载所选 descriptor，用同一轴表生成 real/mock joints 与
  EtherCAT slave diagnostic sensors；bringup 只传 variant selector 并调用该宏。
- `alfa_v1` 保持 16 个 responder、轴环位 `1..12, 14, 15`、CiA402 mode 8、既有
  PDO/SDO profile、比例、方向和 joint/interface ABI。未知变体通过 `xacro.fatal`
  fail closed；本次不承诺任意电机数量可直接替换。
- package 构建门禁逐份严格校验 descriptor schema、完整 0-based ring、family/profile、
  当前认证 mode，以及 slave profile 的 SDO `0x6060` 和 RPDO `0x6060` default 是否与
  descriptor mode 一致。当前每个 family 仅认证 CSP 8，CSV/CST 会 fail closed。
- pinned `ecat_icube` 补丁改从 ros2_control `HardwareInfo::ComponentInfo.parameters`
  读取 `ec_module.*`，不再重新解析 `original_xml`；缺项、非法无符号数、不支持的
  mode、重复 `alias:position` 或插件加载失败均在初始化阶段拒绝。
- Mock 与真实分支由同一硬件包维护，ZeroErr 的 digital IO 与全部状态/命令接口保持
  对齐；Robot Model 仍是 joint 名称和类型的权威源。

## 验证

- focused Python 165/165、`tools/quality_gate.sh` 196/196（门禁覆盖率 83%）通过。
  最终镜像 `rt-control:electri-94-franka-layering-final-bbe5`（ID
  `sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4`）应用冻结的
  EtherCAT patch chain，并成功构建 29 个包。
- 精确该镜像的无设备 Mock 容器
  `a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580` 在
  `network=none`、private IPC、无 device/capability/bind 的条件下得到预期 controller
  状态，controller manager 为 250 Hz，SafetyState 保持 `NOT_READY` 并 fail closed；
  BQ-122 顺序成立，容器退出 0，日志未出现 `ERROR`、`FATAL`、
  `UNCLEAN_SHUTDOWN` 或 `SIGSEGV`。
- 本记录的 `PASS/T1` 只覆盖最终镜像的软件构建、配置契约和无设备 Mock。未执行
  HIL 或实机 EtherCAT 生命周期、PDO/SDO、方向/比例验证；未访问总线或设备，未执行
  reset、enable、运动或 PLC 输出。本机没有 ShellCheck，仍由 CI 补跑。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: EtherCAT descriptor 是轴/family/ring/profile/mode 的 owner-local 注册点；同一
  descriptor 驱动 real/mock ros2_control schema、slave sensors 和 diagnostics 派生，
  bringup 只选择并组合公开系统宏。
- F2: 当前唯一允许的 EtherCAT 变体是 `alfa_v1`；其 14 轴 joint/interface ABI、
  CSP mode 8、环位与既有比例/方向保持不变，未知变体、未认证 mode 和 profile/mode
  漂移必须 fail closed。
- F3: EtherCAT 插件从 `HardwareInfo` 读取并严格校验 `ec_module.*`；不得以重新扫描
  整份 URDF 的方式跨越硬件组件边界。

## 遗留

- 最终 Docker 构建、无设备 Mock 启停和 BQ-122 顺序已由
  `release-deploy-20260819-01` 收口；本记录不改变 BQ-122 的既有退出契约。
- 实机 PDO/SDO、方向、比例和生命周期证据仍需逐项授权后取得。
- 相同 logical/interface ABI 下可把轴映射到已认证 profile；新 profile 仍需 profile +
  descriptor，增删 logical joint 仍需 Robot Model 与 controller/safety owner 联动。
