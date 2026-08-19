---
id: release-deploy-20260819-01
area: release-deploy
title: Franka 风格双硬件插件组合入口
date: 2026-08-19
type: feature
trigger: ELECTRI-94
commits: [feature/rt-control-硬件配置分层组合]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [final image rt-control:electri-94-franka-layering-final-bbe5 sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4 with 29 packages, ros2_canopen 0005 SHA-256 bbe5fddd32760e06eda7c558984647990fbe5bbfb629753e3bebeeaafe315b0a, pristine upstream CANopen 8-package build and 178 tests, focused Python 165/165, quality_gate 196/196 at 83%, Mock container a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580 exit 0]
supersedes: []
related: [BQ-027, BQ-122, BQ-132, ecat-axes-20260819-01, canopen-chassis-20260819-01, lifecycle-20260819-03, contract-20260819-06]
---

## 背景

ELECTRI-94 放弃此前 Phase 2 generator 路线，改为仿照 Franka 的分层、组合方式和
semantic components，但保留多设备系统：统一 bringup 在一个 controller_manager
中组合独立的 EtherCAT `ecat_arms` 与 CANopen `canopen_mobile_axes` 插件。

## 改动

- `rt_control_bringup` 的总装 Xacro 只 include Robot Model 与两个硬件包的公开宏，
  按 `ethercat_variant`/`canopen_variant` 组合两个 `<ros2_control>` system；默认和当前
  唯一已提供的 descriptor 均为 `alfa_v1`，未知值在 Node 创建前或 Xacro 展开阶段
  fail closed。
- bringup 自有 `mock.ros2_control.xacro` 已删除；real/mock schema 由对应硬件包共同
  维护。一个 controller_manager、250 Hz update rate、稳定 component/controller 名称、
  启动顺序和安装后 `rt_control_start` 入口保持不变。
- 两个 hardware package 分别以 `variants/<name>.yaml` 注册本总线 topology。公开
  Xacro 从 descriptor 生成 real/mock system 与 `HardwareInfo`；bringup 在启动任何
  Node 前加载同一对 selected descriptors，派生 diagnostics joint/ring/node 参数，不再
  维护 diagnostics-only composition YAML。
- controller 配置显式提供 enable-manager 的 managed joints、批次、时序和终态策略；
  缺失时 fail closed，不从 descriptor 自动猜测。alignment tests 检查 Robot Model、
  descriptors、profile/bus、控制器和安全 policy，防止多 owner 配置静默漂移。
- Dockerfile 接入新的 EtherCAT/CANopen pinned patches；Native bootstrap 与 IPC/Native
  axis-state checker 同步使用安装/源码中的 controller 配置。
- `.dockerignore` 排除 `.git`、本地 `build/install/log`、已导入的 `src/vendor`、Python
  cache/coverage/pytest 产物以及根级 Markdown/`docs`，防止这些宿主构建与证据产物
  混入 Docker build context；冻结 vendor 仍由镜像构建中的 `deps.repos` 重新导入。
- 本路线不建立 strict SSOT：Robot Model 继续拥有 joint 名称/类型，硬件包拥有总线
  局部 descriptor/Xacro/config/profile，enable-manager 拥有生命周期策略，bringup
  只负责选择、组合和派生 diagnostics topology。
- 新增长期维护指南
  `domains/rt_control/docs/hardware-configuration-layering-and-motor-variants.md`，明确区分
  “通用驱动不写死机型拓扑”与“所有配置一文件化”，并给出增删电机、换算和
  CSP/CSV/CST 的分层修改与验证流程。

## 验证

- 最终 `ros2_canopen` 0005 补丁 SHA-256 为
  `bbe5fddd32760e06eda7c558984647990fbe5bbfb629753e3bebeeaafe315b0a`；在冻结上游的
  pristine 工作树完成 8-package 构建并通过 178 项测试。focused Python 165/165、
  `tools/quality_gate.sh` 196/196（门禁覆盖率 83%）通过。
- 最终镜像 `rt-control:electri-94-franka-layering-final-bbe5`（ID
  `sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4`）成功构建
  29 个包。镜像入口仍为安装后的 `rt_control_start`。
- 精确该镜像的无设备 Mock 容器
  `a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580` 在
  `network=none`、private IPC、无 device/capability/bind 的条件下得到预期 controller
  状态；controller manager 为 250 Hz，SafetyState 保持 `NOT_READY` 并 fail closed。
  停机按 BQ-122 的 `/rt/disable -> controllers quiesced -> ecat_arms inactive ->
  SIGINT -> CAN cleanup` 顺序完成，容器退出 0，日志未出现 `ERROR`、`FATAL`、
  `UNCLEAN_SHUTDOWN` 或 `SIGSEGV`。
- 本记录的 `PASS/T1` 只证明精确最终镜像的构建、无设备 Mock 组合与 clean shutdown；
  未部署或启动生产 Compose，未访问总线/设备，未执行 HIL、reset、enable、运动或 PLC
  输出。本机没有 ShellCheck，仍由 CI 补跑。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 统一 bringup 在一个 controller_manager 内组合两个独立硬件 system：
  `ecat_arms` 与 `canopen_mobile_axes`；功能包不得反向依赖 bringup。
- F2: 当前交付 seam 只允许 `alfa_v1`，并保持相同 logical joint/interface ABI；本次
  不承诺任意电机数量、任意电机类型或未联动修改控制器/模型的热切换。
- F3: 配置采用 owner-local registries 加 alignment tests，不采用 strict SSOT；硬件
  Xacro 与 diagnostics topology 消费同一对 selected descriptors，不再有独立
  composition YAML。相同 logical ABI 的已认证 profile 可做 descriptor 映射；新增
  profile、logical joint 或 mode 仍按 owner 扩大变更范围。
- F4: BQ-122 的停机顺序和 30 s 总 deadline 完全不变；任何新变体都必须复用并验证
  `/rt/disable -> controllers quiesced -> ecat_arms inactive -> SIGINT -> CAN cleanup`。

## 遗留

- 最终 Docker 镜像构建、无设备 Mock 启动/停止和 BQ-122 日志证据已经补齐；CI 仍需
  补本机缺失的 ShellCheck。该镜像 ID 是本次不可变证据，但当前 Docker recipe 的
  `ros:humble-ros-base` 基础镜像仍按 tag 引用，apt/rosdep 包也未锁到 snapshot/逐包
  版本，因此未来重建不能宣称字节级可复现；发布时必须记录并使用本次精确 image ID，
  后续另行完成基础镜像 digest 与系统依赖冻结。
- BQ-027、BQ-132 仍限制 powered CANopen 验证；首个非两节点 CANopen 变体还需专门
  lifecycle fault injection/HIL。不得把本地测试写成实机验收。
- 当前 EtherCAT 只准入 CSP 8，CANopen 只准入 Profiled Velocity 3；CSV/CST 或新的
  CANopen mode 需要完整 profile/interface/controller/safety bundle，不能只改 selector
  或 mode 数字。
