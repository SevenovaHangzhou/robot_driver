---
id: lifecycle-20260819-03
area: lifecycle
title: CiA402 semantic component 与使能策略分层
date: 2026-08-19
type: feature
trigger: ELECTRI-94
commits: [feature/rt-control-硬件配置分层组合]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [final image rt-control:electri-94-franka-layering-final-bbe5 sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4 with 29 packages, focused Python 165/165, quality_gate 196/196 at 83%, Mock container a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580 BQ-122 order and exit 0]
supersedes: []
related: [BQ-122, ecat-axes-20260819-01, canopen-chassis-20260819-01, contract-20260819-06, release-deploy-20260819-01]
---

> 历史说明：本记录来自当时未合并的 `67ca53b` 分支及其候选镜像，只保存原始 T1
> 证据。当前 main 移植、测试数和镜像身份由 `release-deploy-20260903-01` 取代。

## 背景

ELECTRI-94 采用 Franka 风格的 semantic components，但不复制其单设备假设。
CiA402 接口绑定与状态解释应成为无机型策略的可复用层；14 轴批次、时序和 Ti5
失能终态仍属于 `enable_manager` 的整机生命周期策略。

## 改动

- 新增 header-only `rt_control_semantic_components::Cia402Axis`：按完整资源名精确
  绑定一个 `control_word`/`status_word` loaned interface，拒绝缺失或重复资源，支持
  显式释放，严格把有限、非负、整数范围内的 double 转为 `uint16_t`，并统一解码
  八个标准 CiA402 状态及 unknown。该类型不保存拓扑、批次、时序或总线生命周期。
- `enable_manager` 改用 semantic axes，并由 `controllers.yaml` 提供
  `managed_joints`、扁平批次 joint 列表、batch sizes 与允许
  ReadyToSwitchOn 作为失能终态的 joint 集合。配置阶段要求非空唯一、1..127 个轴/
  批次、每批 1..3 轴、所有轴恰好分区一次；这些 topology/safety 参数的声明默认值
  均为空，任何缺失或拼写错误都会拒绝 configure，不再隐式回退到 `alfa_v1` 轴表。
  配置成功后拓扑参数不可在线修改。
- 五个时序浮点参数新增 NaN/Inf 拒绝，原有正值/非负约束不变；250 Hz update 只读
  配置阶段冻结的 vector，不在周期内查参数、扩容或加锁。
- recovery axis-state checker 从同一 `controllers.yaml` 读取 managed joints 与 Ti5
  终态策略；descriptor 只注册硬件，不自动生成或替代 lifecycle safety policy。

## 验证

- 最终工作树的 focused Python 165/165、`tools/quality_gate.sh` 196/196（门禁覆盖率
  83%）通过。最终镜像 `rt-control:electri-94-franka-layering-final-bbe5`（ID
  `sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4`）成功构建
  29 个包。
- 精确该镜像的无设备 Mock 容器
  `a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580` 在
  `network=none`、private IPC、无 device/capability/bind 的条件下得到预期 controller
  状态：`joint_state_broadcaster`、`rt_internal_state_broadcaster`、
  `diff_drive_controller` 和 `enable_manager` active，`whole_body_jtc` inactive；
  controller manager 为 250 Hz，SafetyState 保持 `NOT_READY` 并 fail closed。
- 停机日志按 BQ-122 依次记录 `/rt/disable` 成功、controllers quiesced、
  `ecat_arms` inactive、SIGINT，随后 CAN cleanup；容器退出 0，日志未出现 `ERROR`、
  `FATAL`、`UNCLEAN_SHUTDOWN` 或 `SIGSEGV`。
- 本记录的 `PASS/T1` 只覆盖最终镜像内的无设备 Mock 生命周期和停机顺序。未执行
  HIL 或实机 reset/enable/disable、fault/timeout、掉电恢复；未访问总线或设备，未执行
  reset、enable、运动或 PLC 输出。本机没有 ShellCheck，仍由 CI 补跑。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `Cia402Axis` 只拥有类型化接口绑定、读写和状态解码；不得拥有机型拓扑、批次、
  时序、Ti5 策略或硬件生命周期。
- F2: `enable_manager` 拥有 managed joints、批次、时序和允许 ReadyToSwitchOn 的失能
  终态策略；拓扑参数没有机型默认值，必须显式配置，并在 configure 阶段完整校验和
  冻结后供 250 Hz 路径使用。
- F3: 本次参数化不改变 BQ-122：退出仍必须遵循 `/rt/disable -> controllers
  quiesced -> ecat_arms inactive -> SIGINT -> CAN cleanup`。

## 遗留

- 最终镜像的无设备 Mock 已复核预期 controller 状态和 BQ-122 停机顺序；真实 reset、
  enable、fault/timeout 和掉电恢复仍需单独授权。
- 当前 `alfa_v1` 的控制器与 Robot Model 仍固定相同 logical joint/interface ABI；改变
  电机数量需要同时评审 controller/safety、模型与生命周期；诊断 topology 可从
  descriptor 派生，但这不代表其他 owner 可省略。
