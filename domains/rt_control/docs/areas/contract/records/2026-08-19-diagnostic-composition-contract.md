---
id: contract-20260819-06
area: contract
title: 诊断拓扑组合与稳定摘要契约
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
related: [ecat-axes-20260819-01, canopen-chassis-20260819-01, lifecycle-20260819-03, release-deploy-20260819-01]
---

## 背景

硬件包拥有总线局部配置后，诊断仍需要知道本次总装预期看到哪些 EtherCAT 轴、
responder 和 CANopen 节点。为避免再维护一份机械清单，诊断 topology 应从本次实际
选择的 hardware variant descriptor 派生；它不能反向生成 Robot Model、controller
或生命周期安全策略。

## 改动

- `robot_hw_ethercat/variants/<name>.yaml` 与
  `robot_hw_canopen/variants/<name>.yaml` 是两个 hardware owner 的 schema v1 注册表。
  bringup 根据 launch selector 在创建任何 Node 之前加载所选 descriptor，拒绝非法
  variant 名、重复 YAML key、未知/缺失字段、类型/范围错误、重复 joint/ring/node、
  不完整 EtherCAT ring 和跨总线 joint 重名。
- bringup 从 EtherCAT `axes + extra_responders` 派生 expected responders，并只把
  EtherCAT joint/ring、expected responders 与 CANopen node IDs 四个只读参数传给
  `rt_diagnostics`。仓库不再保存独立的 diagnostics composition YAML。
- `rt_diagnostics` 按配置生成详细 master/slave/node status，并新增稳定摘要
  `/robot/rt_control/ethercat/summary` 与 `/robot/rt_control/canopen/summary`。摘要选择
  最高严重级，平级按 source/message 确定排序，同时保留首个具体故障来源和原消息。
- `control_api_adapter` 只消费 enable-manager、两条总线摘要、PLC 与 BMS，不再复制
  14 个 EtherCAT 和两个 CANopen 拓扑数组；回调组改为 mutually exclusive。
- DynamicJointState 改为整帧替换；joint/interface 重复或长度不匹配会清空旧快照并
  标记 stale，避免旧值 fail open。诊断 `hardware_id` 非空且不得与配置的 CAN node ID
  相同，防止自身归一化输出被再次当作原生 CAN 输入。
- descriptor 同时被 hardware package Xacro 消费，因此诊断和硬件注册共享同一机械
  topology；controller/enable policy 仍由 owner 显式维护，不从 descriptor 推断。
- 没有修改公共 `robot_interfaces` schema、域内 msg/srv 或跨域 topic 名称。

## 验证

- 最终工作树的 focused Python 165/165、`tools/quality_gate.sh` 196/196（门禁覆盖率
  83%）通过。最终镜像 `rt-control:electri-94-franka-layering-final-bbe5`（ID
  `sha256:5a61aa08a6f9b807f32cb6fc5829193f0f0e0060de938e36e128532f2d2d64f4`）成功构建
  29 个包，取代此前仅能说明中间状态的旧测试计数。
- 精确该镜像的无设备 Mock 容器
  `a706422f1a3aec4899265c17009f31bca4e1f4046dc978793f15482cf3fae580` 在
  `network=none`、private IPC、无 device/capability/bind 的条件下得到预期 controller
  状态与派生诊断链；controller manager 为 250 Hz，SafetyState 保持 `NOT_READY` 并
  fail closed。BQ-122 顺序成立，容器退出 0，日志未出现 `ERROR`、`FATAL`、
  `UNCLEAN_SHUTDOWN` 或 `SIGSEGV`。
- 本记录的 `PASS/T1` 只覆盖最终镜像内的诊断组合契约与无设备 Mock；`NOT_READY`
  是没有硬件时的预期保守结果，不是 DomainReadiness/实机 READY 证明。未执行 HIL、
  总线故障注入或实机运行；未访问总线/设备，未执行 reset、enable、运动或 PLC 输出。
  本机没有 ShellCheck，仍由 CI 补跑。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: diagnostics topology 由本次选择的 EtherCAT/CANopen owner-local descriptors
  派生，不存在独立的 composition 配置；descriptor 不是 Robot Model、controller 或
  lifecycle safety policy 的 strict SSOT。
- F2: `rt_diagnostics` 负责把动态详细拓扑归一化成两个稳定总线摘要；公共状态适配器
  只消费摘要，不持有 EtherCAT/CANopen 数量或 ID 列表。
- F3: 详细 diagnostic status 保留，错误输入必须 fail stale；摘要必须确定性携带最高
  严重级和具体来源，且不得通过自身 `hardware_id` 形成反馈回路。

## 遗留

- 最终无设备 Mock 已确认两条总线摘要参与 SafetyState/DomainReadiness 的 fail-closed
  消费链；真实总线 publisher 时效、故障详情和 READY 转换仍需 HIL/实机确认。
- 当前多份 owner-local 配置依靠 alignment contract tests 防漂移；若未来允许不同
  logical ABI，必须同时修改 Robot Model、descriptor、控制器和安全策略，诊断预期则
  由新 descriptor 自动派生。
