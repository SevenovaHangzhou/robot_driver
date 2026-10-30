---
id: release-deploy-20260903-01
area: release-deploy
title: 将 ELECTRI-94 硬件配置分层移植到双 X503 main
date: 2026-09-03
type: feature
trigger: 用户要求保留未合并分支 feature/rt-control-硬件配置分层组合 的改动并移植到最新 main
commits: [feat/electri-94-hardware-composition-main]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - "tools/quality_gate.sh：206/206 PASS，门禁覆盖率 83%"
  - "ecat_icube 1390be7 补丁 0001..0006 顺序 apply-check：PASS"
  - "ros2_canopen fef50e5 补丁 0001..0005 顺序 apply-check：PASS"
  - "镜像内补丁影响的 7 个 ecat_icube/ros2_canopen 上游包：200 tests，0 failures"
  - "完整 Docker 构建：29 packages PASS；image sha256:08d3705b163a6dbe7c6c4ed0031935108d889f8aec5823e0dc511ac065fe29d6"
  - "镜像内 7 个受影响包 BUILD_TESTING=ON：365 tests，0 failures"
  - "无网络/无设备/无 capability/无 bind Mock 容器 9a594e96ad4ce71171954855125d0cea7b78b6a74349750b194f1dfbf5fdcc3a：exit 0"
supersedes: [canopen-chassis-20260819-01, contract-20260819-06, ecat-axes-20260819-01, lifecycle-20260819-03, release-deploy-20260819-01]
related: [ecat-axes-20260902-01, ELECTRI-94]
---

## 背景

ELECTRI-94 的硬件配置分层只存在于未合并提交 `67ca53b`，与当时 main 的 merge base
为 `9d97657`。当前 main 已前进到 `b80f6c1`，并通过 PR #16 把两台 X503 加入 18 位
EtherCAT 拓扑。旧提交不能直接 cherry-pick，否则会把 Turn/Updown 从 16/17 退回
14/15，并覆盖之后的接口、ROS Domain、Robot Model、PCIe CAN、固定 PDO 与部署修复。

## 改动

- 从最新 main 新建 `feat/electri-94-hardware-composition-main`，按语义移植而不改写旧
  远端分支历史。
- EtherCAT owner-local descriptor 增加独立 `sensors` 层：14 个运动轴仍为
  `1..12,16,17`，右/左 X503 为 state-only sensor `14/15`，Hub `0/13` 仍是
  `extra_responders`，合计 18 个 responder。X503 不获得 command interface，不进入
  JTC、enable batch 或公共 `/joint_states`。
- EtherCAT family registry、严格 validator 与 real/mock Xacro 从同一 descriptor 生成
  joint、X503 raw state、master/domain 和 slave AL interface；CANopen 继续由独立
  `alfa_v1` descriptor 与 `bus.yml` 对齐生成 Node 2/3。
- bringup 只选择并组合两个 hardware package 宏，在创建 Node 前校验 descriptor 与
  controllers；删除 bringup 自有的重复 mock hardware schema。controller spawner 采用
  串行、失败即停止的链。
- 新增 header-only `Cia402Axis` semantic component。enable_manager 的 managed joints、
  批次和 Ti5 ReadyToSwitchOn 终态例外由 `controllers.yaml` 显式配置并在 configure
  阶段冻结；250 Hz update 不读取 YAML 或动态参数。
- diagnostics 的 EtherCAT 轴、X503 sensor、responder 数和 CANopen node 均从所选
  descriptors 派生，详细状态汇总为稳定 EtherCAT/CANopen summary；公共 status adapter
  消费 summary，不再持有固定电机数组。公共消息、ErrorInfo 与 endpoint 未修改。
- 保留 PR #16 的 ecat_icube fixed-PDO `0004`，把旧 HardwareInfo 补丁重编号为
  `0005/0006`；ros2_canopen topology 补丁保持 `0005`。Docker、native bootstrap、IPC/
  native launcher 和策略门禁使用相同顺序。
- 保留当前 main 的双 X503 profile、ROS Domain、PCIe CAN、轮距/限速、`lidar_main`
  Robot Model 和接口 pin。

## 验证

- 测试先行：composition import、EtherCAT validator、CANopen validator、semantic
  header、enable-manager enum、diagnostics topology、status summary 及发布脚本均先得到
  与缺失实现对应的 RED，再在同一目标上转为 GREEN。
- `tools/quality_gate.sh`：206 项测试全部通过，门禁覆盖率 83%；本机无 ShellCheck，仍由
  CI 强制执行。
- 两个冻结上游分别从 `deps.repos` 精确 SHA 建立临时 detached worktree；ecat_icube
  `0001..0006` 与 ros2_canopen `0001..0005` 均逐个通过 `git apply --check` 并顺序应用。
- 在最终镜像中以 `BUILD_TESTING=ON` 重新构建补丁影响的 7 个 ecat_icube/ros2_canopen
  上游包，200 项测试全部通过。
- 完整 Docker 构建完成 29 个包，候选镜像 ID 为
  `sha256:08d3705b163a6dbe7c6c4ed0031935108d889f8aec5823e0dc511ac065fe29d6`。该镜像内
  使用独立 build/install/log 根重新以 `BUILD_TESTING=ON` 构建 7 个受影响包，365 项
  测试全部通过。
- Mock 容器使用 `network=none`、private IPC，inspect 确认 devices 为空、capability
  为空、bind 为空。两个 hardware system、14 轴、两台 X503、derived summaries 与
  `lidar_main` 均加载；无 CAN/PLC/BMS 时 SafetyState 保持 `NOT_READY`。停止返回
  `/rt/disable already_disabled`，随后 controllers quiesced、EtherCAT inactive，容器
  exit 0。
- 候选 tag 含 `worktree`，只作为未提交代码的 T1 证据，不是可发布的 commit-locked
  镜像。未访问目标工控机、EtherCAT/CAN 设备或 PLC，未执行 reset、enable 或运动。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前 `alfa_v1` EtherCAT descriptor 明确分离 `axes`、state-only `sensors` 与
  `extra_responders`，组成 PR #16 的 18 位拓扑；运动集合仍严格为 14 轴。
- F2: `rt_control_bringup` 只拥有 variant 选择、双 system 组合、controller 对齐和启动
  顺序；real/mock 资源 schema 归各 hardware package 所有。
- F3: diagnostics topology 从所选 descriptors 派生，公共 status adapter 只消费稳定
  summary；descriptor 不取代 Robot Model 或 enable-manager safety policy。
- F4: ecat_icube 补丁顺序固定为 `0001..0006`，其中 PR #16 fixed-PDO 保护必须先于
  HardwareInfo 参数解析/校验；ros2_canopen 补丁顺序固定为 `0001..0005`。
- F5: 本移植不改变公共接口、Robot Model、运动轴名称、控制模式、比例、方向、限位或
  使能批次语义。
- F6: CANopen `alfa_v1` descriptor 是 Node 2/3、Profiled Velocity mode 3、side 与
  profile 的 owner-local 注册点，必须与同包 `bus.yml` 完全对齐；生成 bin 清单和
  `Cia402System` 生命周期拓扑不得写死左右节点。
- F7: `Cia402Axis` 只负责类型化 control/status interface 绑定和状态解码；managed
  joints、批次、时序和 Ti5 失能终态仍由 enable_manager 显式配置拥有。

## 遗留

- 需要提交后由远端 CI 补 ShellCheck、完整 governance/build，并以最终 commit SHA
  重建镜像；当前 worktree 镜像不能直接发布或部署。
- PR #16 的 18 位拓扑仍缺正式的不使能实机 OP/WC/X503 raw 与有序停机验收；本次
  Mock/Docker 结果不能替代该 T2/T3 证据。
- 未执行 HIL、故障注入、reset、enable、运动或 PLC 输出。
