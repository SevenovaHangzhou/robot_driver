---
id: ecat-axes-20260902-01
area: ecat-axes
title: 双 X503 接入 18 位运行拓扑
date: 2026-09-02
type: feature
trigger: 用户要求两台 X503 串联接入 position 13 的第二个 Hub，并修改运行拓扑
commits: [6轴+力传感器]
env: both
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - "X503 profile/topology pytest：7/7 PASS"
  - "robot_hw_ethercat 与 rt_diagnostics 本机构建：PASS"
  - "robot_hw_ethercat/rt_diagnostics colcon test-result：15 tests，0 failures"
  - "tools/quality_gate.sh：200/200 PASS，策略覆盖率 83%"
  - "IgH pinned source patch apply-check：PASS"
  - "完整 Docker 构建：28 packages PASS；image sha256:f4b7359acc6bdb7f4a79f36d2a34a7d4010901b66392e7eab54540e2268030d4"
  - "无网络/无设备临时容器内安装产物与 18 位 Xacro smoke：PASS"
  - "无网络/无设备 Mock 运行时 dynamic_joint_states 的双 X503 raw 接口：PASS"
supersedes: []
related: []
---

## 背景

第二个 Hub OUT8 后按右、左顺序串联两台 X503。运行配置原为 16 位：position 14/15
分别是 Turn/Updown；因此需要把两台传感器加入运行链，并把两个原运动轴后移，而不能只
增加孤立 YAML。

## 改动

- 新增 `x503_right.yaml` 与 `x503_left.yaml`，分别绑定 position 14/15 和串联上下游。
  两者使用实机 identity `0x00000503/0x26483052@0x00020111`、fixed RxPDO `0x1601`
  及 TxPDO `0x1A00` 25×DINT，只暴露 6 路 raw 与 6 路 sample-code 状态。
- 运行 Xacro 改为 18 位：运动轴仍为 1..12、16、17；X503 为 14、15；Hub 0、13
  不注册运动接口。Mock 和 diagnostics 同步增加两台传感器状态。
- ecat_icube 增加 `use_slave_pdo_defaults` 适配，IgH 增加 `PreservePdoConfig`：启动时把
  期望映射与实机映射逐项比较，不一致则失败，且不写 CoE PDO 映射对象。
- 宿主安装、native/IPC 启动器和 Docker 镜像均校验补丁哈希；启动前强制核对 18 个
  position、两台 Hub、X503 identity 及 8/25 个 PDO 条目。
- 新增自动测试锁定 identity、PDO 尺寸、导出接口、左右顺序、位置和保护补丁链。

## 验证

已完成 X503 聚焦测试 7/7、`robot_hw_ethercat` 与 `rt_diagnostics` 构建、两个包累计
15 项 colcon 测试，以及 200 项仓库质量门禁。固定 IgH commit 上的补丁 apply-check
通过。完整 Docker 构建完成 28 个包；无网络、无设备挂载的临时容器确认安装后的两份
X503 profile、14/15/16/17 位置和 IgH 补丁哈希均正确。
Mock 控制栈的内部 dynamic joint state 快照包含右/左 X503 与 raw channel 接口。

未启动控制容器或 EtherCAT 主站，未进行当前源码的 18 设备实机 OP/WC/raw 数据验证。
`rt_control_bringup` 的单独宿主构建因其余包未在本工作区安装而失败；完整依赖闭包由
Docker 构建验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前运行拓扑固定为 `Hub 0 -> motion 1..12 -> Hub 13`，其中
  `Hub 13 OUT8 -> right_force_sensor(position 14) -> left_force_sensor(position 15)`；
  `turn=16`、`updown=17`，共 18 个 EtherCAT position。
- F2: X503 fixed PDO 使用 Rx `0x1601` 16 bits、Tx `0x1A00` 25×DINT/100 bytes；运行时
  必须同时具备 ecat/IgH 保留并核对固定PDO的支持，不允许以写映射修复。
- F3: EtherCAT 运动集合仍严格为 14 轴（1..12、16、17）；X503 仅导出 raw state
  interface，不提供 command interface，不进入使能批次、JTC 或 `/joint_states`。

## 遗留

尚需在现场保持执行器不使能，完成 18 position 身份/顺序复核、两台 X503 进入 OP、
domain WC complete 且稳定、12 路 raw 状态可读及有序停机回到 18×PREOP。通道物理语义、
小数点、工程单位和六维解耦矩阵仍未确认，因此本次只发布 raw 数据，不得对外宣称力/力矩值。
