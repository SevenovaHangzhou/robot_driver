---
id: ecat-axes-20260906-01
area: ecat-axes
title: ELECTRI-116 X503B raw shadow采集与只读单位元数据桥
date: 2026-09-06
type: feature
trigger: ELECTRI-116
commits: [feat/electri-116-x503b-wrench]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no, sdo: no }
verified: PARTIAL
evidence:
  - "x503_force_sensor package build and colcon test-result: 10 tests, 0 failures"
  - "tools/quality_gate.sh: 214 passed"
  - "robot_hw_ethercat/rt_control_bringup hardware composition contracts: 71 passed"
supersedes: []
related: [ecat-axes-20260902-01]
---

## 背景

当前 main 已将两台 X503B 作为 position 14/15 的 state-only EtherCAT sensors 接入，
但只提供 raw PDO state。ELECTRI-116 需要在不进入运动控制链的前提下，为上层提供采集
影子数据，并为后续工程单位 WrenchStamped 保留最小只读单位/小数位快照入口。

## 改动

- 新增 `x503_force_sensor` 非实时 bridge：从同一条
  `/rt_internal_state_broadcaster/dynamic_joint_states` 帧提取六个 raw DINT 和六个
  sample-code，发布左右 raw `Int32MultiArray` 影子话题。
- 新增只读 `x503_sdo_snapshot` C++ 节点；唯一读取 `0x8005:06~11` 小数位和
  `0x8005:12~17` 单位码，使用 EtherCAT manager 的 read-only ioctl API；不包含 SDO
  download、置零或参数存储路径。
- 单一硬件配置 `robot_hw_ethercat/config/x503b_readback.yaml` 保存对象索引、子索引和
  期望单位码；当前 `validity_policy: unresolved`，因此未确认有效位语义前不会发布
  有效 WrenchStamped。
- 启动编排只在选择了 X503 sensors 时加入 bridge；SDO snapshot 默认关闭，避免 mock
  或普通离线启动访问 EtherCAT。现有 PDO、拓扑、JTC、enable_manager 和 Robot Model
  未修改。

## 验证

- `x503_force_sensor` package build：PASS；C++ snapshot executable 与 Python bridge 安装成功。
- `colcon test --packages-select x503_force_sensor`：10 tests，0 failures。
- X503 bridge pure tests：raw frame 完整性、非整数拒绝、单位/小数转换、错误单位拒绝、
  未完成 readback 时禁止 Wrench：PASS。
- `rt_control_bringup` hardware composition contracts：71 passed。
- `tools/quality_gate.sh`：214 passed；`git diff --check`、shell/Python 静态检查通过。
- 未启动 EtherCAT、未读取现场 SDO、未发布真实 Wrench、未 reset/enable/运动/PLC 写入。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: X503 raw shadow 输出不打开 EtherCAT 网卡、不调用 `ethercat upload`、不提供任何
  控制或 SDO 写入口。
- F2: 最小单位元数据只读集合固定为 `0x8005:06~11` 和 `0x8005:12~17`；其它校准、
  置零、存储对象不属于本阶段。
- F3: `WrenchStamped` 只有在六个 raw 通道完整、单位码/小数位有效且 validity policy
  明确后才允许发布；当前默认策略为 unresolved，raw shadow 可独立运行。

## 遗留

- 目标机只读 SDO/身份/PDO 验证尚未执行；需单独维护窗口授权。
- X503 sample-code 的有效性语义和左右传感器 TF 仍需硬件/模型 owner 确认；确认后把
  `validity_policy` 从 `unresolved` 改为批准策略，才可打开工程量 Wrench 输出。
- Docker/完整 main CI 尚未完成；当前实现不改变正在运行的 Native release。
