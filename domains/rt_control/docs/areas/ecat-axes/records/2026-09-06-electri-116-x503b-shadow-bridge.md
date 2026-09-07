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
  - "user-supplied X503-B V1.6 protocol PDF SHA-256: 39432f1304b68af1839a3553b4581da92a3923306945e5985cdbae7d7689ae0c"
  - "x503_force_sensor package build and colcon test: 2 test targets / 14 cases, 0 failures"
  - "affected native-source build: 29 packages finished"
  - "affected colcon test-result: 1439 tests, 0 errors, 0 failures, 18 skipped"
  - "tools/quality_gate.sh: 214 passed"
  - "robot_hw_ethercat variant tests: 34 passed; rt_control_bringup composition tests: 46 passed"
supersedes: []
related: [ecat-axes-20260902-01, BQ-143]
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
  工程单位确认状态；依据用户提供的 X503-B V1.6 手册固定单位码语义（5=N、7=N·m）
  和期望通道元组 `[5,5,5,7,7,7]`，运行时仍必须用只读 SDO 实际回读验证。当前
  `validity_policy: sample_codes_in_range`，但实际回读仍是发布前置条件。
- 启动编排只在选择了 X503 sensors 时加入 bridge；SDO snapshot 默认关闭，避免 mock
  或普通离线启动访问 EtherCAT。现有 PDO、拓扑、JTC、enable_manager 和 Robot Model
  未修改。

## 验证

- `x503_force_sensor` package build：PASS；C++ snapshot executable 与 Python bridge 安装成功。
- `colcon test --packages-select x503_force_sensor`：2 test targets / 14 cases，0 failures。
- 受影响 native-source build：29 packages finished，0 failures。
- 受影响全量 `colcon test-result`：1439 tests，0 errors，0 failures，18 skipped；Mock launch
  停机时 bridge cleanly exit。
- 无设备 SDO snapshot smoke：读取配置文件成功；`/dev/EtherCAT0` 缺失时在有界重试后退出，
  未调用任何 SDO write 路径。
- X503 bridge pure tests：raw frame 完整性、signed DINT 边界、非整数拒绝、单位/小数转换、
  十位小数上限、采样原码范围门禁、未完成 readback 时禁止 Wrench：PASS。
- `robot_hw_ethercat` variant tests：34 passed；`rt_control_bringup` composition tests：46 passed。
- `tools/quality_gate.sh`：214 passed；`git diff --check`、shell/Python 静态检查通过。
- 未启动 EtherCAT、未读取现场 SDO、未发布真实 Wrench、未 reset/enable/运动/PLC 写入。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: X503 raw shadow 输出不打开 EtherCAT 网卡、不调用 `ethercat upload`、不提供任何
  控制或 SDO 写入口。
- F2: 最小单位元数据只读集合固定为 `0x8005:06~11` 和 `0x8005:12~17`；其它校准、
  置零、存储对象不属于本阶段。
- F3: `WrenchStamped` 只有在六个 raw 通道完整、单位码/小数位通过只读回读、且采样原码
  范围门禁通过后才允许发布；当前默认策略是用户批准的范围假设，raw shadow 可独立运行。

## 遗留

- 目标机只读 SDO/身份/PDO 验证尚未执行；需单独维护窗口授权。
- X503 实际单位码/小数位回读和左右传感器 TF 仍需硬件/模型 owner 确认；V1.6 手册已
  解决单位码字典。范围策略是用户批准的工程假设，不保证检测通讯中断后的冻结旧帧。
- GitHub main CI 尚未执行；当前实现不改变正在运行的 Native release。
