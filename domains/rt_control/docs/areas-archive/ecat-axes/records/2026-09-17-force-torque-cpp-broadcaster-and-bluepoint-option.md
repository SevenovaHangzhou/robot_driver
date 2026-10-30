---
id: ecat-axes-20260917-01
area: ecat-axes
title: 六维力统一 C++ broadcaster 与三代机蓝点可选配置
date: 2026-09-17
type: feature
trigger: 用户要求蓝点与 X503 统一使用 semantic component + C++ broadcaster，并允许传感器缺货期双臂独立运行
commits: [feature/force-torque-cpp-broadcaster]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence:
  - "用户提供蓝点 ESI SHA-256: 8e654bdf540ebac4522f68f403b6a4568677c46ecec14db8451559fa028742ad"
  - "CRLF 规范化仓库副本 SHA-256: f92f783bfe91152163e4812b10314399a2f82ffa8135ea76295ee0f953b6e6f8"
  - "用户提供蓝点协议 PDF SHA-256: 39432f1304b68af1839a3553b4581da92a3923306945e5985cdbae7d7689ae0c"
  - "扩大 Python/config 回归：223 passed"
  - "五个受影响包及 EtherCAT 固定依赖构建：PASS"
  - "五包 colcon test-result：316 tests，0 errors，0 failures，0 skipped"
  - "scoped tests 八包闭包：543 tests，0 errors，0 failures，0 skipped"
  - "tools/quality_gate.sh：282 passed、13 skipped、策略覆盖率 83%"
  - "ament_uncrustify：新增 C++ 文件无格式差异"
  - "Mock：左右 X503 C++ broadcaster active，并随 controller_manager 有序停用"
supersedes: []
related: [BQ-143, BQ-146, BQ-149, ecat-axes-20260906-01, release-deploy-20260907-01]
---

## 背景

二代机两台 X503 原由非实时 Python bridge 消费 DynamicJointState，再发布 raw 与
WrenchStamped。后续力控闭环需要 controller-manager 内的类型化 state interface 路径；三代机
将使用蓝点/ACTI P140000107，但传感器晚于已组装机械臂到货，因此不能把二十从站拓扑变成
双臂启动前提。本次统一发布基础设施，同时保持五个 physical profile 和无传感器十八从站事实。

## 改动

- `rt_control_semantic_components` 新增固定容量 ForceTorqueSensor；新包
  `rt_force_torque_broadcaster` 提供无 command interface 的 C++ controller、纯处理器、raw/
  WrenchStamped/calibration publishers 及 OP/链路失效锁存。update 不访问文件/网络/参数，
  不解析 JSON，不持锁，不创建容器。
- X503 每次启动的 PREOP 只读回读保持不变；snapshot 只生成私有临时 controller 参数。
  左右 broadcaster 顺序进入既有 spawner 链。删除运行时 Python bridge/node/可执行入口，保留
  raw/Wrench/calibration topic、QoS、动态 decimals、sample-code 范围和掉线后需新启动语义。
- 归档蓝点 ESI，新增 `0xA1/0x8081@0x2` state-only family/profile。完整保留 RxPDO
  `0x1600` 8x32 bit 与 TxPDO `0x1A00` 9x32 bit；`0x2000` tare 及七个未解释 Rx 字段固定为零，
  不导出 command interface。六维比例固定 `1e-4`，但 StatusCode/freshness 未据此推断。
- alfa_v3 新增通用 hardware option：默认 `force_sensors=none`；`bluepoint_dual` 仅允许
  `arms_only`、`arms_updown`、`full_robot`，叠加两台 state-only sensor 并把 EtherCAT ring
  置为 TBD。未新增第六个 physical profile，也不自动探测/降级。
- 蓝点 broadcaster draft 固定共享插件与左右 topic，但 ring position、frame_id 和
  `calibration_valid` 保持 TBD/false；`rt_control_module.launch.py` 仍为 validation-only。

## 验证

先后完成 semantic/processor/controller/option/profile 的 RED，再实现到 GREEN。扩大 Python/config
回归 223 项通过。使用冻结 `ecat_icube@1390be7`、`ros2_controllers@cbcf662`、
`robot_interfaces@92d6ff2` 和本地 IgH 前缀构建受影响包；五包测试汇总 316 项、0 error、
0 failure、0 skipped。Mock launch 实际加载左右 C++ broadcaster、发布 raw 接口路径，并在
controller_manager 退出时依次 deactivate/shutdown，进程正常退出。
带固定 vendor/IgH 运行库环境的 `tools/run_scoped_tests.sh` 覆盖八个下游包，最终汇总
543 项测试、0 error、0 failure、0 skipped。本地 `tools/quality_gate.sh` 为 282 passed、
13 个环境性 skipped、策略覆盖率 83%；本机未安装 ShellCheck，强制 ShellCheck 门禁留给 CI。

未连接蓝点实物，未访问工控机 EtherCAT，未 reset、enable、运动、tare、SDO write 或 PLC 输出。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: X503 周期数据发布统一迁移到 semantic component + C++ controller；Python 只保留 OP 前
  的只读快照/参数生成，不再搬运运行帧。X503 拓扑/profile/topic 与使能资源不变。
- F2: 蓝点 profile 的文件事实为 Vendor `0x000000A1`、Product `0x00008081`、Revision
  `0x00000002`，RxPDO 32 bytes、TxPDO 36 bytes，力/力矩 `raw * 1e-4` SI；tare 不开放。
- F3: 三代机默认 `force_sensors=none` 保持已确认 18 responder 双臂布局；
  `bluepoint_dual` 是显式 draft option，不能在缺件时自动回退，也不能在 ring/frame 未确认时运行。
- F4: 旧记录 ecat-axes-20260906-01 中 `39432f...` 的“X503 V1.6 PDF”标签不正确；该哈希
  实际属于本次蓝点 V1.1 PDF。X503 profile 保存的手册哈希仍为 `b68b7cde...`。旧记录原文按
  不可变规则保留，本条作为 provenance corrective。

## 遗留

蓝点到货接线后需在 master Idle/Inactive 下只读确认左右分支枚举、总 responder 数、identity、
PDO 与 DC，再由 Robot Model owner 提供左右 frame 和安装朝向。厂家还需给出 StatusCode 位定义；
sample-counter 停滞阈值须按实测周期裁决后，才能将 `calibration_valid` 提升并进入力控闭环准入。
X503 C++ 路径仍需在旧机维护窗口做不使能实机回归，不能用 Mock 替代。
