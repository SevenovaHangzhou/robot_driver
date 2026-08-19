# canopen-chassis — CANopen 履带底盘

**范围**：CANopen 主站与两条履带（Node 2/3）的 bus 配置、EDS/DCF 生成物、
速度/位置换算、心跳与超时、停机同步策略。
**Owner 包/资产**：`src/rt_control/robot_hw_canopen`、`patches/ros2_canopen`。

不属于本区：diff_drive 控制器参数（→ motion）、CAN 接口宿主配置与命名
（→ realtime-host）、BMS 所在的 can1（→ io-power）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | CANopen descriptor 注册 joint/node/mode/side/profile 并驱动 topology/diagnostics/生成清单；真实运行仍由同包 bus.yml/EDS 持有并严格对齐 | [canopen-chassis-20260819-01](records/2026-08-19-hardwareinfo-motor-topology.md)#F1 | 有效（Docker/T1） |
| 01#F2 | 当前唯一变体 alfa_v1 保持 Node 2/3、mode 3 与既有 ABI；未知变体 fail closed | [canopen-chassis-20260819-01](records/2026-08-19-hardwareinfo-motor-topology.md)#F2 | 有效（Docker/T1） |
| 01#F3 | Cia402System 电机集合来自严格校验的 HardwareInfo；非两节点拓扑仍需生命周期/HIL 准入 | [canopen-chassis-20260819-01](records/2026-08-19-hardwareinfo-motor-topology.md)#F3 | T1 通过；非两节点生命周期/HIL 待验证 |

## 记录索引（倒序）

- 2026-08-19 [CANopen 硬件包接管变体 Xacro 与电机拓扑](records/2026-08-19-hardwareinfo-motor-topology.md) — feature，PASS（Docker/T1；实机/HIL 未执行）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-006、T-016、T-017
- `docs/canopen_drive_adaptation.md`、`docs/ros2_canopen_capability_report.md`、
  `docs/canopen-shutdown-sync-tolerance-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-064、BQ-118、BQ-132（BLOCKED：总线无报文导致
  controller_manager 启动退出）
