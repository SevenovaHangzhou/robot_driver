# canopen-chassis — CANopen 履带底盘

**范围**：CANopen 主站与两条履带（Node 2/3）的 bus 配置、EDS/DCF 生成物、
速度/位置换算、心跳与超时、停机同步策略。
**Owner 包/资产**：`src/rt_control/robot_hw_canopen`、`patches/ros2_canopen`。

不属于本区：diff_drive 控制器参数（→ motion）、CAN 接口宿主配置与命名
（→ realtime-host）、BMS 所在的 can1（→ io-power）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | 当前履带安全节点表固定为 Node 2/3、PV mode 3，并由 ros2_canopen 全部安全路径共同消费 | [canopen-chassis-20260818-01](records/2026-08-18-driver-variant-node-table.md)#F1 | 有效 |
| 01#F2 | 更换 CANopen 驱动型号必须同时通过 profile/EDS、换算、生命周期及停机证据门禁，不能只替换 DCF | [canopen-chassis-20260818-01](records/2026-08-18-driver-variant-node-table.md)#F2 | 有效 |

## 记录索引（倒序）

- 2026-08-18 [履带节点/模式安全投影（ELECTRI-94）](records/2026-08-18-driver-variant-node-table.md) — feature，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-006、T-016、T-017
- `docs/canopen_drive_adaptation.md`、`docs/ros2_canopen_capability_report.md`、
  `docs/canopen-shutdown-sync-tolerance-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-064、BQ-118、BQ-132（BLOCKED：总线无报文导致
  controller_manager 启动退出）
