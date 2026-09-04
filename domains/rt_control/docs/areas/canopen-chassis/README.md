# canopen-chassis — CANopen 履带底盘

**范围**：CANopen 主站与两条履带（Node 2/3）的 bus 配置、EDS/DCF 生成物、
速度/位置换算、心跳与超时、停机同步策略。
**Owner 包/资产**：`src/rt_control/robot_hw_canopen`、`patches/ros2_canopen`。

不属于本区：diff_drive 控制器参数（→ motion）、CAN 接口宿主配置与命名
（→ realtime-host）、BMS 所在的 can1（→ io-power）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| F1 | 履带主动轮半径 `0.1044 m`（直径 `0.2088 m`）；Node 2/3 位置与速度 to-device 比例均为 `-609789.0539919361`，逆比例均为 `-1.63991136517387e-6`；diff-drive 逻辑半径保持 `1.0` | BQ-139；[2026-08-21 记录](records/2026-08-21-track-mechanics-1044mm-19598mm.md) | 主动轮和比例已裁决，待 T4 实车复验；有效轮距由 motion 区 BQ-140 取代 |
| F2 | CANopen `alfa_v1` descriptor 拥有 Node 2/3、mode 3、side/profile 注册并与 `bus.yml` 严格对齐；通用 `Cia402System` 不写死左右节点 | [release-deploy-20260903-01](../release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F6 | PASS（T1 Docker/Mock） |

## 记录索引（倒序）

- [2026-08-21 履带主动轮半径与轮距更正](records/2026-08-21-track-mechanics-1044mm-19598mm.md)
- [2026-08-19 CANopen 硬件包接管变体 Xacro（未合并分支历史）](records/2026-08-19-hardwareinfo-motor-topology.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-006、T-016、T-017
- `docs/canopen_drive_adaptation.md`、`docs/ros2_canopen_capability_report.md`、
  `docs/canopen-shutdown-sync-tolerance-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-064、BQ-118、BQ-132（BLOCKED：总线无报文导致
  controller_manager 启动退出）
