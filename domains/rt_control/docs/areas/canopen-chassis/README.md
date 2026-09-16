# canopen-chassis — CANopen 移动底盘与舵角传感器

**范围**：CANopen 主站、两条履带（Node 2/3）及三代机四路外置舵角编码器的
bus 配置、EDS/DCF 生成物、状态/速度/位置换算、心跳与超时、停机同步策略。
**Owner 包/资产**：`src/rt_control/robot_hw_canopen`、`patches/ros2_canopen`。

不属于本区：diff_drive 控制器参数（→ motion）、CAN 接口宿主配置与命名
（→ realtime-host）、BMS 所在的 can1（→ io-power）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| F1 | 履带主动轮半径 `0.1044 m`（直径 `0.2088 m`）；Node 2/3 位置与速度 to-device 比例均为 `-609789.0539919361`，逆比例均为 `-1.63991136517387e-6`；diff-drive 逻辑半径保持 `1.0` | BQ-139；[2026-08-21 记录](records/2026-08-21-track-mechanics-1044mm-19598mm.md) | 主动轮和比例已裁决，待 T4 实车复验；有效轮距由 motion 区 BQ-140 取代 |
| F2 | CANopen `alfa_v1` descriptor 拥有 Node 2/3、mode 3、side/profile 注册并与 `bus.yml` 严格对齐；通用 `Cia402System` 不写死左右节点 | [release-deploy-20260903-01](../release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F6 | PASS（T1 Docker/Mock） |
| F3 | 三代机四路舵角 provider 复用 Lely/ros2_canopen 主站，导出 position/feedback_age_ms 且无 command interface；硬件 profile 保持 draft/TBD | [CANopen 舵角 provider](records/2026-09-16-swerve-encoder-state-provider.md)#F1-F4 | PARTIAL（T1 单元/Mock；EDS、总线与实机待验） |
| F4 | 四路外置编码器共用 108 齿回转齿圈与 27 齿小齿轮，编码器 4 圈对应舵轴 1 圈；该比例不属于转向电机传动 | [外置编码器齿比确认](records/2026-09-16-swerve-encoder-gearing-confirmation.md)#F1-F4 | PARTIAL（T1 公式/配置；方向、分辨率和实机待验） |

## 记录索引（倒序）

- [2026-09-16 四舵轮外置编码器 108/27 齿传动确认](records/2026-09-16-swerve-encoder-gearing-confirmation.md)
- [2026-09-16 三代机四路 CANopen 舵角 state-only provider](records/2026-09-16-swerve-encoder-state-provider.md)
- [2026-08-21 履带主动轮半径与轮距更正](records/2026-08-21-track-mechanics-1044mm-19598mm.md)
- [2026-08-19 CANopen 硬件包接管变体 Xacro（未合并分支历史）](records/2026-08-19-hardwareinfo-motor-topology.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-006、T-016、T-017
- `docs/canopen_drive_adaptation.md`、`docs/ros2_canopen_capability_report.md`、
  `docs/canopen-shutdown-sync-tolerance-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-064、BQ-118、BQ-132（BLOCKED：总线无报文导致
  controller_manager 启动退出）
