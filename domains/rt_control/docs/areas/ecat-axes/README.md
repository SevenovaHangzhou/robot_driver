# ecat-axes — EtherCAT 14 轴伺服

**范围**：EtherCAT 主站、14 个运动轴与 2 台 X503 的拓扑（18 位环）、slave profile、PDO/SDO 映射、
极性/坐标变换、启动 SDO。
**Owner 包/资产**：`src/rt_control/robot_hw_ethercat`、`patches/ecat_icube`。

不属于本区：使能/失能状态机（→ lifecycle）、控制器与轨迹（→ motion）、
IgH 安装与宿主（→ realtime-host）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | 18 位运行拓扑固定为 Hub 0、运动轴 1..12、Hub 13、右/左 X503 14/15、Turn 16、Updown 17；X503 链为 `Hub 13 OUT8 -> right -> left` | [ecat-axes-20260902-01](records/2026-09-02-add-dual-x503-profile-files.md)#F1 | PARTIAL（离线通过；实机 OP/WC 待验） |
| 01#F2 | 两份 profile 均锁定实机 identity `0x00000503/0x26483052@0x00020111`、fixed RxPDO `0x1601` 与 TxPDO `0x1A00`（25×DINT/100 bytes），必须保留校验而非重写 | [ecat-axes-20260902-01](records/2026-09-02-add-dual-x503-profile-files.md)#F2 | PARTIAL（补丁链通过；实机 OP 待验） |
| 01#F3 | X503 仅提供 raw state interface，不进入 14 轴使能、JTC 或公共 `/joint_states` | [ecat-axes-20260902-01](records/2026-09-02-add-dual-x503-profile-files.md)#F3 | PASS（静态/Mock） |
| 02#F1 | `alfa_v1` descriptor 分离 14 个 axes、两台 state-only X503 sensors 与 Hub responders，并共同生成 real/mock 与诊断 topology | [release-deploy-20260903-01](../release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F1-F3 | PASS（T1 Docker/Mock；实机仍沿用 01 的 PARTIAL） |
| 03#F1 | 18 位环进入 Operation 后，Hub position 0 为 OP，Hub position 13 保持 PREOP；Native READY 门禁必须逐位验证，不能把两个 Hub 合并为同一状态假设。 | [ecat-axes-20260904-01](records/2026-09-04-hub-runtime-state-gate.md)#F1 | PASS（T2 现场只读；主线运行复验待维护窗口） |

## 记录索引（倒序）

- 2026-09-04 [Hub 0/13 运行态门禁修正](records/2026-09-04-hub-runtime-state-gate.md) — fix，PARTIAL（现场事实已确认；新 main 运行复验待维护窗口）
- 2026-09-02 [双 X503 接入 18 位运行拓扑](records/2026-09-02-add-dual-x503-profile-files.md) — feature，PARTIAL（离线构建/测试通过；实机 OP/WC/raw 待验）
- 2026-08-19 [EtherCAT 硬件包接管变体 Xacro（未合并分支历史）](records/2026-08-19-hardware-owned-variant-xacro.md) — feature，历史 T1；当前由 02#F1 取代

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-010、T-013、Joint5/ZeroErr/Ti5 极性三条散记
- `docs/xmc-updown-sw511-fixed-pdo.md`、`docs/ethercat_enable_disable_commissioning.md`
- 相关 BQ（不完全）：BQ-114、BQ-115、BQ-117（OPEN/HIGH-RISK）、BQ-126
