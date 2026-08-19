# ecat-axes — EtherCAT 14 轴伺服

**范围**：EtherCAT 主站与 14 轴伺服的拓扑（16 位环）、slave profile、PDO/SDO 映射、
极性/坐标变换、启动 SDO。
**Owner 包/资产**：`src/rt_control/robot_hw_ethercat`、`patches/ecat_icube`。

不属于本区：使能/失能状态机（→ lifecycle）、控制器与轨迹（→ motion）、
IgH 安装与宿主（→ realtime-host）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | EtherCAT descriptor 是轴/family/ring/profile/mode 的 owner-local 注册点，并驱动 real/mock schema、slave sensors 与 diagnostics 派生 | [ecat-axes-20260819-01](records/2026-08-19-hardware-owned-variant-xacro.md)#F1 | 有效（Docker/T1） |
| 01#F2 | 当前唯一变体 alfa_v1 保持 14 轴 ABI、CSP 8 与环位；未知变体、CSV/CST 和 profile/mode 漂移 fail closed | [ecat-axes-20260819-01](records/2026-08-19-hardware-owned-variant-xacro.md)#F2 | 有效（Docker/T1；实机/HIL 待验证） |
| 01#F3 | EtherCAT 插件从 HardwareInfo 严格读取 ec_module.*，不再扫描整份 URDF | [ecat-axes-20260819-01](records/2026-08-19-hardware-owned-variant-xacro.md)#F3 | 有效（Docker/T1） |

## 记录索引（倒序）

- 2026-08-19 [EtherCAT 硬件包接管变体 Xacro 与 HardwareInfo 拓扑](records/2026-08-19-hardware-owned-variant-xacro.md) — feature，PASS（Docker/T1；实机/HIL 未执行）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-010、T-013、Joint5/ZeroErr/Ti5 极性三条散记
- `docs/xmc-updown-sw511-fixed-pdo.md`、`docs/ethercat_enable_disable_commissioning.md`
- 相关 BQ（不完全）：BQ-114、BQ-115、BQ-117（OPEN/HIGH-RISK）、BQ-126
