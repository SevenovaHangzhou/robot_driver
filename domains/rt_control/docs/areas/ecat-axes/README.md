# ecat-axes — EtherCAT 14 轴伺服

**范围**：EtherCAT 主站、14 个运动轴与 2 台 X503 的拓扑（18 位环）、slave profile、PDO/SDO 映射、
极性/坐标变换、启动 SDO。
**Owner 包/资产**：`src/rt_control/robot_hw_ethercat`、`patches/ecat_icube`。

不属于本区：使能/失能状态机（→ lifecycle）、控制器与轨迹（→ motion）、
IgH 安装与宿主（→ realtime-host）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| MAIN-RATE-1 | Native/Docker包含0001..0012链和1 kHz周期；0012修正CI风格检查，JSB显式125 Hz保持公共契约。容器安装入口/Mock生命周期/有序退出0通过。 | [ecat-axes-20260910-02](records/2026-09-10-electri-97-main-packaging.md)#F1-F3 | PASS（T1容器；实机Docker发布未验） |
| RATE-1 | 默认部署漏改的 Xacro 250 Hz 已补齐为 1000；首轮启动 6.821 秒，16 个 DC 节点实际周期 1 ms、同相，14 轴保持失能；5 秒 DC 告警本轮未出现。 | [ecat-axes-20260910-01](records/2026-09-10-electri-97-startup-rate-correction.md)#F1-F4 | PARTIAL（T3单轮；冷启动/长期/运动未验） |
| TI5-PDO-1 | 四份Ti5 profile保留已校验PDO，失败在外层终止；三轮映射abort为0，PDO阶段由4.224秒降至约3.454秒，最终新配置保留、14轴失能无故障位。 | [ecat-axes-20260909-03](records/2026-09-09-electri-97-ti5-preserved-pdo.md)#F1-F4 | PASS（T3失能启动；DC/长期/运动未闭环） |
| DC-STAGES-1 | 优化前非DC约11.5秒；Ti5保留后约10.73秒（PDO约3.45秒、进入OP约4.57秒）。DC仍独立波动，OP/WKC正常不保证持续偏差小于10 us。 | [ecat-axes-20260909-03](records/2026-09-09-electri-97-ti5-preserved-pdo.md)#F2-F3 | PASS（阶段计时；整体DC未修复） |
| DC-HANDOFF-1 | Native 用短期周期维护覆盖初始化返回至正常 read 接管；三轮交接窗口最大间隔4.005/4.018/4.019 ms，双X503无0x001A、接管后WKC完整。 | [ecat-axes-20260909-01](records/2026-09-09-electri-97-continuous-handoff.md)#F1-F4 | PASS（T3三轮暖启动；初始DC等待、长期/运动未关闭） |
| DC-TIME-1 | ICube application_time 四条路径统一使用 CLOCK_MONOTONIC uptime 纳秒；三轮时基单变量测试仍有启动等待及 X503 0x001A 瞬态。 | [ecat-axes-20260908-01](records/2026-09-08-electri-97-monotonic-application-time.md)#F1-F2 | PARTIAL（T3 Native） |
| 01#F1 | 18 位运行拓扑固定为 Hub 0、运动轴 1..12、Hub 13、右/左 X503 14/15、Turn 16、Updown 17；X503 链为 `Hub 13 OUT8 -> right -> left` | [ecat-axes-20260902-01](records/2026-09-02-add-dual-x503-profile-files.md)#F1 | PARTIAL（离线通过；实机 OP/WC 待验） |
| 01#F2 | 两份 profile 均锁定实机 identity `0x00000503/0x26483052@0x00020111`、fixed RxPDO `0x1601` 与 TxPDO `0x1A00`（25×DINT/100 bytes），必须保留校验而非重写 | [ecat-axes-20260902-01](records/2026-09-02-add-dual-x503-profile-files.md)#F2 | PARTIAL（补丁链通过；实机 OP 待验） |
| 01#F3 | X503 仅提供 raw state interface，不进入 14 轴使能、JTC 或公共 `/joint_states` | [ecat-axes-20260902-01](records/2026-09-02-add-dual-x503-profile-files.md)#F3 | PASS（静态/Mock） |
| 02#F1 | `alfa_v1` descriptor 分离 14 个 axes、两台 state-only X503 sensors 与 Hub responders，并共同生成 real/mock 与诊断 topology | [release-deploy-20260903-01](../release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F1-F3 | PASS（T1 Docker/Mock；实机仍沿用 01 的 PARTIAL） |
| 03#F1 | 18 位环进入 Operation 后，Hub position 0 为 OP，Hub position 13 保持 PREOP；Native READY 门禁必须逐位验证，不能把两个 Hub 合并为同一状态假设。 | [ecat-axes-20260904-01](records/2026-09-04-hub-runtime-state-gate.md)#F1 | PASS（T2 现场只读；主线运行复验待维护窗口） |
| 04#F1 | X503B raw shadow bridge 只消费 rt-control state frame；V1.6 已定义单位码 `5=N`、`7=N·m`，采样原码采用 `-999999..999999` 范围门禁，实际回读/TF 未完成时禁止 WrenchStamped | [ecat-axes-20260906-01](records/2026-09-06-electri-116-x503b-shadow-bridge.md)#F1-F3 | PARTIAL（T1；目标 SDO/实际参数/TF 待验） |

## 记录索引（倒序）

- 2026-09-10 [ELECTRI-97 main容器与125 Hz公共状态兼容](records/2026-09-10-electri-97-main-packaging.md) — fix，PASS（T1 Docker）
- 2026-09-10 [ELECTRI-97 默认启动周期遗漏与旧结论纠错](records/2026-09-10-electri-97-startup-rate-correction.md) — corrective，PARTIAL（T3失能启动6.821秒）
- 2026-09-09 [ELECTRI-97 Ti5保留PDO及失败传播修正](records/2026-09-09-electri-97-ti5-preserved-pdo.md) — fix，PASS（T3三轮失能启动）
- 2026-09-09 [ELECTRI-97 启动阶段计时与初始偏移取证](records/2026-09-09-electri-97-startup-stage-timing.md) — investigation，PASS（测量）
- 2026-09-09 [ELECTRI-97 连续周期交接与发送间隔实测](records/2026-09-09-electri-97-continuous-handoff.md) — fix，PARTIAL（交接T3通过，整体DC问题未关闭）
- 2026-09-08 [ELECTRI-97 单调应用时间修正与三轮对照](records/2026-09-08-electri-97-monotonic-application-time.md) — fix，PARTIAL
- 2026-09-04 [Hub 0/13 运行态门禁修正](records/2026-09-04-hub-runtime-state-gate.md) — fix，PARTIAL（现场事实已确认；新 main 运行复验待维护窗口）
- 2026-09-06 [ELECTRI-116 X503B raw shadow采集与只读单位元数据桥](records/2026-09-06-electri-116-x503b-shadow-bridge.md) — feature，PARTIAL（T1；目标 SDO/validity/TF 待验）
- 2026-09-02 [双 X503 接入 18 位运行拓扑](records/2026-09-02-add-dual-x503-profile-files.md) — feature，PARTIAL（离线构建/测试通过；实机 OP/WC/raw 待验）
- 2026-08-19 [EtherCAT 硬件包接管变体 Xacro（未合并分支历史）](records/2026-08-19-hardware-owned-variant-xacro.md) — feature，历史 T1；当前由 02#F1 取代

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-010、T-013、Joint5/ZeroErr/Ti5 极性三条散记
- `docs/xmc-updown-sw511-fixed-pdo.md`、`docs/ethercat_enable_disable_commissioning.md`
- 相关 BQ（不完全）：BQ-114、BQ-115、BQ-117（OPEN/HIGH-RISK）、BQ-126
