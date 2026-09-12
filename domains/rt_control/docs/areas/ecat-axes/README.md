# ecat-axes — EtherCAT 14 轴伺服

**范围**：EtherCAT 主站、14 个运动轴与 2 台 X503 的拓扑（18 位环）、slave profile、PDO/SDO 映射、
极性/坐标变换、启动 SDO。
**Owner 包/资产**：`src/rt_control/robot_hw_ethercat`、`patches/ecat_icube`。

不属于本区：使能/失能状态机（→ lifecycle）、控制器与轨迹（→ motion）、
IgH 安装与宿主（→ realtime-host）。

三代机的模块组合和分范围校验由
[`release-deploy-20260907-01`](../release-deploy/records/2026-09-07-electri-118-machine-profiles.md)
登记；在步科 EtherCAT 身份、PDO/SDO、环位和机械参数确认前，它不改变本区当前
`alfa_v1` 的生产拓扑。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| X503-PREOP-1 | 两侧单位/小数位每次初始化只在 PREOP 读取一次，OP 用本次快照与 PDO；离开 OP 或断链后快照失效。主站拒绝非 PREOP CoE 请求。 | [ecat-axes-20260912-01](records/2026-09-12-x503-preop-snapshot.md)#F1-F4 | PASS（T3不使能；PDI固件根因、长期/运动未闭环） |
| TI5-ASSIGN-1 | Native 启动检查实际安装 profile、驱动分配和 IgH 缓存，仅将已知默认分配恢复为已验证的 1601/1A01；未知映射拒绝启动，正确暖态无写入/重扫。 | [ecat-axes-20260911-01](records/2026-09-11-ti5-assignment-recovery.md)#F1-F3 | PARTIAL（T3；掉电重复性未验） |
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
| 05#F1 | PP 夹爪可选模块按已发送 PDO 与同序号反馈握手；旧 CSP profile 不变，TBD 和实机准入继续阻塞 | [PP 夹爪记录](records/2026-09-08-zeroerr-pp-gripper.md)#F1 | PARTIAL（T1 离线/增量容器） |
| 06#F1 | GR10-EC-6SW ESI 声明两个 JunctionSlave 设备，ID/端口说明已归档；实物身份与十八从站数量已在 07#F1 核验 | [汇川 ESI 记录](records/2026-09-12-inovance-gr10-esi.md) | T0 文件事实；T2 进展见 07#F1 |
| 07#F1 | arms_only 实扫左臂 X2=1..8、右臂 X3=9..16、分支器=0/17，共 18 PREOP 从站；物理轴序后续在 08#F1 确认，运行态与 DC 待验 | [双臂分支扫描](records/2026-09-12-gen3-arm-branch-scan.md)#F1 | PARTIAL（T2 只读） |
| 08#F1 | 用户确认两臂均为 J1..J7 后接夹爪：左 CSP=1..7、PP=8，右 CSP=9..15、PP=16；物理清单已保存，正式 Model/运行绑定仍 TBD | [双臂物理轴序](records/2026-09-12-gen3-arm-axis-order.md)#F1 | UNVERIFIED（T0 配置；无实机运动） |
| 09#F1 | 16 台当前均为 CSP、10-byte Rx/Tx；PP 候选为 8/14-byte、6072=int16。605D 实机不存在，730F 位置 3/8/12/16，停止与电池问题待确认 | [零差协议核验](records/2026-09-12-gen3-zeroerr-protocol.md) | PARTIAL（T2 只读、T1 软件） |
| 10#F1 | 用户确认仅左 J3/右 J4 按单圈，夹爪需要多圈；补电池后夹爪错误变为 7314，需单独恢复圈数基准，未自动复位 | [编码器用途与电池复查](records/2026-09-12-gen3-encoder-usage-policy.md) | PARTIAL（策略、T2 只读） |
| 11#F1 | 左夹爪 8 已获授权写一次 2242，7314 清除但 Fault 持续；按停止条件暂停，右夹爪 16 未写，普通 6040 清错待追加授权 | [左夹爪编码器复位](records/2026-09-12-left-gripper-encoder-reset.md) | PARTIAL（T3 单次写入、未使能） |
| 12#F1 | 左夹爪获追加授权完成一次 6040 SDO 清错，但仍 Fault/1001=1/603F=0；右侧条件不满足，需厂家确认 PREOP 清错及 2242 语义 | [左夹爪普通清错](records/2026-09-12-left-gripper-fault-reset.md) | PARTIAL（T3，未恢复/未使能） |
| 13#F1 | 用户上位机重置编码器后，两夹爪获新授权各做一次普通清错；6040=80 可回读但 Fault 持续，控制字已归 0 | [上位机复位后双夹爪清错](records/2026-09-12-grippers-post-vendor-fault-reset.md) | PARTIAL（T3，未恢复/未使能） |
| 14#F1 | 用户重新上电后两夹爪 603F/1001=0，全部 16 轴 Fault 位清除；仍为 CSP、NotReadyToSwitchOn，PP 和运行未验证 | [重新上电后复查](records/2026-09-12-grippers-power-cycle-recovery.md) | PARTIAL（T2；更新 11..13 的历史故障状态） |
| 15#F1 | 用户零位已归档；14 CSP+2 PP 实际 OP、WC48/48、16轴使能/保持/失能成功，最终 Idle/PREOP；PP运动/限力尚未验证 | [原位使能实测](records/2026-09-12-gen3-stationary-enable.md) | PARTIAL（T3；当前状态及模式以本记录为准） |

## 记录索引（倒序）

- 2026-09-12 [X503 PREOP 一次快照与 CoE 状态限制](records/2026-09-12-x503-preop-snapshot.md) — fix，PASS（T3 不使能启动；OP 零邮箱流量）

- 2026-09-11 [Ti5分配和安装配置恢复，使能交付](records/2026-09-11-ti5-assignment-recovery.md) — corrective，PARTIAL（T3）

- 2026-09-12 [三代机零位与14 CSP+2 PP原位使能](records/2026-09-12-gen3-stationary-enable.md)：PARTIAL，真实使能/失能完成，当前已停机。
- 2026-09-12 [重新上电后双夹爪故障解除](records/2026-09-12-grippers-power-cycle-recovery.md)：PARTIAL，只读确认，未再次清错或使能。
- 2026-09-12 [零差上位机复位后两夹爪普通清错](records/2026-09-12-grippers-post-vendor-fault-reset.md)：PARTIAL，清错位回读成功，但故障态未退出。
- 2026-09-12 [左夹爪一次普通清错后仍 Fault](records/2026-09-12-left-gripper-fault-reset.md)：PARTIAL，SDO 成功但驱动未恢复，右侧暂停。
- 2026-09-12 [左夹爪编码器复位后 Fault 持续](records/2026-09-12-left-gripper-encoder-reset.md)：PARTIAL，仅位置 8 写入一次，右侧暂停。
- 2026-09-12 [J3/J4 单圈与夹爪多圈电池复查](records/2026-09-12-gen3-encoder-usage-policy.md)：PARTIAL，策略登记，编码器复位未执行。
- 2026-09-12 [零差协议回读与 PP 类型/对齐修正](records/2026-09-12-gen3-zeroerr-protocol.md)：PARTIAL，回读与软件修正，停止策略和电池问题开放。
- 2026-09-12 [双臂 J1..J7/夹爪物理轴序与模式](records/2026-09-12-gen3-arm-axis-order.md)：用户确认，物理清单落盘，保持运行准入关闭。
- 2026-09-12 [三代机双臂 X2/X3 与十八从站扫描](records/2026-09-12-gen3-arm-branch-scan.md)：PARTIAL，接线与身份核验，未启动控制栈。
- 2026-09-12 [汇川 GR10-EC-6SW 分支器 ESI](records/2026-09-12-inovance-gr10-esi.md)：investigation，文件已核对，实际拓扑待验。
- 2026-09-08 [ELECTRI-118 零差 PP 夹爪适配](records/2026-09-08-zeroerr-pp-gripper.md)：PARTIAL，离线/增量容器通过，实机待验。
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
