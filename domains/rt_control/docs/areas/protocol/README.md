# protocol — 总线协议、驱动插件与生命周期

**范围**：总线协议、驱动插件与生命周期。本区对应一个同名职责agent。

**Owner路径**：src/rt_control/hw_ethercat（总线胶水、从站插件、协议profile、ESI）；hw_canopen；hw_can_damiao；hw_semantic_components；patches/ecat_icube、patches/igh、patches/ros2_canopen。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：机型事实与标定（hwcfg_machines）归hw-config；控制器（含ctrl_enable_manager）归control；共享库归infra；hostsetup归platform。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| canopen-chassis/F3 | 三代机四路舵角 provider 复用 Lely/ros2_canopen 主站，导出 position/feedback_age_ms 且无 command interface；硬件 profile 保持 draft/TBD | [CANopen 舵角 provider](../../areas-archive/canopen-chassis/records/2026-09-16-swerve-encoder-state-provider.md)#F1-F4 | PARTIAL（T1 单元/Mock；EDS、总线与实机待验） |
| ecat-axes/X503-PREOP-1 | 两侧单位/小数位每次初始化只在 PREOP 读取一次，OP 用本次快照与 PDO；离开 OP 或断链后快照失效。主站拒绝非 PREOP CoE 请求。 | [ecat-axes-20260912-01](../../areas-archive/ecat-axes/records/2026-09-12-x503-preop-snapshot.md)#F1-F4 | PASS（T3不使能；PDI固件根因、长期/运动未闭环） |
| ecat-axes/TI5-PDO-1 | 四份Ti5 profile保留已校验PDO，失败在外层终止；三轮映射abort为0，PDO阶段由4.224秒降至约3.454秒，最终新配置保留、14轴失能无故障位。 | [ecat-axes-20260909-03](../../areas-archive/ecat-axes/records/2026-09-09-electri-97-ti5-preserved-pdo.md)#F1-F4 | PASS（T3失能启动；DC/长期/运动未闭环） |
| ecat-axes/DC-HANDOFF-1 | Native 用短期周期维护覆盖初始化返回至正常 read 接管；三轮交接窗口最大间隔4.005/4.018/4.019 ms，双X503无0x001A、接管后WKC完整。 | [ecat-axes-20260909-01](../../areas-archive/ecat-axes/records/2026-09-09-electri-97-continuous-handoff.md)#F1-F4 | PASS（T3三轮暖启动；初始DC等待、长期/运动未关闭） |
| ecat-axes/DC-TIME-1 | ICube application_time 四条路径统一使用 CLOCK_MONOTONIC uptime 纳秒；三轮时基单变量测试仍有启动等待及 X503 0x001A 瞬态。 | [ecat-axes-20260908-01](../../areas-archive/ecat-axes/records/2026-09-08-electri-97-monotonic-application-time.md)#F1-F2 | PARTIAL（T3 Native） |
| ecat-axes/01#F2 | 两份 profile 均锁定实机 identity `0x00000503/0x26483052@0x00020111`、fixed RxPDO `0x1601` 与 TxPDO `0x1A00`（25×DINT/100 bytes），必须保留校验而非重写 | [ecat-axes-20260902-01](../../areas-archive/ecat-axes/records/2026-09-02-add-dual-x503-profile-files.md)#F2 | PARTIAL（补丁链通过；实机 OP 待验） |
| ecat-axes/01#F3 | X503 仅提供 raw state interface，不进入 14 轴使能、JTC 或公共 `/joint_states` | [ecat-axes-20260902-01](../../areas-archive/ecat-axes/records/2026-09-02-add-dual-x503-profile-files.md)#F3 | PASS（静态/Mock） |
| ecat-axes/05#F1 | PP 夹爪可选模块按已发送 PDO 与同序号反馈握手；旧 CSP profile 不变，TBD 和实机准入继续阻塞 | [PP 夹爪记录](../../areas-archive/ecat-axes/records/2026-09-08-zeroerr-pp-gripper.md)#F1 | PARTIAL（T1 离线/增量容器） |
| ecat-axes/06#F1 | GR10-EC-6SW ESI 声明两个 JunctionSlave 设备，ID/端口说明已归档；实物身份与十八从站数量已在 07#F1 核验 | [汇川 ESI 记录](../../areas-archive/ecat-axes/records/2026-09-12-inovance-gr10-esi.md) | T0 文件事实；T2 进展见 07#F1 |
| ecat-axes/15#F1 | 用户零位已归档；14 CSP+2 PP 实际 OP、WC48/48、16轴使能/保持/失能成功，最终 Idle/PREOP；PP运动/限力尚未验证 | [原位使能实测](../../areas-archive/ecat-axes/records/2026-09-12-gen3-stationary-enable.md) | PARTIAL（T3；当前状态及模式以本记录为准） |
| lifecycle/03#F1 | `Cia402Axis` 只绑定/解码接口；managed joints、批次、时序和 Ti5 终态由 enable_manager 显式配置并冻结 | [release-deploy-20260903-01](../../areas-archive/release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F7 | PASS（T1 Docker/Mock） |
| release-deploy/2026-09-24#F1 | 旧履带启动、差速控制、Leadshine CANopen 生成链及宿主 can0 已移除；上游补丁仅保留共享生命周期/RPDO 与 Lely 队列修复，编码器 CAN 身份仍 TBD | [旧履带运行链退役](../../areas-archive/release-deploy/records/2026-09-24-retire-track-runtime.md)#F1-F2 | UNVERIFIED（T0；隔离 9 包/150 测试、Docker 39 包与无设备静态 validation 通过；实机运行待验） |

| protocol-20261014-01#F1–F3 | 离线 renderer、输出侧编码器换算、数据驱动原位预检；无新增运动入口总线检查 | [EtherCAT renderer](records/2026-10-14-ethercat-renderer.md) | UNVERIFIED（T0；conformance/既有测试通过，实机未验） |

| integration-20261014-03#F1–F4 | 头部协议Mapping/被动力传感渲染与独立只读预检，HeadPhase迁入semantic | [第四步记录](../integration/records/2026-10-14-head-force-composition.md) | PARTIAL（离线/Mock证据；真实未验） |

## 跨区事实关联

力反馈发布链与使能状态机见[control](../control/README.md)；机型/标定事实见[hw-config](../hw-config/README.md)。X503 PREOP准入属于本区，发布转换属于control。
