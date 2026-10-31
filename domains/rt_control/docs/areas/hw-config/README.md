# hw-config — 机型事实、标定与模型快照

**范围**：机型事实、标定与模型快照。本区对应一个同名职责agent。

**Owner路径**：src/rt_control/hwcfg_machines/**（机型 manifest、单机 deployment/标定、策略文件、设备事实）；src/description/**快照同步。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：协议实现、ESI与通用从站profile归protocol；公共模型权威仍在独立仓库，不在本区私改。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| canopen-chassis/F4 | 四路外置编码器共用 108 齿回转齿圈与 27 齿小齿轮，编码器 4 圈对应舵轴 1 圈；该比例不属于转向电机传动 | [外置编码器齿比确认](../../areas-archive/canopen-chassis/records/2026-09-16-swerve-encoder-gearing-confirmation.md)#F1-F4 | PARTIAL（T1 公式/配置；方向、分辨率和实机待验） |
| contract/08#F1 | 共享模型新增固定边 `base_link → lidar_main`，位姿为 `xyz=(0.382364228640, 0.133500000000, 0.121820508080)`、`rpy=(0, 0.523598775598, 0)` | [contract-20260902-01](../../areas-archive/contract/records/2026-09-02-lidar-main-tf.md)#F1 | 有效；消费者联合验证待完成 |
| contract/08#F2 | `lidar_main.STL` 使用 `base_link` 坐标和毫米单位，消费时必须使用 `0.001` 缩放 | [contract-20260902-01](../../areas-archive/contract/records/2026-09-02-lidar-main-tf.md)#F2 | 有效 |
| contract/08#F3 | `lidar_main` 变更不新增公共接口、不改变 odom TF 所有权或实时控制参数 | [contract-20260902-01](../../areas-archive/contract/records/2026-09-02-lidar-main-tf.md)#F3 | 有效 |
| ecat-axes/07#F1 | arms_only 实扫左臂 X2=1..8、右臂 X3=9..16、分支器=0/17，共 18 PREOP 从站；物理轴序后续在 08#F1 确认，运行态与 DC 待验 | [双臂分支扫描](../../areas-archive/ecat-axes/records/2026-09-12-gen3-arm-branch-scan.md)#F1 | PARTIAL（T2 只读） |
| ecat-axes/08#F1 | 用户确认两臂均为 J1..J7 后接夹爪：左 CSP=1..7、PP=8，右 CSP=9..15、PP=16；物理清单已保存，正式 Model/运行绑定仍 TBD | [双臂物理轴序](../../areas-archive/ecat-axes/records/2026-09-12-gen3-arm-axis-order.md)#F1 | UNVERIFIED（T0 配置；无实机运动） |
| ecat-axes/10#F1 | 用户确认仅左 J3/右 J4 按单圈，夹爪需要多圈；补电池后夹爪错误变为 7314，需单独恢复圈数基准，未自动复位 | [编码器用途与电池复查](../../areas-archive/ecat-axes/records/2026-09-12-gen3-encoder-usage-policy.md) | PARTIAL（策略、T2 只读） |
| ecat-axes/16#F1 | Kinco 四转向 CSP/四驱动 CSV 已建立协议无关接口合同和八轴 fail-closed draft；未创建 identity/profile/runtime variant | [Kinco 舵轮合同草案](../../areas-archive/ecat-axes/records/2026-09-16-kinco-swerve-contract-draft.md)#F1-F4 | UNVERIFIED（T0 配置；实机参数和运行待验） |
| motion/RATIO-1 | 安装机型行走总比27.48、转向总比140；外置编码器108/27独立，运行原始计数单位仍待确认 | [2026-10-07用户确认](../../areas-archive/motion/records/2026-10-07-swerve-installed-ratios.md)#F1-F2 | 已确认机械比；运行换算待验 |
| motion/F5 | HT-WS-HH270 厂家图纸已登记名义轮径、传动、安装包络和悬挂参数；数据保持 draft，不能替代逐轮有效半径、零位、硬限位和背隙标定 | [厂家机械图纸提取](../../areas-archive/motion/records/2026-09-15-swerve-module-mechanical-drawing.md)#F1-F4 | UNVERIFIED（T0 图纸提取；厂家澄清/实测待完成） |
| motion/F9 | 用户与厂商确认无电池多圈编码器按单圈使用会报 `0x730F`，J3/J4 每次使能前需确认受控复位；十四轴 `+1` 是未验证的方向试验值 | [旧工控机 JTC 原位验证](../../areas-archive/motion/records/2026-09-19-v3-jtc-old-ipc-readonly.md)#F4-F5 | PARTIAL（T3 使能已验；方向待逐轴验证） |
| release-deploy/02#F1 | ELECTRI-118 machine manifest 统一登记三代机模块、五个 physical profile 和 control scope；当前为静态 draft，TBD 不得进入真实启动 | [release-deploy-20260907-01](../../areas-archive/release-deploy/records/2026-09-07-electri-118-machine-profiles.md)#F1-F5 | UNVERIFIED（T0 静态） |
| release-deploy/03#F1 | owner_refs 按硬件协议登记，底盘包更名不删除头部 damiao_can；4 ms SYNC 为设计约束而非实机时序结论 | [舵轮关联修复](../../areas-archive/release-deploy/records/2026-09-08-swerve-owner-reference-corrective.md)#F1 | UNVERIFIED（T0 配置回归通过） |

| hw-config-20261014-01#F1–F3 | 设备/机型/单机事实分离；型号与额定参数不猜值；台架腕部环位不外推全机 | [手臂事实分离](records/2026-10-14-arm-fact-separation.md) | UNVERIFIED（T0；schema/对齐通过，实机未验） |

| integration-20261014-03#F1–F4 | head配置按README示例保持draft；确认人/日期、方向/零位、全机力传感拓扑仍TBD | [停止点三报告](../integration/records/2026-10-14-head-force-composition.md) | PARTIAL（离线准入检查；实际确认待补） |

## 跨区事实关联

协议与驱动事实见[protocol](../protocol/README.md)，启动拼装见[integration](../integration/README.md)。

- hw-config-20261023-01#F1：J6专用26B TxPDO包含0x2241与0x3B69，已在未使能OP状态验证同帧采集；使能后标定未验证。见[记录](records/2026-10-23-j6-dual-encoder-pdo.md)。
