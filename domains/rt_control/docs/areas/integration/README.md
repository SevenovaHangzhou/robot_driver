# integration — 启动组合、接口适配与诊断

**范围**：启动组合、接口适配与诊断。本区对应一个同名职责agent。

**Owner路径**：rt_control_bringup其余部分；control_api_adapter（vacuum_adapter除外）；src/interfaces/**。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：config/machines及alfa_v3.yaml归hw-config；vacuum_adapter及其专属测试归peripheral-control；控制器实现归其控制区。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| contract/02#F1 | vendored 权威 RT-Control 视图固定 /joint_states=125 Hz | [contract-20260814-01](../../areas-archive/contract/records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F1 | 有效 |
| contract/02#F2 | src/interfaces 只保留域内私有接口，公共 schema 从 vendor 构建 | [contract-20260814-01](../../areas-archive/contract/records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F2 | 有效 |
| contract/02#F3 | 跨域 Topic 使用 robot_interfaces_qos 命名 profile | [contract-20260814-01](../../areas-archive/contract/records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F3 | 有效 |
| contract/03#F1 | ErrorInfo 是跨域公共载荷，code 唯一采用 uint32 DREE | [contract-20260816-01](../../areas-archive/contract/records/2026-08-16-bq137-shared-error-readiness.md)#F1 | 有效 |
| contract/03#F2 | 当前 DomainReadiness 为最终公共字段集并遵守一致性规则 | [contract-20260816-01](../../areas-archive/contract/records/2026-08-16-bq137-shared-error-readiness.md)#F2 | 有效 |
| contract/09#F1 | diagnostics topology 从所选 hardware descriptors 派生；公共 status adapter 只消费稳定总线 summary，不持有固定电机数组 | [release-deploy-20260903-01](../../areas-archive/release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F3 | PASS（T1 Docker/Mock） |
| contract/10#F1 | pinned `ros2_canopen` 的完整零值 error-reset EMCY 文本归一化为 cleared；其它非空或格式漂移继续 fail closed | [contract-20260904-01](../../areas-archive/contract/records/2026-09-04-cleared-emcy-readiness.md)#F1-F2 | PASS（T3 Native；PR CI/Docker 待完成） |
| contract/13#F1 | ELECTRI-133 新增六个 RT 私有 chassis 类型；真实 ROS action/service 验证由无设备 Mock 与 opt-in loaned-interface adapter 提供，不增加域外消费者或 production 准入 | [Tier A 私有协议](../../areas-archive/contract/records/2026-09-25-tier-a-private-chassis-protocol.md)#F1-F3 | PARTIAL（T1；实机 PDO/标定/IMU 待验） |
| contract/14#F2 | PP、头部、升降使用位置 Action；未配置资源 fail closed | [V3 公共接口实现](../../areas-archive/contract/records/2026-10-02-v3-interface-contract-implementation.md)#F2 | PARTIAL（T1；实物配置待补） |
| lifecycle/04#F1 | 原位入口仅开放控制字，enable_only 省略 JTC 切换并保留驱动状态/故障/失能；16轴实测成功，默认运动栈不变 | [原位使能入口](../../areas-archive/lifecycle/records/2026-09-12-stationary-enable-entry.md) | PARTIAL（T1/T3，完整运动/交付待验） |
| motion/F8 | V3 FJT/rolling 固定 7+7 CSP 轴序，PP 夹爪不进入运动 writer；初始周期为 1 ms、maximum 2 ms | [V3 双七轴运动 runtime](../../areas-archive/motion/records/2026-09-19-v3-arm-fjt-rolling-runtime.md)#F1-F3 | PARTIAL（T1 Mock；标定/目标机/实机待验） |
| motion/F11 | PP、头部、升降采用绝对位置 Action；缺少实物限位/停止保持时不准入 | [位置 Action 适配](../../areas-archive/motion/records/2026-10-02-position-action-adapters.md)#F1-F2 | PARTIAL（T1；BQ-152） |
| motion/F12 | 不实现底盘与双臂/升降/PP跨组互斥；同资源 writer 和 FJT/Rolling 互斥保留 | [位置 Action 适配](../../areas-archive/motion/records/2026-10-02-position-action-adapters.md)#F3 | PASS（源码/契约；联合运行待验） |
| motion/F13 | 组合运行只保留一套 RSP，每关节状态与 TF 边保持唯一来源 | [位置 Action 适配](../../areas-archive/motion/records/2026-10-02-position-action-adapters.md)#F4 | PARTIAL（入口已支持；完整组合待验） |
| release-deploy/07#F1 | 三代机底盘 scope 显式绑定 swerve_controller 与四个必需外置编码器；只有静态控制计划，真实后端/标定未准入 | [ELECTRI-117 底盘模块控制器绑定](../../areas-archive/release-deploy/records/2026-09-13-swerve-controller-module-binding.md)#F1-F3 | UNVERIFIED（T0 配置；无实机/容器启动） |
| release-deploy/V3-ARM-RUNTIME-1 | V3 机械臂运动入口显式选择，JTC/rolling 先 INACTIVE、enable_manager 后 ACTIVE；默认入口仍为 validation | [V3 arm runtime 入口](../../areas-archive/release-deploy/records/2026-09-19-v3-arm-runtime-entry.md)#F1-F3 | PARTIAL（T1 Mock；旧机/真实总线未验） |
| release-deploy/01#F1 | bringup 只选择并组合 EtherCAT/CANopen 两个 package-owned real/mock system，配置对齐在 Node 创建前 fail closed | [release-deploy-20260903-01](../../areas-archive/release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F1-F3 | PASS（T1 Docker/Mock） |
| release-deploy/04#F1 | swerve_driver 已进入 bringup 构建闭包，但未加入生产启动；增量容器测试不替代真实后端及整机验收 | [舵轮包集成](../../areas-archive/release-deploy/records/2026-09-09-swerve-package-integration.md) | PARTIAL（T1 增量容器） |

| integration-20261013-01#F1–F4 | M0b 两包及九份 Draft 7 草案；模块私有 spec、设备 TBD、XML/GPIO 投影契约与离线 YAML 边界；不改变运行入口 | [M0b schema 与骨架](records/2026-10-13-kkozia-5-m0b-composer-skeleton-and-schemas.md) | UNVERIFIED（T0；A0/运行验证待完成） |

| integration-20261014-02#F1–F4 | 底盘离线双总线组合、共享主站合并、真实缺项拒绝与联合测试迁移 | [第三步底盘组合](records/2026-10-14-chassis-offline-composition.md) | UNVERIFIED（T0；五包576 tests通过，实机/新CI待验） |

| integration-20261014-03#F1–F4 | 头部/力传感离线组合、十份金样、共享HeadPhase与停止点三报告 | [第四步与准入报告](records/2026-10-14-head-force-composition.md) | PARTIAL（头部T1等价；force/原位T1未闭环，默认旧入口） |

| integration-20261014-04#F1–F4 | 用户批准五个默认入口与头部参数；旧V3删旧，V3基线清空，冻结V2文件哈希范围例外获批 | [第五步默认切换](records/2026-10-14-composer-defaults.md) | PARTIAL（712 tests通过；T1缺口与最终CI/实机未闭环） |

| integration-20261015-01#F1–F4 | 底盘事实公共绑定、头部名称注入与力传感器私有结构隔离 | [ELECTRI-183 公共参数绑定](records/2026-10-15-electri-183-public-bindings.md) | UNVERIFIED（T0 定向测试；运行与实机未验） |

| integration-20261015-02#F1–F4 | 控制器参数声明化；部署时生成、启动只校验；EtherCAT 设备族注册表与主站 profile | [ELECTRI-184 声明式组合与生成物](records/2026-10-15-electri-184-declarative-composer-artifacts.md) | UNVERIFIED（T0 定向测试；部署/实机未验） |

## 跨区事实关联

| 当前来源坐标 | 事实 | 原始记录 | 验证状态 |
| --- | --- | --- | --- |
| integration-20261014-01#F1–F5 | 手臂四层离线组合、六份金样、opt-in 入口；手臂 Mock 状态/占用等价，原位 Mock 查询待闭环 | [第二步组合记录](records/2026-10-14-arm-offline-composition.md) | PARTIAL（T0/T1；默认旧入口，实机未验） |

控制器实现见[arm-control](../arm-control/README.md)；生命周期见[protocol](../protocol/README.md)；机型组合输入见[hw-config](../hw-config/README.md)。同一原始记录跨资产事实按对应区引用。
