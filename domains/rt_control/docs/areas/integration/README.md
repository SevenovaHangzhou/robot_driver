# integration — 启动组合、接口适配与诊断

**范围**：启动组合、接口适配与诊断。本区对应一个同名职责agent。

**Owner路径**：rt_control_bringup其余部分；control_api_adapter（vacuum_adapter除外）；src/interfaces/**；rt_diagnostics；runtime_log_recorder（状态接口到共享内存的被动记录插件）。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：config/machines及alfa_v3.yaml归hw-config；vacuum_adapter及其专属测试归peripheral-control；控制器实现归其控制区。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| integration-20261029-08#F1 | 作业四轮显式允许CSV/CSP，启动仍固定CSV；交接成功继续依赖控制器四轮反馈及ACK | [作业切换监控修复](records/2026-10-29-chassis-operation-zero.md) | PARTIAL（515结果通过；实机连续两轮零位移成功，非零运动未验） |
| integration-20261029-07#F1 | 初始NAV轮速零确认与CSV/CSP全轮系静止交接分开，原始反馈按有限范围和持续时间判断；不恢复旧指令 | [导航启动及作业静止范围](records/2026-10-29-chassis-steering-startup.md) | PARTIAL（T3 NAV零速度20秒通过；作业运动未验） |
| integration-20261029-06#F1–F2 | 实机启动通过，使能后实时更新故障；NAV未就绪，OP和零位移未执行，不能外推T1结果 | [实机零指令检查](records/2026-10-29-chassis-real-zero-check.md) | PARTIAL（T3 检查失败，故障未使能） |
| integration-20261029-05#F1–F3 | 网页配置单个相对位移任务，控制器限制回读和实际准入；停止/释放/断网请求取消，NAV启动不变 | [网页作业位移](records/2026-10-29-web-operation-motion.md) | PARTIAL（T1 无设备；实机运动待验） |
| contract/02#F1 | vendored 权威 RT-Control 视图固定 /joint_states=125 Hz | [contract-20260814-01](../contract/records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F1 | 有效 |
| contract/02#F2 | src/interfaces 只保留域内私有接口，公共 schema 从 vendor 构建 | [contract-20260814-01](../contract/records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F2 | 有效 |
| contract/02#F3 | 跨域 Topic 使用 robot_interfaces_qos 命名 profile | [contract-20260814-01](../contract/records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F3 | 有效 |
| contract/03#F1 | ErrorInfo 是跨域公共载荷，code 唯一采用 uint32 DREE | [contract-20260816-01](../contract/records/2026-08-16-bq137-shared-error-readiness.md)#F1 | 有效 |
| contract/03#F2 | 当前 DomainReadiness 为最终公共字段集并遵守一致性规则 | [contract-20260816-01](../contract/records/2026-08-16-bq137-shared-error-readiness.md)#F2 | 有效 |
| contract/09#F1 | diagnostics topology 从所选 hardware descriptors 派生；公共 status adapter 只消费稳定总线 summary，不持有固定电机数组 | [release-deploy-20260903-01](../release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F3 | PASS（T1 Docker/Mock） |
| contract/10#F1 | pinned `ros2_canopen` 的完整零值 error-reset EMCY 文本归一化为 cleared；其它非空或格式漂移继续 fail closed | [contract-20260904-01](../contract/records/2026-09-04-cleared-emcy-readiness.md)#F1-F2 | PASS（T3 Native；PR CI/Docker 待完成） |
| contract/13#F1 | ELECTRI-133 新增六个 RT 私有 chassis 类型；真实 ROS action/service 验证由无设备 Mock 与 opt-in loaned-interface adapter 提供，不增加域外消费者或 production 准入 | [Tier A 私有协议](../contract/records/2026-09-25-tier-a-private-chassis-protocol.md)#F1-F3 | PARTIAL（T1；实机 PDO/标定/IMU 待验） |
| contract/14#F2 | PP、头部、升降使用位置 Action；未配置资源 fail closed | [V3 公共接口实现](../contract/records/2026-10-02-v3-interface-contract-implementation.md)#F2 | PARTIAL（T1；实物配置待补） |
| lifecycle/04#F1 | 原位入口仅开放控制字，enable_only 省略 JTC 切换并保留驱动状态/故障/失能；16轴实测成功，默认运动栈不变 | [原位使能入口](../lifecycle/records/2026-09-12-stationary-enable-entry.md) | PARTIAL（T1/T3，完整运动/交付待验） |
| motion/F8 | V3 FJT/rolling 固定 7+7 CSP 轴序，PP 夹爪不进入运动 writer；初始周期为 1 ms、maximum 2 ms | [V3 双七轴运动 runtime](../motion/records/2026-09-19-v3-arm-fjt-rolling-runtime.md)#F1-F3 | PARTIAL（T1 Mock；标定/目标机/实机待验） |
| motion/F11 | PP、头部、升降采用绝对位置 Action；缺少实物限位/停止保持时不准入 | [位置 Action 适配](../motion/records/2026-10-02-position-action-adapters.md)#F1-F2 | PARTIAL（T1；BQ-152） |
| motion/F12 | 不实现底盘与双臂/升降/PP跨组互斥；同资源 writer 和 FJT/Rolling 互斥保留 | [位置 Action 适配](../motion/records/2026-10-02-position-action-adapters.md)#F3 | PASS（源码/契约；联合运行待验） |
| motion/F13 | 组合运行只保留一套 RSP，每关节状态与 TF 边保持唯一来源 | [位置 Action 适配](../motion/records/2026-10-02-position-action-adapters.md)#F4 | PARTIAL（入口已支持；完整组合待验） |
| release-deploy/07#F1 | 三代机底盘 scope 显式绑定 swerve_controller 与四个必需外置编码器；只有静态控制计划，真实后端/标定未准入 | [ELECTRI-117 底盘模块控制器绑定](../release-deploy/records/2026-09-13-swerve-controller-module-binding.md)#F1-F3 | UNVERIFIED（T0 配置；无实机/容器启动） |
| release-deploy/V3-ARM-RUNTIME-1 | V3 机械臂运动入口显式选择，JTC/rolling 先 INACTIVE、enable_manager 后 ACTIVE；默认入口仍为 validation | [V3 arm runtime 入口](../release-deploy/records/2026-09-19-v3-arm-runtime-entry.md)#F1-F3 | PARTIAL（T1 Mock；旧机/真实总线未验） |
| release-deploy/01#F1 | bringup 只选择并组合 EtherCAT/CANopen 两个 package-owned real/mock system，配置对齐在 Node 创建前 fail closed | [release-deploy-20260903-01](../release-deploy/records/2026-09-03-port-hardware-composition-to-main.md)#F1-F3 | PASS（T1 Docker/Mock） |
| release-deploy/04#F1 | swerve_driver 已进入 bringup 构建闭包，但未加入生产启动；增量容器测试不替代真实后端及整机验收 | [舵轮包集成](../release-deploy/records/2026-09-09-swerve-package-integration.md) | PARTIAL（T1 增量容器） |

| integration-20261013-01#F1–F4 | M0b 两包及九份 Draft 7 草案；模块私有 spec、设备 TBD、XML/GPIO 投影契约与离线 YAML 边界；不改变运行入口 | [M0b schema 与骨架](records/2026-10-13-kkozia-5-m0b-composer-skeleton-and-schemas.md) | UNVERIFIED（T0；A0/运行验证待完成） |

| integration-20261014-02#F1–F4 | 底盘离线双总线组合、共享主站合并、真实缺项拒绝与联合测试迁移 | [第三步底盘组合](records/2026-10-14-chassis-offline-composition.md) | UNVERIFIED（T0；五包576 tests通过，实机/新CI待验） |

| integration-20261014-03#F1–F4 | 头部/力传感离线组合、十份金样、共享HeadPhase与停止点三报告 | [第四步与准入报告](records/2026-10-14-head-force-composition.md) | PARTIAL（头部T1等价；force/原位T1未闭环，默认旧入口） |

| integration-20261014-04#F1–F4 | 用户批准五个默认入口与头部参数；旧V3删旧，V3基线清空，冻结V2文件哈希范围例外获批 | [第五步默认切换](records/2026-10-14-composer-defaults.md) | PARTIAL（712 tests通过；T1缺口与最终CI/实机未闭环） |

| integration-20261015-01#F1–F4 | 底盘事实公共绑定、头部名称注入与力传感器私有结构隔离 | [ELECTRI-183 公共参数绑定](records/2026-10-15-electri-183-public-bindings.md) | UNVERIFIED（T0 定向测试；运行与实机未验） |

| integration-20261015-02#F1–F4 | 控制器参数声明化；部署时生成、启动只校验；EtherCAT 设备族注册表与主站 profile | [ELECTRI-184 声明式组合与生成物](records/2026-10-15-electri-184-declarative-composer-artifacts.md) | UNVERIFIED（T0 定向测试；部署/实机未验） |
| integration-20261020-01#F1–F2 | 研华 V3 底盘启动须先确认 EtherCAT 8 从站在线；网页为独立临时服务 | [底盘现场启动 README](records/2026-10-20-v3-chassis-start-readme.md) | PARTIAL（既有现场启动日志与 HTTP 200；本次仅文档静态检查） |

| integration-20261029-03#F1–F3 | 单机作业显式启用，NAV启动不变；四驱动真实交接ACK、已有接收IMU和导航单位换算保持准入条件，估计限制须实机验收 | [作业运行配置](records/2026-10-29-chassis-operation-runtime.md) | PARTIAL（T1 无设备；实机切换/运动待验） |
| integration-20261029-04#F1–F3 | 初次CSV零速确认有界等待反馈；年龄反推时刻保留原值、序号仍验顺序；准入拒绝保持inactive，运行故障仍走owner失能 | [使能启动修复](records/2026-10-29-chassis-enable-startup.md) | PARTIAL（T1 回归与归档反馈离线重放；实机重载/使能待验） |

## 跨区事实关联

- [网页作业位移](records/2026-10-29-web-operation-motion.md)：配置运动、限制回读、实际反馈和取消；T1 无设备端到端通过，实机运动待验。

- [使能启动与重试修复](records/2026-10-29-chassis-enable-startup.md)：启动零速确认有界等待PDO、年龄估计抖动处理、激活拒绝保持inactive；保留运行故障保护与owner失能链路，现场重载/使能待验。
- [复用导航配置接通作业运行](records/2026-10-29-chassis-operation-runtime.md)：显式单机部署、四驱动切换、实际 IMU 及正式产物接通；构建/离线联合测试通过，实机切换和低速运动待验。
- [操作站导航/作业切换](records/2026-10-29-operator-chassis-mode.md)：已有私有服务回读确认、速度入口互斥与未配置拒绝；无设备验证通过，实机作业准入待验。

| 当前来源坐标 | 事实 | 原始记录 | 验证状态 |
| --- | --- | --- | --- |
| integration-20261014-01#F1–F5 | 手臂四层离线组合、六份金样、opt-in 入口；手臂 Mock 状态/占用等价，原位 Mock 查询待闭环 | [第二步组合记录](records/2026-10-14-arm-offline-composition.md) | PARTIAL（T0/T1；默认旧入口，实机未验） |

控制器实现见[arm-control](../arm-control/README.md)；生命周期见[protocol](../protocol/README.md)；机型组合输入见[hw-config](../hw-config/README.md)。同一原始记录跨资产事实按对应区引用。
