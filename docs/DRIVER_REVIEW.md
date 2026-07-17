# 电控驱动层代码评审报告

> 审查对象：`feature/unsafe-direct-motion-entry` @ `07c87e5`（2026-07-16，当前最新开发线 = `feature/ethercat-motor-driver` 全部历史 + 12 个 unsafe-direct 提交）
> 审查方法：4 路并行逐行审查（EtherCAT 驱动 / CANopen 主站 / lifecycle 编排层 / bringup 与外围）+ 交叉验证，所有发现均有 文件:行号 支撑
> 对照基准：`docs/ARCHITECTURE.md`（本仓库整机软件架构设计）

---

## 0. 总评

**先说做对了什么（这些要保住）：**

- **安全兜底的层级放对了**：Python 层只做"授权"（permit token），真正的强制执行在 C++ 实时域——CANopen/EtherCAT 硬件插件逐周期解码 permit，token 停止翻转 ≤200ms 后 lease 失效、命令被抑制。任何 Python 进程崩溃的后果是**失去运动能力（fail-closed），不是失去安全**。这套"驱动器侧强制执行的 command permit lease"是这套代码里最有价值的发明。
- 控制循环实时纪律达标：CANopen `read()/write()` 无阻塞 SDO、无锁、非阻塞 socket；EtherCAT fork 清掉了上游周期路径的 printf，策略类（分批使能/DC 监控/transport 健康）做成 ROS-free 纯状态机且单测充分。
- 工程纪律高于平均：PATCHES.md 逐条记录动机/证据/未验收项、CiA 402 状态机实现正确、测试密度大、调试脚本自带安全限幅（±3°/dry-run/确认门）、bms_node 实现质量良好、文档与代码一致性很高。

**三个结构性风险（比任何单个 bug 都重要）：**

1. **真机现场实际跑的是 `unsafe_direct_motion` 旁路，不是精心设计的 managed 安全架构。** 最近三周的提交都在加固旁路（PP 目标重发、重试、bringup 硬化），而不是把 managed 路径修到能用——"临时旁路"正在固化为事实产品形态。该模式下 fault/EMCY 期间继续满速发目标、bus-off 只打日志、permit/安全停车/指令保鲜全部失效。
2. **主站进程死亡时履带会以最后速度永续运转（CANopen 侧无任何驱动器侧失联保护）。** 主站不发心跳、不配 0x1016 consumer heartbeat、无节点守护，PP 帧还做了去重（稳态静默）；所有保护都跑在"可能死掉的那个进程"里。`docs/interfaces.md:117-118` 自己承认了这一点。这是移动机器人最经典的 runaway 场景。
3. **EtherCAT 13 轴对上层轨迹零防御 + 故障恢复路径有一颗必踩的雷。** 软件层无任何位置/速度限位（0x607D 未下发、joint_limits.yaml 运行时无人消费、JTC 无容差约束）；且 CSP 目标预装载跨激活会话不复位——任何一次"故障→on_error→重新 activate"的正常恢复流程中，若停机期间臂被移动/重力下沉，二次使能瞬间驱动器会以全动态追上一会话的旧目标。

**一句话结论**：这套代码可以支撑**有人盯着的、低速小幅的使能与点动验收**；在修掉下面 P0 清单、补上断电/拔线实测、并给 unsafe 模式排出退役计划之前，不具备无人值守或全速度域长稳运行的条件。

---

## 1. P0：上真机长稳前必须修（按危险度排序）

### P0-1 EtherCAT：CSP 预装载跨激活会话不复位 → 二次使能瞬间跳变

- `ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp:475-481`：`target_position_preloaded_ = true` 是全仓唯一赋值点，无任何复位；`ethercat_driver.cpp:1205-1296`（shutdownAndDeactivate）与 `:1382`（on_activate）都不做插件级会话复位。
- 后果：P1 补丁的核心安全机制（使能前 0x607A 初始化为当前 0x6064）只在进程内**第一次**激活生效。第二会话从第一个写周期起发送**上一会话的旧目标**；批次使能放行前也没有 |0x607A−0x6064| 复检。
- 修法：给 `EcSlave` 加显式会话复位钩子（清 preload/actual_seen/epoch 标志），在 deactivate 末尾或 activate 开头对全部模块调用；使能放行前加"预装载新鲜度 + |target−actual|≤阈值"门，超差重新预装载；补"第二次 activate"单测（当前无覆盖）。

### P0-2 CANopen：主站失联无驱动器侧故障反应 → 履带 runaway

- 全仓无主站心跳帧构造、无 0x1016 写入（grep 零命中）；PP 帧去重后稳态静默（`canopen_runtime.cpp:1005-1032`）。
- 修法：① 确认 LD2-CAN/Leadshine 的通信看门狗对象（vendor RPDO timeout 或 0x1016+主站 0x700 心跳），纳入现有 `configureSafetyOd_` 的"写入+精确读回"框架，并写进 `real_driver_permit_v1` 准入合同；② PV 轴每周期发帧天然喂狗；PP 轴加低频 keep-alive（每 N 周期重发锁存目标，controlword 0x000F 不带 bit4）。
- 同族问题：EtherCAT 侧 SM 看门狗只用从站上电默认值，全仓无 `ecrt_slave_config_watchdog`（分频/超时未按设计定值），"拔线→两族驱动器进入故障反应+抱闸"从未实测（PATCHES P15 自认）。**把拔线/断电实测列为验收硬项。**

### P0-3 EtherCAT：全链路无软件限位/限幅

- 0x607D 未进 startup SDO（icube_profiles 全 grep 无命中）；`joint_limits.yaml` 只被 launch 静态校验消费（`dual_arm_ethercat_control.launch.py:296-299`），运行时无人加载；运行时 URDF 无 kinematic joint/`<limit>`；`controllers.yaml:35-59` JTC 无每关节容差。唯一量值闸门是 0.0175 rad 接管容差，接管后每周期命令无步长钳制直写 0x607A。
- 修法（三层同时做）：(a) 按 SPEC 把 0x607D:01/02（由 joint_limits 推导、含零位偏移换算）、ZeroErr 0x6065/0x6067、Ti5 0x6081 放进 startup SDO 并读回；(b) `EcCiA402Drive` 写 0x607A 前加每周期 Δ 钳制（max_velocity×cycle+裕量）与软限位夹紧，超差 hold+上报；(c) JTC 配置每关节 `constraints` 容差。对齐 CANopen 侧已有的硬件层二次 clamp（那边做对了）。

### P0-4 unsafe_direct_motion 模式的收敛计划

- `canopen_runtime.cpp:872`（fault/EMCY/quick-stop/not-ready 抑制整体 `continue`）、`canopen_system_hardware.cpp:1016-1018/1060-1064`（bus-off/TX 失败不 ERROR）、`ethercat_driver.cpp:1779-1795/1880-1889`（cyclic IO 异常吞掉重试）。
- 修法：短期先把 **bus-off 与 EMCY≠0 两个条件从旁路中剔除**（unsafe 只旁路"软件运动门"，不旁路"总线/驱动器硬故障"）；中期列出 managed 路径当初在真机起不来的原因清单（从提交序列看：fault 残留、readiness 卡死、0x60C2/命令门时序），逐项修复并给 unsafe 排退役里程碑；`robot_status` 应把该模式恒定上报为降级状态。

### P0-5 quick-stop 状态机死锁：停止在驱动器未使能时触发则永远无法确认、无法 resume

- `robot_lifecycle/stop_core.py:359-363`（resume 仅接受 QUICK_STOP_ACTIVE/RESUME_FAILED）+ `:273-279`（确认唯一途径）+ `robot_status/stop_status.py:63-66`（要求全轴 `drive_state == QUICK_STOP_ACTIVE`）。CiA 402 只有 Operation Enabled 的驱动器才会进入 Quick Stop Active；且 605A 默认不写（`configure_safety_od_on_activate` 默认 false，全仓无生产配置设置 quick-stop option code），多数厂商默认 605A=2 时该状态只在斜坡期短暂可见，10Hz 采样极易错过。
- 后果：stop manager 永久卡 `QUICK_STOP_REQUESTED`，resume 被拒，permit 排除 chassis，ENABLE 卡死——唯一出路是**重启安全节点**（最坏的现场操作激励）。
- 修法：确认谓词改为析取："QUICK_STOP_ACTIVE **或**（safety_stop 命令驱动侧回读确认 + 全轴零速/未使能）"；C++ 暴露 safety_stop 回执接口；生产 xacro 固化 605A（5–8）与 6085 并读回。

### P0-6 升降（重力轴）停机时序开环

- `canopen_system_hardware.cpp:2079-2137`：QuickStop → 固定 sleep 200ms → Shutdown（撤扭矩）→ 20ms → DisableVoltage，全程不确认停稳、不确认抱闸（0x60FE/厂商抱闸对象未涉及）。若 6085 减速度不足或负载偏大，撤扭矩瞬间升降轴在抱闸咬合延迟窗口内自由下坠。
- 修法：QuickStop 与 Shutdown 之间 bounded 轮询 statusword/6064 确认零速；commissioning 阶段确认驱动抱闸自动模式；固定延时改"上限等待+条件满足即推进"。对照架构文档 §4.2："重力轴必须掉电即抱 + 时序确认"。

### P0-7 joint6 零位偏移后限位映射超出单圈编码器计数域

- `icube_profiles/right_joint6.yaml:12`（offset=467955）/`left_joint6.yaml:12`（offset=70624）+ `joint_limits.yaml` ±3.12414 rad：right_joint6 上限映射 raw ≈ 728,643 > 524,288（单圈计数域），left_joint6 下限 ≈ −190,064 < 0。若 0x6064/0x607A 是单圈语义，合法角度内反馈会 ±2π 跳变、目标越界，叠加 P0-1 可能整圈追位。`set_joint6_current_zero.py:84-141` 只改 offset 不重算限位；`test_config_tables.py` 不验证限位映射落在计数域内。
- 修法：实机确认单圈/多圈语义；单圈则脚本同时收缩 lower/upper 并留余量、测试加"限位不出计数域"断言、TPDO 位置加 unwrap 或越界即 fault。

---

## 2. P1：高优先（长稳与可用性）

| # | 子系统 | 问题 | 位置 | 修法要点 |
|---|---|---|---|---|
| P1-1 | EtherCAT | SYNC0 相位与 CM 写入相位解耦，目标到达时刻相对锁存点不可控 → 低频纹波/偶发跟随误差 | `ec_master.cpp:99-105`（相位一次性采样）；运行时用 `now()` 作应用时间 | sync0_shift 定义为"CM 写完成时刻+固定裕量"；应用时间用期望 deadline；示波器验收写入→SYNC0 裕量分布 |
| P1-2 | EtherCAT | `on_error` 在调用线程阻塞跑 1s+ 关断循环（含日志/字符串分配）；RT 路径存在事件沿日志 | `ethercat_driver.cpp:1205-1296, 1735-1751, 1845-1902` | 终态关断迁独立低优先级线程；RT 日志改"置标志+诊断线程输出"。（双 CM 分进程部署已隔离履带侧，此项影响限于臂自身） |
| P1-3 | CANopen | PP setpoint 无确认闭环（bit12 无效且无替代确认），managed 去重后单次丢失不可恢复不可检测 | `canopen_runtime.cpp:15, 1009-1032` | setpoint 后监督 `|6064−target|`：超时未收敛且零速 → 重脉冲+计数告警；把 coalesce/重发计数暴露到诊断 |
| P1-4 | CANopen | 履带位置积分按"到帧周期"欠计（TPDO 慢于控制周期时 5 倍级低估）；PP 速度求导用错 Δt；靠 `position_feedback:false` 掩盖，但整机 yaml **没设**该项——两份 real 配置 odom 数学不一致 | `canopen_runtime.cpp:625-640`；`canopen_real_hardware_controllers.yaml:29-59` vs `canopen_crawler_real_hardware_controllers.yaml:36-39` | 积分/求导用"距上次反馈实际时间"，或直接消费 TPDO3 字节 0-3 的驱动器位置（含回绕）；整机 yaml 立即补 `position_feedback:false` 对齐 |
| P1-5 | 拓扑 | `/cmd_vel` 无仲裁：teleop 与导航多 publisher 只靠文字约定，teleop 的双 publisher 自杀检测是单侧自愿行为 | `alfa_robot_mixed_real_hardware_control.launch.py:181-190`；README:234 | 补 twist_mux（急停恢复>遥操作>docking>导航），对照架构文档 §8.4 |
| P1-6 | lifecycle | 运动的持续前提是 50Hz Python 心跳（permit token 翻转）：一次 >200ms 的 GC/调度抖动 → 全机 permit 掉落走 guarded rearm；merged `/joint_states`（全机 TF 唯一来源）同样由 Python 50Hz 发布 | `lifecycle_node.py:194-196`；`status_node.py:167-173` | 中期把 lease 心跳下沉为 ~30 行 C++ permit heartbeat controller（Python 只做 arm/disarm 电平）；joint_states 合并交还两个 CM 的 broadcaster + robot_state_publisher 标准链 |
| P1-7 | EtherCAT | 运行时对 0x6061 实际模式与 0x603F 故障码全盲（选定 PDO 映射不含，且无低频 SDO 轮询补偿） | `generic_ec_cia402_drive.cpp:394-397`；profiles 无 0x6061/0x603F | ZeroErr 改选含两者的 0x1A06 映射（SPEC 自荐）；Ti5 用非实时线程低频 SDO 轮询注入诊断 |
| P1-8 | lifecycle | 新 status publisher 建立信任前，运行期故障监测有 2s 盲窗且"not established"诊断为死分支 | `runtime_failure_monitor.py:203-216` | untrusted 即返回非空 stale_reason，让 permit 关闭计时器立即启动 |
| P1-9 | 拓扑 | 软件对安全链"零可见"（STO/急停状态无只读接口，`/robot_status` 无法呈现），A4 类验收只能人工判断 | 全仓 grep 无 STO/急停读取；`task_plan.md:122,173` | 安全 PLC/急停回路若有 DI/Modbus 出口，加只读映射进 `/robot_status`（对照架构文档 §4.6"只读不写"） |

---

## 3. P2：中优先（工程质量与防翻车）

- **EtherCAT direct+strict 模式跳过全部 13 轴准入校验**（`ethercat_driver.cpp:369-413`：未声明 marker 时直接放行）→ strict 模式无条件启用 admission marker。
- **`on_init` 插件加载失败被吞**（`ethercat_driver.cpp:258-263` catch 后继续循环不返回 ERROR）→ direct 模式 12/13 轴静默启动、缺轴 NaN 进 JTC → catch 内 `return ERROR` + 断言模块数。
- **`control_frequency` 缺省 100Hz 且不与 CM update_rate 交叉校验**（`ethercat_driver.cpp:1391`）→ 不一致即 FATAL，缺省改拒绝。
- **CANopen 单帧 TX 失败（含瞬态 ENOBUFS）即整机 ERROR**（`canopen_system_hardware.cpp:1050-1064`）→ ENOBUFS/EAGAIN 给 1 周期重试窗，连续 N 周期再升级。
- **PP 轴在 hard_stop/EMCY 下主站一帧不发**（`canopen_runtime.cpp:919-922`），QuickStopActive 会话内不可恢复 → hard_stop 时对 PP 轴也发 quick-stop 控制字；给 PP 补受控恢复路径或明确"需重新激活"诊断。
- **接收路径 32 帧/周期上限无积压观测**（`canopen_runtime.cpp:18`）→ 排空至 EAGAIN + `SO_RXQ_OVFL` 丢帧统计 + TPDO inhibit/event-timer 进 commissioning 校验清单。
- **节点 boot-up 后无诊断**（重启驱动器 pre-op，"莫名不动"只能倒查）→ ingestFrame 检出 boot-up 置 latched ERROR 诊断。
- **stop 请求历史满时幂等保护静默失效**（`stop_core.py:440-448`）→ 无法驱逐时驱逐最旧并记日志。
- **`count_publishers()>1` 自杀在 DDS 发现残留下形成重启风暴**（`lifecycle_node.py:383-402`、`stop_node.py:176-187`；SIGKILL 后 stale endpoint ~20s）→ 启动期文件锁互斥 + 运行期降级为"告警+permit 关闭"+ 消抖。
- **resume 驱动侧放行早于 manager 健康确认**（`stop_core.py:347-353` + `canopen_runtime.cpp:877-879`）→ 放行移到确认 IDLE 之后，或与 permit rearm 序列绑定。
- **plc_node 每次写输出无条件抬升 PLC 远程使能位**（`plc_node.py:250-254`）→ 改显式服务触发；非使能默认拒绝写并报警（防止绕过现场检修隔离意图）。
- **mock 与 real 两套关节命名/拓扑不同构**（`rightjoint1` vs `right_joint1`；mock 双臂直挂 base_link）→ 统一命名与限位，用测试锁定两图同构。
- **协调使能组的 RT 路径全局互斥锁+静态 map 仍在产品代码**（`generic_ec_cia402_drive.cpp:43-44,1270,1294`；已确认弃用）→ 物理删除或编译期隔离。
- **can_bus_guard 锁文件在 /tmp 且 0600**（PrivateTmp 下互斥失效、root/非 root 混跑异常）→ 移 `/run/lock/` 定权限策略。

---

## 4. 架构级评估与收敛建议

### 4.1 双 controller_manager：接受为正式 ADR，但补齐代价说明

现状：EtherCAT `/controller_manager`（250Hz）+ CANopen `/canopen/controller_manager`（50Hz）两进程，文档明确"隔离是有意设计"。**建议接受这条对架构文档 §5.4（单 CM 双 hardware）的偏离**：250/50Hz 双频、独立故障域、admission/namespace 合同都已围绕双 CM 建立，现在合并是高成本低收益。但要把代价写进 ADR：臂/底盘无共同实时上下文，跨子系统联动走 ROS 层（数百 ms 级），**因此臂-底盘互锁的 L1 强制执行（架构文档 §9.6）需要在两个 CM 里各自实现自己那半边**（臂侧"底盘未停稳不放行"、底盘侧"双臂未收拢钳零"各自独立判定），不能依赖跨 CM 的软件回路。

### 4.2 robot_lifecycle：保留合同，砍掉引擎（3.6 万行 → ~1500 行的路线）

这层用 1.5 万行生产代码 + 1.8 万行测试实现了一套自研分布式事务引擎（根事务/lane/补偿、request ledger 封签、进程内 HMAC 签名的 admission evidence、代际 fence），服务于 3 个 subsystem——超出实际需求约一个数量级，且 ledger 封签后（256 条）**不重启进程就不再接受 ENABLE**，长期运行必撞。分五步收敛（每步可独立验证，风险递减）：

1. 先修 P0-5/P1-8/P2 里的 stop/monitor 缺陷（不动架构）+ 生产 xacro 固化 605A/6085；
2. invariant 全量断言移出生产 tick（现每 50Hz tick 多次全量快照重建）；`/joint_states` 合并交还标准 broadcaster 链；
3. 契约唯一源化：从 xacro 展开自动生成 admission 期望表与 status topology（现在同一套轴契约硬编码在 URDF/C++/Python/yaml/测试**五处**）；
4. 用 launch+spawner+lifecycle service 重写 START/ENABLE/DISABLE（保持对外 service 与 `/robot_lifecycle/status` 消息不变，旧 executor 并存一个版本周期）；
5. 测试迁移：协议内部语义测试换成对外行为测试 + 真正缺失的 controller_manager 进程级集成测试。

**保留**：C++ permit lease/safety_stop/clamp 全套、`bounded_controller_spawner`、robot_status 只读聚合语义、stop manager 合同（latched/manual-resume/零速 guard）。

### 4.3 自研 CANopen 栈：不迁移，但把"隐式约定"变成代码资产

自研在当前域（3-4 节点、固定映射、50Hz）是可辩护的，测试质量高于 ros2_canopen 现状。真正欠账恰是成熟栈免费提供的：主站心跳、0x1016、EDS/DCF 驱动的 PDO 映射下发与读回校验、SYNC。建议扩展现有 `configureSafetyOd_`"写入+精确读回"框架覆盖 PDO 通信/映射参数、event-timer/inhibit、1016/1017、厂商看门狗对象——激活时校验不符即拒绝。PDO 映射目前是"厂商工具离线配好+代码隐式假设+真机试错"（TPDO3 修复就是试错产物），这类问题应在此框架下根治。

### 4.4 EtherCAT fork：承认硬分叉，补记 provenance

核心文件已是上游 2.5-3 倍行数，升级路径实际已死（PATCHES.md 自认未记录 import 基线 commit）。建议：补记 fork 基线；修复文档漂移（DC 门 PATCHES 写 ≤10µs、代码是 60µs——安全参数记录必须一致；joint6 注释与 offset 矛盾；`ethercat_topology.yaml` 的中文警告拦不住魔法字符串门）；"确认废弃"的机制（协调使能组）物理删除。

### 4.5 分支与工程卫生

- `codex/crawler-nav-integration`、`codex/ethercat-transport-timeout-20260713`、`claude/ethercat-motor-driver-prod-oxeq9u` 的修复已被重放进主线（`29d67fb`/`1f1a46e`/`ea620fd` 等），三条分支是重复历史，确认后删除；统一"修复只进主线、分支短命"的流程。
- `start_ethercat_master.sh:120` 的 `chmod 666 /dev/EtherCAT0` 改组权限；`ethercat_driver.cpp:1349` 死代码 `std::vector<double> test;` 删除；`commandToCounts` 三份近重复实现合并。

---

## 5. 行动顺序建议（对照架构文档路线图）

**第一批（真机全速/无人值守前的硬门槛）**：P0-1 会话复位、P0-3 三层限位、P0-2 驱动侧看门狗（含拔线/断电实测验收）、P0-5 quick-stop 死锁+605A/6085 固化、P0-6 升降停机时序、P0-7 joint6 计数域。这一批对应架构文档 §2.5 失效行为矩阵和 §5.6 看门狗语义的落地——**当前失效矩阵里"B 域崩溃→驱动器自主抱闸"这一行在两条总线上都还只是假设**。

**第二批（收敛运行剖面）**：P0-4 unsafe 模式退役计划（先剔除 bus-off/EMCY 旁路）、P1-3 PP 监督、P1-4 odom 修正与配置对齐、P1-5 twist_mux、P2 的 direct 准入/频率护栏/TX 分级。

**第三批（结构收敛）**：§4.2 lifecycle 五步瘦身、§4.3 CANopen 声明式 provisioning、契约唯一源化、fork provenance 与文档漂移修复、分支清理。

---

*本报告由 4 路并行深度代码审查（各自独立通读对应子系统全部源码与测试）交叉验证后合并；每条发现在原始审查中均含完整代码引用与修复建议。*
