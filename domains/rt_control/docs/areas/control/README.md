# control — 使能状态机、机械臂、头部与底盘控制器

**范围**：控制层控制器：CiA402/握手式使能状态机、机械臂轨迹、力反馈发布、四舵轮控制。2026-10-31 由 arm-control 与 chassis-control 合并而成（用户裁定），对应 `ARCHITECTURE.md` 的 control 层。

**Owner路径**：ctrl_enable_manager；ctrl_rolling_trajectory；ctrl_force_torque_broadcaster；ctrl_swerve；patches/ros2_controllers。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：EtherCAT/CAN协议、编码器提供器与hw_semantic_components归protocol；机型事实/标定（hwcfg_machines）归hw-config；启动拼装归integration；共享库归infra。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的区名（含 arm-control、chassis-control 及旧区名）仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| ecat-axes/04#F1 | X503B raw shadow bridge 只消费 rt-control state frame；V1.6 已定义单位码 `5=N`、`7=N·m`，采样原码采用 `-999999..999999` 范围门禁，实际回读/TF 未完成时禁止 WrenchStamped | [ecat-axes-20260906-01](../../areas-archive/ecat-axes/records/2026-09-06-electri-116-x503b-shadow-bridge.md)#F1-F3 | PARTIAL（T1；目标 SDO/实际参数/TF 待验） |
| ecat-axes/17#F1 | X503 周期发布迁入共享 semantic component + C++ broadcaster。原行中的旧蓝点revision/布局描述归档，不作为当前实物身份 | [统一六维力发布与蓝点可选配置](../../areas-archive/ecat-axes/records/2026-09-17-force-torque-cpp-broadcaster-and-bluepoint-option.md)#F1-F4 | PARTIAL（T1 Mock；X503/蓝点实机待验） |
| lifecycle/20261009-02#F1 | 头部会话已归档；两台临时配置速度0.2rad/s，历史PI比较0.1rad/s | [会话归档](../../areas-archive/lifecycle/records/2026-10-09-damiao-session-readme.md)#F1-F2 | PARTIAL（新速度未加载验证） |
| lifecycle/20261009#F1 | 达妙控制器切换保留 ListControllers 响应生命周期，20次Mock失能通过 | [响应生命周期修复](../../areas-archive/lifecycle/records/2026-10-09-damiao-controller-response-lifetime.md)#F1-F2 | PARTIAL（T1；实机待验） |
| motion/F2 | 官方夹爪 controller 的 PP 扩展默认关闭；启用后按同序号反馈完成到位/取消，max_effort 使用 N，不接受无限力请求 | [PP Action 记录](../../areas-archive/motion/records/2026-09-08-pp-gripper-action.md) | PARTIAL，实机待验 |
| release-deploy/V3-HEAD-CAN-1 | V3 头部使用 `head_joint/head_pitch_joint`、非 JTC ForwardCommand + `/rt/head/*` manager；达妙 T 型由 ACC/DEC/MAX_SPD 承担，参数工具默认不写 flash | [V3 达妙头部非 JTC 位置速度运行链](../../areas-archive/release-deploy/records/2026-09-19-v3-damiao-head-runtime.md)#F1-F4 | PARTIAL（T1 Mock；CAN/机械/整机 full runtime 待验） |
| arm-control-20261014-01#F1–F3 | 显式批次/容差/包络保留旧数值与顺序，provisional 不冒充实测 | [显式手臂策略](records/2026-10-14-explicit-arm-policies.md) | UNVERIFIED（T0；策略/金样验证通过，实机未验） |
| arm-control-20261019-01#F1–F3 | JTC 在非实时回调拒绝超过 `max_trajectory_points`（默认 2000）的轨迹；2000 点时新轨迹周期约 1.17 ms | [JTC 点数上限](records/2026-10-19-jtc-max-trajectory-points.md) | PARTIAL（T1 mock；台架未发轨迹） |
| integration-20261014-03#F1–F4 | 控制包只依赖共享HeadPhase；控制器集合/参数保持，D-b不准入新真实头部 | [第四步记录](../integration/records/2026-10-14-head-force-composition.md) | PARTIAL（头部T1等价；实机未验） |
| lifecycle/20261009-03#F1 | 模式切换成功路径测试持续提供 JTC 状态，保留生产新鲜度与停稳门禁 | [CI 源状态刷新修复](../../areas-archive/lifecycle/records/2026-10-09-mode-switch-ci-source-freshness.md)#F1-F2 | PARTIAL（65用例及30次延迟回归通过；全仓CI待验） |
| lifecycle/01#F1 | /rt/disable 挂起症状真实（4 例日志）；病因指向服务端 waitForResult 30s 自旋+RT 循环未填槽，非串行预算叠加 | [lifecycle-20260814-01](../../areas-archive/lifecycle/records/2026-08-14-rt-disable-log-forensics.md)#F1 | 有效 |
| lifecycle/01#F2 | rt_disable_once 三阶段共享单一绝对 deadline 是已验证现状（29 例日志佐证），非缺陷 | [lifecycle-20260814-01](../../areas-archive/lifecycle/records/2026-08-14-rt-disable-log-forensics.md)#F2 | 有效 |
| lifecycle/02#F1 | enable_manager 单元测试基线 = 36 用例 @ 3ff153d（a–f 六项）；行为修改须先过套件 | [lifecycle-20260814-02](../../areas-archive/lifecycle/records/2026-08-14-enable-manager-gtest.md)#F1 | 有效 |
| lifecycle/05#F1 | V3 enable_manager 保持 16 轴 CiA402 管理，14 CSP motion registry 在 enable 后默认进入 FJT_READY，并与 rolling 严格互斥 | [V3 运动控制器切换](../../areas-archive/lifecycle/records/2026-09-19-v3-motion-controller-switching.md)#F1-F3 | PARTIAL（T1 Mock；真实故障/停机待验） |
| chassis-control-20261022-01 | 整车速度在逆运动学前可选 S 曲线；保护停车保留原策略；研华已按1.0/4.0启动至IDLE | [整车速度 S 曲线](records/2026-10-22-chassis-twist-scurve.md) | PARTIAL（T3；未使能启动通过，实车响应/导航联调待验） |
| integration-20261014-02#F1–F4 | 底盘CSP/CSV与编码器只读边界保持；cyclic跨包测试移至integration | [第三步底盘组合](../integration/records/2026-10-14-chassis-offline-composition.md) | UNVERIFIED（T0；构建/测试通过，真实参数待补） |
| contract/14#F3 | `/cmd_vel` 是当前唯一底盘入口，取消底盘与双臂/升降/PP跨组互斥 | [V3 公共接口实现](../../areas-archive/contract/records/2026-10-02-v3-interface-contract-implementation.md)#F3 | PASS（契约/源码检查；联合运行待验） |
| motion/F3 | 三代机转向 CSP、驱动 CSV；外置编码器用于实际舵角观测与校验，不增加转向位置外环或运行中慢速修正 | [CSP 舵角决策](../../areas-archive/motion/records/2026-09-08-swerve-csp-feedback-decision.md)；BQ-144 | 已裁决，待实现/实机验证 |
| motion/F4 | ctrl_swerve（原 swerve_driver）算法与 ros2_control 插件已迁入，只申请 4 路位置与 4 路速度命令，不含 rezero/MIT/力矩前馈 | [源码迁移记录](../../areas-archive/motion/records/2026-09-09-swerve-controller-migration.md) | PARTIAL（T1 Native/容器；真实后端待接） |
| motion/F6 | 四轮速度使用固定规模最小二乘残差剔除；同一异常轮从 twist 与位置里程计排除且恢复无补跳，阈值保持待标定；共模一致误差不可辨识 | [残差剔除记录](../../areas-archive/motion/records/2026-09-16-swerve-slip-residual-rejection.md)#F1-F4 | PARTIAL（T1 Native/Mock；阈值和实机打滑待验） |
| motion/F7 | 舵轮只做逐轮机械区间内的正/反轮速分支选择和线性 slew；不存在 continuous-joint 模式，输出层不二次选支或静默 clamp | [有限转向记录](../../areas-archive/motion/records/2026-09-16-swerve-bounded-steering-planner.md)#F1-F4 | PARTIAL（T1 Native/Mock；逐轮限位与实机跟踪待验） |
| motion/F10 | ELECTRI-133 Tier A 固定规模 SE(2) 相对运动、独立无设备 Mock 与 opt-in loaned-interface/cyclic adapter 已实现；connected synthetic PDO 已验证 sent-cycle 与全四轮切换门禁，production profile/gate 未放开 | [Tier A 记录](../../areas-archive/motion/records/2026-09-25-tier-a-relative-move-mock.md)#F1-F4 | PARTIAL（T1；真实切换/标定/IMU 待验） |

## 跨区事实关联

底层PREOP/PDO/semantic约束见[protocol](../protocol/README.md)；FJT/Rolling启动拼装见[integration](../integration/README.md)。本区负责controller实现，不扩大为硬件使能授权。

[arm-control-20261023-05](records/2026-10-23-dpi-tool-preset-groups.md)#F1-F3：新增吸盘装配T0/T2/T4六个大组，保留旧预设；静态/API验收，不授权运动，实际新采集待用户执行（PARTIAL，T2）。

[arm-control-20261023-06](records/2026-10-23-dpi-j6-measurement-sequences.md)#F1-F3：新增⑦J6已建模吸盘自重换向诊断及⑧固定姿态准备；不自动加载外力矩，0x2241待准备（PARTIAL，T2）。

[arm-control-20261024-01](records/2026-10-24-vacuum-web-controls.md)#F1-F3：临时真空三按钮复用现场脚本，右阀00、左阀01、泵02；网页轮询只读，开关不自动联动。105项测试与浏览器模拟通过，实际读回三路OFF，实机开关待验（PARTIAL，T2）。

[arm-control-20261027-01](records/2026-10-27-vacuum-pressure-display.md)#F1-F3：操作台右上角消费左右压力 ROS 话题，AI0=左、AI1=右，显示换算后的 kPa 表压；单路无效或超过 1 秒未更新清除数字，与泵阀和运动准入独立。107 项包测试、构建、质量门禁及真实页面只读验收通过（PASS，T2）。
