# chassis-control — 四舵轮控制算法与执行

**范围**：四舵轮控制算法与执行。本区对应一个同名职责agent。

**Owner路径**：src/rt_control/swerve_driver/**。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：硬件协议与编码器提供器归protocol；机型事实/标定归hw-config；启动组合归integration。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| chassis-control-20261024-01 | 保持使能时通过公共候选服务停稳切换模式；运动中拒绝、保留执行历史 | [运行时模式切换](records/2026-10-24-chassis-mode-switch.md) | PARTIAL（T3；研华未使能启动/接口通过，真实切换与契约发布待验） |
| chassis-control-20261022-01 | 整车速度在逆运动学前可选 S 曲线；保护停车保留原策略；研华已按1.0/4.0启动至IDLE | [整车速度 S 曲线](records/2026-10-22-chassis-twist-scurve.md) | PARTIAL（T3；未使能启动通过，实车响应/导航联调待验） |
| integration-20261014-02#F1–F4 | 底盘CSP/CSV与编码器只读边界保持；cyclic跨包测试移至integration | [第三步底盘组合](../integration/records/2026-10-14-chassis-offline-composition.md) | UNVERIFIED（T0；构建/测试通过，真实参数待补） |
| contract/14#F3 | `/cmd_vel` 是当前唯一底盘入口，取消底盘与双臂/升降/PP跨组互斥 | [V3 公共接口实现](../contract/records/2026-10-02-v3-interface-contract-implementation.md)#F3 | PASS（契约/源码检查；联合运行待验） |
| motion/F3 | 三代机转向 CSP、驱动 CSV；外置编码器用于实际舵角观测与校验，不增加转向位置外环或运行中慢速修正 | [CSP 舵角决策](../motion/records/2026-09-08-swerve-csp-feedback-decision.md)；BQ-144 | 已裁决，待实现/实机验证 |
| motion/F4 | swerve_driver 算法与 ros2_control 插件已迁入，只申请 4 路位置与 4 路速度命令，不含 rezero/MIT/力矩前馈 | [源码迁移记录](../motion/records/2026-09-09-swerve-controller-migration.md) | PARTIAL（T1 Native/容器；真实后端待接） |
| motion/F6 | 四轮速度使用固定规模最小二乘残差剔除；同一异常轮从 twist 与位置里程计排除且恢复无补跳，阈值保持待标定；共模一致误差不可辨识 | [残差剔除记录](../motion/records/2026-09-16-swerve-slip-residual-rejection.md)#F1-F4 | PARTIAL（T1 Native/Mock；阈值和实机打滑待验） |
| motion/F7 | 舵轮只做逐轮机械区间内的正/反轮速分支选择和线性 slew；不存在 continuous-joint 模式，输出层不二次选支或静默 clamp | [有限转向记录](../motion/records/2026-09-16-swerve-bounded-steering-planner.md)#F1-F4 | PARTIAL（T1 Native/Mock；逐轮限位与实机跟踪待验） |
| motion/F10 | ELECTRI-133 Tier A 固定规模 SE(2) 相对运动、独立无设备 Mock 与 opt-in loaned-interface/cyclic adapter 已实现；connected synthetic PDO 已验证 sent-cycle 与全四轮切换门禁，production profile/gate 未放开 | [Tier A 记录](../motion/records/2026-09-25-tier-a-relative-move-mock.md)#F1-F4 | PARTIAL（T1；真实切换/标定/IMU 待验） |
