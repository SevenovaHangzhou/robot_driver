# motion — 轨迹与运动执行

**范围**：whole_body_jtc 与 FJT 执行、diff_drive 底盘速度、控制器参数
（`controllers.yaml`）、关节限位（`joint_limits.yaml`）、控制环频率配置。
**Owner 包/资产**：`src/rt_control/rt_control_bringup` 的控制器/限位配置、
`patches/ros2_controllers`。

不属于本区：轨迹规划与任务编排（运控域，不在本仓库）、使能状态机（→ lifecycle）、
launch 结构与容器（→ release-deploy）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| F1 | `diff_drive_controller.wheel_separation=0.95089496 m`，为用户明确要求的有效轮距参数 `1.9598 * 0.4852`；该值不等同于已实测物理轮距 | BQ-140；[2026-08-24 记录](records/2026-08-24-track-effective-separation-95089496mm.md) | 已裁决，待 T4 实车转向/里程计复验 |
| F2 | 官方夹爪 controller 的 PP 扩展默认关闭；启用后按同序号反馈完成到位/取消，max_effort 使用 N，不接受无限力请求 | [PP Action 记录](records/2026-09-08-pp-gripper-action.md) | PARTIAL，实机待验 |
| F3 | 三代机转向 CSP、驱动 CSV；外置编码器用于实际舵角观测与校验，不增加转向位置外环或运行中慢速修正 | [CSP 舵角决策](records/2026-09-08-swerve-csp-feedback-decision.md)；BQ-144 | 已裁决，待实现/实机验证 |
| F4 | swerve_driver 算法与 ros2_control 插件已迁入，只申请 4 路位置与 4 路速度命令，不含 rezero/MIT/力矩前馈 | [源码迁移记录](records/2026-09-09-swerve-controller-migration.md) | PARTIAL（T1 Native/容器；真实后端待接） |
| F5 | HT-WS-HH270 厂家图纸已登记名义轮径、传动、安装包络和悬挂参数；数据保持 draft，不能替代逐轮有效半径、零位、硬限位和背隙标定 | [厂家机械图纸提取](records/2026-09-15-swerve-module-mechanical-drawing.md)#F1-F4 | UNVERIFIED（T0 图纸提取；厂家澄清/实测待完成） |
| F6 | 四轮速度使用固定规模最小二乘残差剔除；同一异常轮从 twist 与位置里程计排除且恢复无补跳，阈值保持待标定；共模一致误差不可辨识 | [残差剔除记录](records/2026-09-16-swerve-slip-residual-rejection.md)#F1-F4 | PARTIAL（T1 Native/Mock；阈值和实机打滑待验） |
| F7 | 舵轮只做逐轮机械区间内的正/反轮速分支选择和线性 slew；不存在 continuous-joint 模式，输出层不二次选支或静默 clamp | [有限转向记录](records/2026-09-16-swerve-bounded-steering-planner.md)#F1-F4 | PARTIAL（T1 Native/Mock；逐轮限位与实机跟踪待验） |
| F8 | V3 FJT/rolling 固定 7+7 CSP 轴序，PP 夹爪不进入运动 writer；初始周期为 1 ms、maximum 2 ms | [V3 双七轴运动 runtime](records/2026-09-19-v3-arm-fjt-rolling-runtime.md)#F1-F3 | PARTIAL（T1 Mock；标定/目标机/实机待验） |
| F9 | 用户与厂商确认无电池多圈编码器按单圈使用会报 `0x730F`，J3/J4 每次使能前需确认受控复位；十四轴 `+1` 是未验证的方向试验值 | [旧工控机 JTC 原位验证](records/2026-09-19-v3-jtc-old-ipc-readonly.md)#F4-F5 | PARTIAL（T3 使能已验；方向待逐轴验证） |
| F10 | ELECTRI-133 Tier A 固定规模 SE(2) 相对运动、独立无设备 Mock 与 opt-in loaned-interface/cyclic adapter 已实现；connected synthetic PDO 已验证 sent-cycle 与全四轮切换门禁，production profile/gate 未放开 | [Tier A 记录](records/2026-09-25-tier-a-relative-move-mock.md)#F1-F4 | PARTIAL（T1；真实切换/标定/IMU 待验） |

## 记录索引（倒序）

- [2026-09-25 ELECTRI-133 Tier A 相对运动核心、无设备 Mock 与 cyclic adapter 集成](records/2026-09-25-tier-a-relative-move-mock.md) — PARTIAL（T1；connected synthetic PDO 通过，真实 Kinco 切换仍阻塞）

- 2026-09-19 [V3 JTC-only 旧工控机旁路与原位使能验证](records/2026-09-19-v3-jtc-old-ipc-readonly.md) — commissioning，PARTIAL（T3；分支已恢复，16 轴使能/保持/失能通过，JTC 轨迹未开始）
- [2026-09-19 V3 双七轴 FJT 与 rolling 实时控制](records/2026-09-19-v3-arm-fjt-rolling-runtime.md)
- [2026-09-16 四舵轮机械限位内转向规划与输出门禁](records/2026-09-16-swerve-bounded-steering-planner.md)
- [2026-09-16 四舵轮最小二乘残差剔除与里程计隔离](records/2026-09-16-swerve-slip-residual-rejection.md)
- [2026-09-15 HT-WS-HH270 舵轮厂家机械图纸提取](records/2026-09-15-swerve-module-mechanical-drawing.md)
- [2026-09-09 swerve_driver 实际源码迁移](records/2026-09-09-swerve-controller-migration.md)
- [2026-09-08 舵轮保留转向 CSP 与外置舵角观测](records/2026-09-08-swerve-csp-feedback-decision.md)
- [2026-09-08 官方夹爪 PP Action 扩展](records/2026-09-08-pp-gripper-action.md)

- [2026-08-24 履带有效轮距调整为 0.95089496 m](records/2026-08-24-track-effective-separation-95089496mm.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-019 三条（partial / corrective / 14-axis minimal FJT）
- `docs/fjt-14axis-low-speed-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-119、BQ-120、BQ-122
