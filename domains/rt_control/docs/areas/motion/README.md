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

## 记录索引（倒序）

- [2026-09-09 swerve_driver 实际源码迁移](records/2026-09-09-swerve-controller-migration.md)
- [2026-09-08 舵轮保留转向 CSP 与外置舵角观测](records/2026-09-08-swerve-csp-feedback-decision.md)
- [2026-09-08 官方夹爪 PP Action 扩展](records/2026-09-08-pp-gripper-action.md)

- [2026-08-24 履带有效轮距调整为 0.95089496 m](records/2026-08-24-track-effective-separation-95089496mm.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-019 三条（partial / corrective / 14-axis minimal FJT）
- `docs/fjt-14axis-low-speed-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-119、BQ-120、BQ-122
