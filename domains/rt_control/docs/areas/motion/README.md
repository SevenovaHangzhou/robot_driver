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
| 01#F1 | 首点一致性准入启用时，whole_body_jtc 只接受 FJT action | [motion-20260819-01](records/2026-08-19-jtc-action-only-admission.md)#F1 | 有效（T1） |
| 01#F2 | 一致性准入关闭时保留上游 JTC topic 行为 | [motion-20260819-01](records/2026-08-19-jtc-action-only-admission.md)#F2 | 有效（T1） |
| 01#F3 | topic 存在性与准入开关绑定，不增设独立配置位 | [motion-20260819-01](records/2026-08-19-jtc-action-only-admission.md)#F3 | 有效（T1） |

## 记录索引（倒序）

- 2026-08-19 [JTC 首点一致性启用时关闭 topic 命令旁路](records/2026-08-19-jtc-action-only-admission.md) — fix，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-019 三条（partial / corrective / 14-axis minimal FJT）
- `docs/fjt-14axis-low-speed-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-119、BQ-120、BQ-122
