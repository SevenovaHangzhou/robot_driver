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

## 记录索引（倒序）

- [2026-08-24 履带有效轮距调整为 0.95089496 m](records/2026-08-24-track-effective-separation-95089496mm.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-019 三条（partial / corrective / 14-axis minimal FJT）
- `docs/fjt-14axis-low-speed-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-119、BQ-120、BQ-122
