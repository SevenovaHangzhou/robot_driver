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
| 01#F1 | `driver_variant.yaml` 是 actuator 到既有 logical joint 的驱动映射及控制/生命周期参数事实源；joint 名称与类型仍以 Robot Model 为权威 | [motion-20260818-01](records/2026-08-18-driver-variant-manifest.md)#F1 | 有效 |
| 01#F2 | 当前机型固定为 14 个 whole-body EtherCAT joint 与 2 个 CANopen track joint | [motion-20260818-01](records/2026-08-18-driver-variant-manifest.md)#F2 | 有效 |
| 01#F3 | ELECTRI-94 phase 1 只做离线一致性门禁，不提供运行时热切换 | [motion-20260818-01](records/2026-08-18-driver-variant-manifest.md)#F3 | 有效 |

## 记录索引（倒序）

- 2026-08-18 [驱动变体 manifest 与控制器投影（ELECTRI-94）](records/2026-08-18-driver-variant-manifest.md) — feature，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-019 三条（partial / corrective / 14-axis minimal FJT）
- `docs/fjt-14axis-low-speed-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-119、BQ-120、BQ-122
