# contract — 公共契约与适配层

**范围**：公共契约 vendor（`robot_rt_control_interfaces`、`robot_system_interfaces`、
`robot_interfaces_qos`）与 `source-lock.yaml`、域私有接口（`rt_control_interfaces`）、
公共适配器（enable/vacuum/status）、诊断归一化及 vendored RT-Control 契约视图。
**Owner 包/资产**：`src/rt_control/control_api_adapter`、`src/interfaces/*`、
`src/rt_control/rt_diagnostics`。

不属于本区：契约 schema 本身的修改（权威源在 `robot_interfaces` 仓库，本区只做
锁定升级）、PLC/BMS 节点实现（→ io-power）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | /joint_states 契约视图频率=125 Hz（旧本地视图来源） | [contract-20260813-01](records/2026-08-13-joint-states-125hz.md)#F1 | 已由 02#F1 取代来源指针 |
| 02#F1 | vendored 权威 RT-Control 视图固定 /joint_states=125 Hz | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F1 | 有效 |
| 02#F2 | src/interfaces 只保留域内私有接口，公共 schema 从 vendor 构建 | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F2 | 有效 |
| 02#F3 | 跨域 Topic 使用 robot_interfaces_qos 命名 profile | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F3 | 有效 |
| 02#F4 | /cmd_vel_safe QoS 由 Motion 与 RT-Control 原子升级 | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F4 | 有效 |
| 03#F1 | ErrorInfo 是跨域公共载荷，code 唯一采用 uint32 DREE | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F1 | 有效 |
| 03#F2 | 当前 DomainReadiness 为最终公共字段集并遵守一致性规则 | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F2 | 有效 |
| 03#F3 | 全部生产/消费域必须锁定同一接口 SHA 原子升级 | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F3 | 有效 |
| 03#F4 | d8236bd 是 PR #6 验证 SHA；发布仍须回填最终 main SHA | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F4 | 有效 |

## 记录索引（倒序）

- 2026-08-16 [BQ-137 跨域错误与 readiness 语义裁决落地](records/2026-08-16-bq137-shared-error-readiness.md) — decision，PASS（T1；发布待 PR #6 最终 SHA 与全域迁移）
- 2026-08-14 [公共接口改为固定 SHA vendoring 并采用命名 QoS](records/2026-08-14-robot-interfaces-vendoring-and-qos.md) — decision，PASS（T1；当时的 BQ-137 语义阻塞已由 03#F1～F4 闭环）
- 2026-08-13 [/joint_states 频率裁决为实测 125 Hz（BQ-135）](records/2026-08-13-joint-states-125hz.md) — decision，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-IF-RT-001..005（契约 0.5.0/0.6.0、公共适配器、
  ErrorInfo 采用）
- 已删除的本地跨域接口视图（历史 main：契约 0.6.0 @ `e19d1450`）
- 相关 BQ（不完全）：见 BLOCKED-questions.md 中 T-IF-RT 关联裁决
