# contract — 公共契约与适配层

**范围**：公共契约镜像（`robot_rt_control_interfaces`、`robot_system_interfaces`）
与 `source-lock.yaml`、域私有接口（`rt_control_interfaces`）、公共适配器
（enable/vacuum/status）、诊断归一化、`docs/cross-domain-interfaces.md` 实现视图。
**Owner 包/资产**：`src/rt_control/control_api_adapter`、`src/interfaces/*`、
`src/rt_control/rt_diagnostics`。

不属于本区：契约 schema 本身的修改（权威源在 `robot_interfaces` 仓库，本区只做
锁定升级）、PLC/BMS 节点实现（→ io-power）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | /joint_states 契约视图频率=125 Hz（实测口径，BQ-135） | [contract-20260813-01](records/2026-08-13-joint-states-125hz.md)#F1 | 有效 |

## 记录索引（倒序）

- 2026-08-13 [/joint_states 频率裁决为实测 125 Hz（BQ-135）](records/2026-08-13-joint-states-125hz.md) — decision，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-IF-RT-001..005（契约 0.5.0/0.6.0、公共适配器、
  ErrorInfo 采用）
- `docs/cross-domain-interfaces.md`（当前 main 实现视图，契约 0.6.0 @ `e19d1450`）
- 相关 BQ（不完全）：见 BLOCKED-questions.md 中 T-IF-RT 关联裁决
