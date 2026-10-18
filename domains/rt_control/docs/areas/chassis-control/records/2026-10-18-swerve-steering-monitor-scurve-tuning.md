---
id: chassis-control-20261018-02
area: chassis-control
title: 转向控制：外置编码器改为监视、S 型转向曲线、现场调参与取消对齐等待
date: 2026-10-18
type: feature
trigger: ELECTRI-127；2026-10-18 研华台架压测与用户决定
commits: [feature/electri-127-swerve-steering]
env: native
risk: T3
writes: {reset: no, enable: yes, motion: yes, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [chassis-control-20261018-01, chassis-control-20261017-03]
---

## 用户决定（2026-10-18）

| 事项 | 决定 |
| --- | --- |
| 外置编码器角色 | 监视为主：250 Hz CANopen、与 1 kHz 控制不同步、经齿轮有间隙，不用于控制与里程计方向 |
| 编码器故障判定 | 持续超过时间窗口才报（默认 50 ms），单周期异常不报 |
| 转向曲线 | S 型加减速（加加速度限制） |
| 转向速度 | 网页可调，令牌确认，上限 45°/s |
| 参数持久化 | 网页修改写入工控机现场调参文件，重启保留；仓库默认值不变 |
| 舵角对齐 | 不等待对齐：按 cos(舵角误差) 缩放各轮速度 |

## 实现

**外置编码器（`control_core`、`swerve_controller`）**
- 当前转角、对齐判断、保持检查、里程计方向使用电机推算角（`steering_position`）。
- 外置编码器：激活时核对一次（与电机角相差 ≤ `max_encoder_difference`，且在测量容差内）；之后每周期校验有效、范围与一致性，连续不一致超过 `encoder_fault_window`（默认 0.05 s）报 `feedback_fault`。
- 保持中的模块若电机角偏离锁存目标超过 `max_encoder_difference`，该周期不计入里程计，避免触发保持故障的跳变污染位姿。
- 诊断增加锁存故障类型、各模块原因（`stale`、`not_enabled_or_mode`、`encoder_difference`、`non_finite`）、最大编码器差与最大目标偏差。

**S 型转向（`swerve_setpoint`）**
- 新参数 `maximum_steering_jerk_radps3`（控制器参数 `max_steering_jerk`，默认 100°/s³）；无限大保留原梯形曲线。
- 每周期按当前转速与加速度做制动检查：从下一状态按加速度与加加速度限制停车仍能在剩余角度内完成，则继续加速或巡航，否则制动。到达时转速与加速度连续归零，不越过目标；目标中途改变时同样适用。保持分支同样按加加速度把在途转速降到零。

**在线调参（`swerve_controller`）**
- `max_steering_slew`、`max_steering_acceleration`、`max_steering_jerk` 设置时按上限 `max_steering_slew_limit`（45°/s）、`max_steering_acceleration_limit`（90°/s²）、`max_steering_jerk_limit`（1000°/s³）校验；`alignment_threshold` 限定 (0, 90°]。
- 新值在下一次激活时生效（重建控制核心），操作台只在未使能时修改。

**对齐门限**
- `controller.draft.yaml` 默认 `alignment_threshold` 改为 90°（原 1°）；门限触发时各轮按轮加速度限值斜坡降速，不再一周期置零。

**现场调参（`rt_control_bringup`）**
- 新步骤 `scope_operator_tuning`：运动控制器加载后、使能管理器之前，读取 `/var/lib/rt-control/tuning/<machine_unit>/<controller>.yaml`，按白名单（四个转向参数）经控制器参数服务写入，限值由控制器校验。文件无效或被拒时保留默认并输出 `RT_SCOPE_TUNING`，不阻止启动。

## 验证

- 单元测试（本机）：`swerve_driver` 全部通过（含新增：编码器监视窗口边界、孤立无效样本不累计、门限触发时轮速斜坡、S 曲线限值/平滑到达/目标变向、加加速度参数校验、参数上限与激活时生效）；`rt_control_bringup` 26 个 Python 测试通过（含现场调参文件校验与启动步骤顺序）；组合器 217 通过（1 项安装布局环境失败与本变更无关）。
- 研华台架 2026-10-18：编码器监视、S 曲线、在线调参、现场调参持久化（保存→重启→启动日志 `applied`→恢复默认）、取消对齐等待均已部署，用户压测使能、旋转、平移、换向通过。

## 未验证与剩余风险

- 以下两点在部署后补充，仅有单元测试：保持中偏离目标的模块不计入里程计；参数校验回调改为在全部声明之后注册（部署版本靠配置覆盖值通过声明校验，行为一致）。
- 取消对齐等待后，转向未到位时驱动轮已出力，变向与原地旋转起步时实际轨迹偏离指令、轮胎有横向擦地；可用网页把门限调到 30–45°。
- 现场调参文件位于工控机，不在仓库；确定数值后应回写仓库默认值。
