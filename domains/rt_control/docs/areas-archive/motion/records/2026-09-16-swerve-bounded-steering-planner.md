---
id: motion-20260916-02
area: motion
title: 四舵轮机械限位内转向规划与输出门禁
date: 2026-09-16
type: corrective
trigger: ELECTRI-117，补齐 main 模块化接入遗漏的有限转向 Phase 8 语义
commits: [work/electri-117-swerve-module-binding]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [ELECTRI-117, BQ-145, motion-20260909-01, motion-20260915-01]
---

## 背景

main 的模块化 `swerve_driver` 仍把舵角当连续坐标：`+179° -> -179°` 会生成 `+181°`
目标，再由末端范围检查停车。这不会直接越界下发，但没有完成用户已裁决的有限机械行程内
分支规划，也会在 pi 附近造成不必要停车。本次把最新的有限位语义迁入协议无关控制器。

## 改动

- 删除 `continuous_angle_rad` 目标语义。每轮分别计算“目标角+正轮速”和“等效角+反轮速”
  在安全机械区间内的最近候选，按实际线性转向距离和滞回选择。
- slew 不再对角差执行周期 wrap；起点和终点都在同一安全区间内，整条线性轨迹不会跨越
  `steering_min/max` 硬端点。
- 逐轮增加 `steering_limit_margin` 与 `steering_limit_tolerance`。margin 只向内缩目标区间；
  tolerance 只接受端点附近测量误差并把规划观测投影回安全区间，不扩大可下发目标。
- 纯平移在四轮共同可行时用总转向代价选择统一 0°/180° 分支并保持滞回；若逐轮非对称
  限位使两个统一分支都不可行，才退回每轮独立的安全候选。
- 新增 `steering_angle_deadband` 和 `translation_heading_epsilon`；所有值保持 draft/TBD，
  不从厂家图纸猜测。
- `ControlCore` 在生成后再次校验每轮目标；`SwerveController::write_output()` 在写任何接口前
  整体检查四个 CSP 目标和四个 CSV 数值，失败时全轮速度置零并拒绝周期。输出层不重选分支、
  不做 PMAX 重定位，也不静默 clamp 命令。

## 验证

- 小型 TDD 覆盖：`+179° -> -179°` 选择 `+1°` 反轮速表示、pi 边界 slew 不越界、
  逐轮范围不足拒绝、平移统一分支、角度死区、测量容差不扩大命令区间。
- 控制核心级回归确认同一 pi 边界场景最终 CSP 输出为 `+1°`，对齐前四轮速度保持零。
- Native clean build 通过；`swerve_driver` 6 个测试目标当前 77 tests，0 errors、
  0 failures、0 skipped。
- ASan+UBSan 算法/核心回归 42 tests 通过；受影响的 CANopen/EtherCAT/machine/governance
  配置回归 149 passed；仓库 quality gate 282 passed、13 skipped、策略覆盖率 83%。
  本机没有 ShellCheck，留待 CI 执行。
- 未执行真实 EtherCAT/CANopen、使能或运动；四轮数值硬限位、裕量、测量容差、死区、
  转向 slew 和跟踪过冲均待现场标定/验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 三代机舵轮是有限行程关节，不提供 continuous-joint 运行模式。
- F2: `+179° -> -179°` 不能作为跨硬限位的 2° 路径；目标和整段 slew 都必须位于同一
  已标定安全区间。
- F3: 分支选择只在 `swerve_driver` 发生一次；EtherCAT 执行层和命令写出层不得重新做
  ±pi 换向、轮速反转或 PMAX 重定位。
- F4: 测量容差与目标裕量职责相反：前者只判断反馈可接受，后者缩小命令范围，二者都不能
  替代真实机械硬限位标定。

## 遗留

厂家/现场仍需为 FL/FR/RL/RR 分别提供或标定物理最小/最大角、目标裕量、允许的测量端点误差、
最大可靠 slew 和角度死区。台架需在断使能手推、低速 CSP 和带载跟踪三个层次验证端点方向、
跟踪过冲及 `+pi/-pi` 附近往返，不得用软件参数扩大机械行程。
