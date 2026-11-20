---
id: control-20261120-01
area: control
title: 双臂速度跟随 URDF，加速度采用分关节规划上限
date: 2026-11-20
type: feature
trigger: 用户要求速度限幅跟随 URDF，加速度采用 J1–J3 373.06、J4 498.41、J5–J7 895.35 °/s²
commits: []
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PASS
evidence: []
supersedes: []
related: []
---

## 背景

双臂控制器此前从机器人模型的限位 YAML 取得统一的 60°/s² 加速度。
本次按用户明确给出的规划峰值，采用 RT 共享策略；速度仍从所选 URDF 的
`<limit velocity>` 读取，不在 RT 另写固定速度值。

## 改动

`hwcfg_machines/machines/Alfa_V3/policies/arm_acceleration_limits.yaml` 按关节名记录双臂上限：

| 关节（左右臂相同） | 原加速度 °/s² | 新加速度 °/s² | 新加速度 rad/s² |
|---|---:|---:|---:|
| J1–J3 | 60 | 373.06 | 6.511125307490046 |
| J4 | 60 | 498.41 | 8.698895524864938 |
| J5–J7 | 60 | 895.35 | 15.62680545773123 |

`arm_trajectory.yaml` 中四个 FJT 配置及两个 Rolling 配置共用此文件。
组合器的 YAML 参数源新增 `joint_order: {policy, path}`，按当前策略关节顺序生成数组；
缺少选中关节的数值时拒绝组合。机型 manifest 登记策略与正数上限 schema。
不修改 `robot_description` 构建副本或其上游限位文件。

控制器超限拒绝逻辑保持原有行为。Rolling 的既有制动实现也复用
`joint_acceleration_limits`，因此其制动轨迹计算会采用新值；这不构成实机制动能力证明。

## 验证

- T0：`integ_composer/test` 全套 pytest，258 项通过。
- 新增回归覆盖左臂、右臂、双臂：改变测试 URDF 速度、反转策略关节顺序，
  验证最终 FJT/Rolling 的速度来自 URDF、加速度按关节名匹配；缺项时拒绝组合。
- T0：`colcon build --packages-select hwcfg_machines integ_composer` 通过，
  构建及安装目录均在 `/tmp/arm-acceleration-limits-20261120/`。
- `python3 tools/arch_check.py` 与 `git diff --check` 通过。
- 未进行 T1 控制器运行或 T2–T4 实机验证；未部署、未修改现场参数。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 双臂速度上限继续由当前选用的 URDF 决定。
- F2: 双臂加速度上限采用上表，FJT 与 Rolling 共用同一份 RT 配置。
- F3: 本地源码和组合输出已验证；现场旧 artifact 不会自动更新，生效需要重新生成并部署配置。

## 遗留

新加速度限制及其对 Rolling 制动的影响尚未经过实机验证。
