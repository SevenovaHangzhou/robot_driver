---
id: control-20261121-01
area: control
title: 规划加速度回归模型唯一来源
date: 2026-11-21
type: fix
trigger: SEV-134 / robot_driver#298；用户确认加速度应由 robot_description 提供
commits: [codex/control-backlog-fixes]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [control-20261120-01]
related: [SEV-134, robot_driver#298, robot_driver#297]
---

## 背景

模型历史上保留统一 60°/s²，而 RT 直接读取另一份逐轴上限。只把 RT 文件改走 policy 加载，
仍会保留两个数值来源。#297 的统一 10 rad/s² 提案已关闭为不计划实施；沿用用户已确认的逐轴规划值。

## 改动

权威模型提交 `3147e94` 在 `config/arm_motion_limits.yaml` 定义规划加速度：
J1–J3 373.06、J4 498.41、J5–J7 895.35°/s²（用户 2026-11-20）。生成器统一转换为 SI，
写入 joint_limits 和 cuRobo。模型源、生成器、两份输出和哈希一起更新；其余模型几何和速度不变。
驱动以 source-lock 的完整 SHA/tree 导入同一模型，六处控制器绑定改用 model_limits.acceleration，
入口 schema 拒绝非数、非有限、非正数，选中轴缺项则拒绝组合。取消重复 RT 策略、schema 和无调用方的 YAML 排序分支。

控制器加速度数值不因本次来源切换而变化。Rolling 现有制动计算仍复用同一数组；该规划数值不是实机制动能力认证。
模型外置是另一项结构变更，本次仍保留构建副本。

## 验证

- T0：模型 pytest 20 通过、2 项可选几何依赖跳过；已安装 check_urdf 环境下通过。
- T0：组合/启动 pytest 317 通过；覆盖左、右、双臂限值变化传播、轴顺序变更、缺项及非法值拒绝。
- T0：相关 8 包本地构建通过；colcon 汇总 561 项、0 error、0 failure、2 skip；quality_gate 185 通过、11 skip。
- 未进行 T1 全图或 T2–T4 实机验收；未更新现场 artifact。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 双臂速度来自模型 URDF，规划加速度来自同一模型的 joint_limits；RT 不再维护另一份加速度数值。
- F2: 模型与 cuRobo 使用同一 authored planning source；统一 60°/s² 是已被本记录替换的历史配置。
- F3: 保留 Rolling 现有制动语义，实机停止时间/距离与负载能力仍须单独验证。

## 遗留

由现场人员完成分级速度/加速度、负载与制动验收；不因离线数值一致就关闭现场验收项。
