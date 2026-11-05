---
id: integration-20261104-02
area: integration
title: R-IN-16 /rt/head/move 由 integ_head_move 提供，网页改用 Action
date: 2026-11-04
type: behavior
trigger: 契约 cb379db 登记 /rt/head/move 并删除 /head_position_controller/commands 外部写入；用户 2026-11-04 选集成层小节点
commits: [feature/arch-head-move-action]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [integration-20261104-01]
---

## 改动

- 新包 `integ_head_move`：节点 `rt_head_move` 提供 `/rt/head/move`（`MoveHead`）。从制品读取头部控制器关节顺序、命令范围（机器人模型 `command_interface position min/max`）和限速（`velocity_limit`），向内部 `/head_position_controller/commands` 下发一次绝对目标。
- 判定（用户 2026-11-04）：两轴都进入 ±0.01 rad 为到位；超时 = 最慢关节距离 / 限速 + 2 s。执行中再来目标拒绝；取消、超时、`/rt/readiness` 变为不就绪时下发实测位置原地保持；关节反馈超过 500 ms 未更新则中止。错误码：成功 0、取消 80、超时 2、超出命令范围 4、不就绪 130、反馈过期 1141。
- 能力 `head_position` 改为 `head_move`（`/rt/head/move`）；作用域声明该能力时 bringup 启动节点。
- 网页头部区改为发送 Action 目标；新目标前先取消正在执行的目标。

## 验证

本机：`integ_head_move` 用真实节点对模拟头部与 readiness 测试到位、忙拒绝、取消保持、超范围、不就绪；相关 5 个包 537 项测试通过。未上台架。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 头部外部运动入口只有 `/rt/head/move`；目标字段顺序 head_joint=H1（偏航）、head_pitch_joint=H2（俯仰）。

## 遗留

- 台架：使能后经网页与 `ros2 action send_goal` 各走一次小角度目标、取消一次。
