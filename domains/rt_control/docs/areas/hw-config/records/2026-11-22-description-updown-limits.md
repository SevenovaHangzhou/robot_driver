---
id: hw-config-20261122-03
area: hw-config
title: 同步 Robot Model 到 robot_description 09a7ca4：升降实测限位，并带入主动悬挂、后舵轮反装与双臂手册速度
date: 2026-11-22
type: sync
trigger: 用户 2026-11-22 给出升降行程与速度（ELECTRI-241），并决定双臂速度直接采用手册峰值
commits: [feature/sync-description]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [protocol-20261122-01]
---

## 背景

升降将以 CSP 进入机械臂 15 轴轨迹，轨迹位置/速度限位取自 Robot Model；快照里的 `updown` 仍是占位 [-1, 0] m、5 m/s。robot_description#21 已把它改为实测值。快照必须与上游 tree 一致，所以同时带入 98d498e 之后的上游改动。

## 改动

`src/description/robot_description` 按上游 09a7ca4（tree b13cd94）导入，`source-lock.yaml` 同步。相对 98d498e：

- `updown`：[-1.05, 0] m、0.1666 m/s，加速度上限 0.4 m/s²（限位 YAML 与 cuRobo 配置）。
- 双臂 J1–J7 速度上限由 30°/s 改为 eRob V3.44 输出端峰值 [75, 75, 75, 100.2, 180, 180, 180]°/s；用户 2026-11-22 决定 robot_driver 直接采用，JTC 与 Rolling 准入速度随之变化。
- 新增主动悬挂移动关节 `suspension`（base_link 下，后两组舵轮挂在其下，0 时位姿不变）。
- 后两组舵轮 steer3/steer4 绕转向轴反装 180°，wheel3/wheel4 轴取反；底盘控制使用单机标定坐标，不读这些原点。
- 升降架与头部碰撞球按已批准拟合；Home 的升降为 0。

金样：arm/head 的 `robot_description.xml`、arm 的 JTC/Rolling `joint_velocity_limits`（manifest `description_updown_limits_regeneration`）。测试：双臂速度断言改为手册峰值；TF 合同加入 `suspension`。

## 验证

- 导入目录 tree 与上游 b13cd94 一致；repository gate、组合器与 bringup 测试（除 1 项环境相关）、tools 测试通过。
- 未验证：实机；cuRobo 规划回归。

## 结论与冻结事实

- F1: robot_driver 的 Robot Model 快照为 robot_description 09a7ca4（tree b13cd94）。
- F2: 双臂轨迹与 Rolling 的速度准入采用 eRob 手册峰值（用户 2026-11-22）。

## 遗留

- `suspension` 没有硬件状态源时，robot_state_publisher 发不出 `suspension_link` 及其下后舵轮的 TF；整机接入主动悬挂前需补状态源或固定发布。
- 后舵轮反装后的轮向显示需在实机 TF 中核对。
