---
id: control-20261107-01
area: control
title: 关节与末端页面共享双臂权限，按入口自动选模并适配 RollingSender
date: 2026-11-07
type: feature
trigger: 用户要求远端 18121 与 18131 互斥、页面跳转及按运动入口自动选择 JTC 或 Rolling
commits: [codex/jog-current-framework]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 改动

共享 ArmConsole.authority 统一关节动作、末端窗口和作用域使能/复位准入。
按连接与代次互斥，同一登录也不共享所有权。请求终态与其后的实测静止证据齐全才交接。
启动对账覆盖 JTC、当前 Rolling HOLDING 和两控制器明确 inactive 的状态；位置静止阈值取 EnableManager 的公开参数。
可选末端发布采用官方 RollingSender 薄绑定，默认关闭。顶栏增加当前主机 18131 的末端页入口。
有效关节动作或序列选择 JTC，末端 begin_motion 选择 Rolling；在同一运动预留中调用公开模式服务，并确认实际互斥状态与切换后的停稳反馈。等待期间保持停止、释放、断线可用，取消后不能迟到发送。超时或不明确结果继续占用，硬件使能仍显式发起。
位置反馈的 JTC 复用 EnableManager 的实测静止窗口；已观测的 inactive 状态建立首次无活动目标证据，不依赖不存在的初始空 Action 状态消息。

## 验证与边界

基于 cbf4116，公共接口 aa82a77。本地包构建、行为回归和独立 localhost Domain 的官方 sender 传输测试通过。
已在远端统一网页发行目录部署两个入口；同一登录双向互斥、释放交接、空闲断线归还、浏览器跳转和整机渲染通过。
权限验证结束全部归还，没有使能、模式切换或发送运动目标。
当前 18131 已运行 Mink 键盘入口，18121 已配置末端发布；跟随实际运行的 bench_dual_arm_head_wrist_force artifact、Domain 77，仅重启网页，保留底层实时进程。重启后未使能、未接管，权限验证结束空闲。独立求解环境、当前模型与反馈 SHA-256 匹配，左右臂六方向的 0.5 mm/s 离线窗口复核通过，首窗口约 20–23 ms。Mink 键盘链的实机运动、TCP/方向和碰撞候选余量尚待核验。
自动选模与取消竞态、服务消息及拒绝/超时语义通过本地行为测试；尚未发送真机模式切换请求。
