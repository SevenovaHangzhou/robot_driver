---
id: control-20261109-01
area: control
title: 冻结末端键盘控制组件 v0.2.0 与配套 Rolling 接入
date: 2026-11-09
type: feature
trigger: 用户完成现场试用后要求打版本并推送至 robot_driver
commits: [codex/jog-current-framework]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261107-01]
---

## 行为

18121 的关节动作与 18131 的末端窗口经同一双臂权限入口；按请求入口选择互斥的 JTC 或 Rolling，停稳反馈确认后才发送。重新登录不重放运动。Rolling 的静止命令查询用于下一次会话锚点，避免反馈位置与已保持命令不同而拒绝再次起步。

Rolling 非实时状态快照在配置的控制周期预算内有界重读；实时 update 不新增等待。拒绝诊断在初始交接门外发布，包含 epoch、sequence、具体失败分支，以及动态/制动空间超限的关节、段号、数值和限制。标准诊断主题为控制器私有 `~/rejection`，公共 Rolling schema 不变。

外部 Mink 客户端采用原硬件包络内 98% 规划预算、段内速度约束及连续制动尾段，提供时延分位统计。初窗 120 ms、替换窗 100 ms，现场 artifact、跟随误差阈值与碰撞余量不变。

## 源码版本

驱动接入分支 `codex/jog-current-framework`，配套组件标签 `jog-rolling-integration-v0.2.0`。外部客户端在同仓库的独立源码分支 `release/gripper-jog-v0.2.0`，标签 `gripper-jog-v0.2.0`；其 `integration/release.json` 绑定本驱动提交及接口 pin。客户端不参与 RT-Control 包构建。

## 证据与边界

基线 cbf4116、公共接口 aa82a77。63 项控制器 C++、372 项操作台 Python、406 项客户端 Python 通过；客户端另有 180 项条件跳过，旧公共 ROS 会话 schema 的测试未纳入。受影响 CMake 包构建、集成补丁重放、17 个 Node 输入/登录用例和桌面/窄屏渲染通过。提交前运行仓库质量门禁。

2026-11-09 已部署到现场 Domain 77 的 bench_dual_arm_head_wrist_force；源码与实际加载插件哈希匹配，主页面、末端页、反馈和拒绝诊断订阅只读检查通过。更新流程没有主动使能或发送运动。随后用户反馈“效果不错”，本记录不据此补写未采集的运动指标。完整生产验收及整栈镜像发布未执行。

插件 SHA-256：`27a721dc0452d384fb8c8e9e1acd66d2f4722ca10ff325da4c14337c94022658`。

现场部署回执：`/home/ar/gripper-jog/releases/shared-authority-20261107/rolling-reliability-receipt-20261109.json`。
