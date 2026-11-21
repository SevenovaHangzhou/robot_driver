---
id: control-20261121-03
area: control
title: 软急停范围收敛到头部与双臂
date: 2026-11-21
type: decision
trigger: 用户 2026-11-21 要求本次修改收敛到头和双臂的软急停；SEV-135 / robot_driver#299
commits: [codex/control-backlog-fixes]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [control-20261121-02]
related: [SEV-135, robot_driver#299, SEV-73, robot_driver#300]
---

## 背景

用户明确把本次交付限定为头部和双臂的软急停。现场普通 IO → Modbus → DDS → 软件停机链
保持当前职责；硬件安全回路设计和认证另行处理，不再列为本软件任务的开发验收项。
本记录覆盖 control-20261121-02 的范围与验收安排，保留其中已经完成的软件锁存和序号修复事实。
加速度来源协调与 robot_description 外置仍是独立任务，本次没有模型变更。

## 改动

- 独立 head_only 显式启用 emergency_stop_response，和左臂、右臂、双臂、双臂＋头部组合
  使用同一台套输入及接收时间配置。实机 artifact 生成输入节点参数与启动清单；mock 不连接物理 IO。
- 未启用响应的实例不创建该输入订阅；底盘、主动悬挂和只读传感器范围不要求这份软急停配置。
  诊断字段名保持原契约，未启用实例的 reported/effective 为 UNKNOWN。
- 头部复用握手接口 enable_request=0 和驱动失能状态机；双臂复用既有 Quick Stop/失能流程。
  停用当前轨迹、头部位置及已选伴随控制器，保留原离线 restart_required。
- 按下、无效输入、过期和失联均走现有锁存。按钮释放不自动使能，未收到控制器停用确认不能复位。
  MoveHead 复用 readiness 结束在途目标，readiness 恢复不重发旧目标。
- 重新生成 head-mock/head-real 冻结快照；补头部单独/伴随、输入短脉冲/过期/无效、范围排除和目标终止回归。

## 验证

T0 原生离线：4 个相关包构建通过；ctrl_enable_manager 的 131 个 C++ 用例通过；
integ_composer、integ_bringup、integ_head_move 的 Python 用例共 352 项通过。
使用本地测试台套时间及 localhost DDS；无现场连接或执行器操作。
初次沙箱 colcon 子进程结束后父进程未退出，结束本任务进程后在原生环境完成构建和包测试。
13 种台架 profile/scope 配置检查通过；quality_gate（含 arch_check）通过。
本机缺 ShellCheck，本轮未修改 Shell；T1 全图及现场验证未执行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本次范围是独立头部、左右臂/双臂及其组合的软急停，其他控制范围不接入本输入。
- F2: 软件开发验收不包含硬件安全回路建设或认证；软急停不替代安全继电器、STO 或驱动保护。
- F3: 软件响应时间仍须现场确认，四个时间参数为 TBD，不能使用测试夹具值作为部署默认值。

## 遗留

按既有分级流程补 T1 全图与 T2–T4 现场验收；软件验收核对按下/失联/过期停止、
头臂控制器停用、释放不自动恢复、显式复位和完整重启锁存，并测量检测至停止的时间。
现场动作由操作台执行，按仓库现行硬件授权与安全条件逐级进行。
硬件安全回路另行处理，其独立验收结论不由本软件测试给出。
