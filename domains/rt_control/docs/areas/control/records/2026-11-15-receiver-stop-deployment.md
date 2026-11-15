---
id: control-20261115-01
area: control
title: 执行周期停止候选在双臂头部实机未使能部署
date: 2026-11-15
type: commissioning
trigger: 用户确认沿用 j6-stop-continuity 任务标识并授权现场协调后的控制栈及18121切换
commits: [de2ae668b737fbd2edf7d2b21b77c5d4ce818b30]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - Dynamic_Parameter_Identification/artifacts/j6_stop_repair_20261114/release/target-evidence/deployment-result.json
  - Dynamic_Parameter_Identification/artifacts/j6_stop_repair_20261114/release/target-evidence/real-start-verification.json
  - Dynamic_Parameter_Identification/artifacts/j6_stop_repair_20261114/release/target-evidence/web-verification.json
supersedes: []
related: [control-20261114-02]
---

## 背景

上一条记录完成源码验证与目标机隔离 mock。用户在本会话明确批准任务标识例外，并确认
现场有人监护、其他操作暂停，允许有序停止双臂＋头部栈、通过已有权限包装器启动候选，
保持未使能、不运动，随后仅重载 18121。

## 改动

无控制算法新增。提交 `de2ae668b737fbd2edf7d2b21b77c5d4ce818b30` 已推送
`codex/j6-stop-continuity` 并核对远端 SHA。
目标 `ar@192.168.100.40` 选择独立目录
`/home/ar/rt-control-arch-20261106/releases/j6-stop-20261114-candidate`，
加载目标机重新编译的插件及私有接口、重新生成的现场 artifact 和网页适配副本。
网页保留现场共享控制权、序列、提交隔离和视图配置；精确文件版本以候选部署清单 SHA256 为准。

## 验证

- 旧栈：向核验后的 `rt_control_start` 发送 TERM；退出日志确认 motion_inactive、
  drives_disabled、controllers_inactive 均为 True，ecat_arms 与 damiao_head 均 cleaned=True。
- 新栈：复用既有 start_bench/start_with_caps，ROS 域 77。新停止控制器、rolling、头部运动
  控制器均 inactive；enable manager、两侧力发布及 Runtime-LOG recorder active。
- 实机只读：enable manager 为 IDLE，头部 motion_allowed=false；14 个关节状态字均未处于
  CiA402 Operation Enabled；16 个关节反馈新鲜，约 3 秒分别接收左右六维力 3001/2999 帧。
- 私有停止服务已发现；没有调用停止服务、使能或运动进行功能试验。
- 18121：新进程返回 `dual_arm_stoppable_jtc`、IDLE、控制权 idle/none、无运动无序列。
  已服务的 JavaScript 哈希与候选文件一致；148 个位姿、原档位、两侧 J6 ±110° 限制保留。
  网页当前不能执行运动的原因是“手臂未使能”。
- 18200 原 PID 3445630、夹爪网关原 PID 1752748 均保持运行。
- 插件 SHA256：`788fe46a418dc41938681ed43cd2f87c8e7abad1d841d3bb9db584e0a8d308d1`。

本次实机操作经现场授权，操作人：Codex（用户现场监护），确认短语入口：本会话
“现场已协调，允许按上述方案切换”；执行入口为候选目录 `activate_stack.py`、`activate_web.py`。
本次没有发送使能或运动请求；有序停栈包含已授权的失能。

## 结论与冻结事实

- F1: 候选已完成双臂＋头部实机未使能启动及 18121 切换，验证层级为 T3 生命周期。
- F2: 实机参考制动连续性及物理停止表现尚未验证；T3 不替代 T4。
- F3: 旧 artifact 与安装目录保留；候选含配套启动和回退入口。现有 `arm/arm-new` 快捷入口
  没有改写，之后使用旧快捷入口重新启动仍会选择旧配置。

## 遗留

现场小幅低速停止试验、T4 连续性日志及生产容器镜像验证待执行。再次启动此候选须使用候选
`start-candidate.sh` 并按其配套网页入口操作；既有快捷入口的持久化切换不在本次范围。
