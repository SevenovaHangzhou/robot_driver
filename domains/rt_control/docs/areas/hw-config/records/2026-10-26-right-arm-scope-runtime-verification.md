---
id: hw-config-20261026-03
area: hw-config
title: 授权退出传感器台架并验证右臂9..15实时反馈
date: 2026-10-26
type: commissioning
trigger: 用户报告启动被inactive master检查拒绝，随后明确允许退出占用主站的右腕传感器测试并启动右臂scope
commits: [feature/right-arm-scope-20261026]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - src/rt_control/rt_control_machine_config/evidence/Alfa_V3_bench_right_arm_runtime_2026-10-26.yaml
supersedes: []
related: [hw-config-20261026-02, BQ-146]
---

## 背景

用户执行 `arm start right` 后，被 `Expected inactive master with the declared responder count`
拒绝。实读为18从站、主站Active yes；占用者是此前右腕传感器测试的launch599163和
控制节点599175，并非数量不符。用户在对“退出该传感器测试栈并启动右臂scope，
不复位、不使能、不运动”的明确询问后回答“允许”。

## 改动

无控制源码改动。执行本次授权的生命周期操作并保存运行证据：

- 核实目标控制节点的启动XML仅含 `bluepoint_raw_bench`，无电机关节。
- 向传感器launch发送SIGINT后，其忽略的信号未触发退出；再次检查信号处理状态，
  向该传感器控制节点发送其已注册处理的SIGINT，正常释放主站，未强杀进程。
- 等待主站Active no且18从站全部PREOP，才执行 `arm start right`。
  启动生成11文件，运行目录 `arm-20261026-134603`，返回0，使能管理器初始IDLE。
- 操作台与guard由既有启动入口启动；保留独立右臂位姿文件和现有网页资源。

## 验证

- 启动前主站/从站缓存检查：inactive，18 PREOP，拓扑准入通过。
- 18121公开页面、会话信息与SSE可读，实机标志 `mock=false`，反馈新鲜。
  首次只读网页验证误把ros2启动器也计作服务进程；改按实际服务入口识别后验证通过，
  此过程未重启网页或控制栈。
- 使用只读 `GetParameters` 查询运行中controller_manager的 `robot_description`，
  确认 `right_joint1..7 → 9..15`，没有沿用磁盘产物或软件名称作为物理绑定的唯一证据。
- ROS采得439帧七轴 `/joint_states`，时间戳推进；SSE采得41帧，同期读值最大差0.002747°。
  内部动态状态同时包含七轴position/status_word等真实硬件反馈。
- 实时缓存从站：右9..15全部OP，左1..7全部PREOP，蓝点8/16保持PREOP。
  `ecat_arms` active；`right_arm_jtc`、两个状态广播器、enable_manager及recorder均active。
- 启动后的日志显示外部 `/rt/enable` 请求以及轨迹目标；验证时管理器为ENABLED、七驱动
  operation-enabled。没有将此状态写成“仍未使能”，也没有为维持本次初始IDLE而打断现场操作。
  本次工具只提交登录及读取请求，没有调用复位、使能、FJT或网页运动接口。
- 详细只读报告保存在工控机 `/tmp/right-arm-correction-20261026/authorized-verification.json`。

没有进行逐轴物理运动、方向/零点重标定或新右臂栈的停机验收；现有6项旧模型快照断言
失败仍见纠错记录，不能据此声称全部测试或整机验收通过。

本次实机操作经现场授权，操作人：Codex（用户明确回复“允许”），确认短语入口：
本对话中的“退出传感器测试栈并启动右臂scope”询问及回复；执行入口为正常SIGINT退出与 `arm start right`。

## 结论与冻结事实

- F1: 右臂scope已在18从站现场拓扑上成功启动，运行参数绑定右七轴9..15，左七轴未参与控制。
- F2: 18121已接收新鲜的真实右臂反馈，ROS与SSE同期读值已验证一致。
- F3: 启动时为IDLE，后续外部使能与轨迹请求导致ENABLED；本次agent没有发送复位、使能或运动。
- F4: 主站已被其他台架激活时应正常退出占用者，不能删除inactive/topology检查以强行启动第二个栈。

## 遗留

物理方向/标定与运动验收仍按独立授权执行。保留当前运行状态供现场使用；本次未提交或推送Git。

## 后续源码保存

用户随后要求“本地再保存一个版本并上库”。已从主线建立独立分支
`feature/right-arm-scope-20261026`，保存本次scope、拓扑检查、回归与三条开发记录；
前述“未提交或推送”仅描述实机验证结束时点。其他头部TF、传感器采集和网页资源改动
保留在原工作区，不并入此版本。

现场运维包装器含凭据，不整份入库。仅将新增right选项的差异保存为
[启动器补丁](../assets/2026-10-26-arm-right-scope.patch)，使用零上下文避免带入凭据。
从含 `tools-local/arm` 的运维工作区根目录先备份该私有文件，再执行：

```bash
git apply --check --unidiff-zero /path/to/2026-10-26-arm-right-scope.patch
git apply --unidiff-zero /path/to/2026-10-26-arm-right-scope.patch
bash -n tools-local/arm
```

仅适用于本次已有left/dual包装器；其他版本应人工核对选择函数，不能强行套用。
本地副本已验证补丁应用后与部署候选完全相同，left/right/dual选择、JTC及独立位姿文件
均通过；此验证不执行启动。独立分支重新运行组合器/EtherCAT测试282项及质量门禁通过。

用户之后贴出的左臂启动日志首错为 `Failed to reserve master: Device or resource busy`；
当时另一个测试仍占用主站。末尾class_loader警告属于退出阶段，本次源码保存未再启停现场进程。
