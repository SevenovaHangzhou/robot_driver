---
id: ecat-axes-20260912-03
area: ecat-axes
title: 三代机双臂 X2/X3 接线与十八从站只读核验
date: 2026-09-12
type: investigation
trigger: ELECTRI-118，用户确认左臂 X2、右臂 X3
commits: []
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-146, ecat-axes-20260912-02, release-deploy-20260912-01]
---

## 背景

分支器 ESI 已归档，用户补充左右臂端口。此次仅保存接线与实际只读扫描事实，
目标资产为 robot_hw_ethercat 的硬件记录和 bringup manifest 注释；无公共接口变更。

## 改动

无执行代码改动。更新 ESI README、manifest 待核验事项与 BQ-146；保留 draft 和
运行布局占位，扫描所得分支归属不等于链内关节/夹爪绑定。其他物理组合不套用该环序。

## 验证

- 在已部署测试主机执行 `ethercat master`、`ethercat slaves`、`ethercat slaves -v` 和
  `ethercat slaves -p 0,17 -v`；均成功。原始详细输出另存于隔离部署目录的
  `arms-topology-slaves-20260912.txt`，不包含 SDO upload/download 操作。
- master 为 Idle、Active=no、Link UP；18 从站均为 PREOP。该状态只证明发现设备，
  不等于 CiA402 OperationEnabled、4 ms 周期稳定或可执行运动。
- 主设备 port 3 NextSlave=1、port 1 NextSlave=9、port 2 NextSlave=17。
  链 1..8 与 9..16 的相邻 NextSlave 连续，8/16 下游端口关闭；子设备 17 的外部端口关闭。
- 分支器身份与 ESI 相符，名称显示 1.4.2.0 而 ESI 显示 1.4.2.3；未据此宣称固件版本一致。
- 本地 `tools/quality_gate.sh` 通过：214 tests，受门禁代码覆盖率 83%；ShellCheck 本机
  未安装。manifest 的 10 个 profile/scope 静态组合与差异空白检查通过。
- 未启动控制栈、改变总线状态、写 SDO、复位、使能或运动。PDO/SDO、运行状态/DC 与
  关节/模式绑定不在此次扫描的已验证范围内。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本次 arms_only 接线为左臂 X2、右臂 X3；结合 ESI 与实际链路，左臂占绝对位置
  1..8，右臂占 9..16，主/子分支器占 0/17；共 18 响应设备，16 电机。
- F2: 分支器 Vendor=0x00100000，主/子 Product=0x10F40931/0x10F40932，
  Revision=0x00010000，存储 alias=2/7；alias 不是绝对 ring position。
- F3: 十六电机均读到 Vendor=0x5A65726F、Product=0x00029252、Revision=0x00000001、
  Serial=0。相同身份无法区分物理关节或确定哪两个是 PP 夹爪。

## 遗留

确认每臂沿总线的 J1..J7/夹爪顺序，再补 owner-local 运行布局和 PDO/SDO。
新分支器运行状态/DC、设备名称版本差异与目标 IgH PDO 保留补丁检查仍未闭环；
静态 launch 继续保留运行准入限制。
