---
id: ecat-axes-20261017-01
area: ecat-axes
title: 单臂无夹爪台架的物理配置与仅使能 scope
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 要求在台架工控机部署 #80，并为单臂无夹爪连接补一个 scope，不破坏分层
commits: [feature/electri-127-bench-left-arm-scope]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: [src/rt_control/rt_control_machine_config/evidence/Alfa_V3_bench_left_arm_scan_2026-10-17.yaml]
supersedes: []
related: [ecat-axes-20260912-04, chassis-control-20261017-01]
---

## 背景

台架工控机 `ar-Default-string`（192.168.100.40）通过 GR10 分支器连接一条七轴零差臂，
夹爪驱动已摘除，X3 无连接。主站 Idle 时只读扫描结果：9 个从站，主分支器 port3(X2)→1..7，
port1(X3) 断开，EBUS port2→子分支器 8；7 轴身份与 `zeroerr_cia402_*` profile 一致，
均为 PREOP、无故障位、`0x6040=0`、`0x603F=0`、`0x6061=8`。
`arms` 模块描述双臂 + 夹爪（18 位），因此这个台架需要另外一份物理配置。

## 改动

只加数据，不改组合器、渲染器和 owner schema：

- `machines/Alfa_V3/bench_left_arm.yaml`：新模块，`zeroerr` 族，只含 `left` 分支（X2，J1..J7 位于 1..7），
  主分支器在 0，子分支器在 8。方向为 `TBD`，`branch_mapping_verified: false`。
- `manifest.yaml`：新增模块 `bench_left_arm`、物理配置 `bench_left_arm`
  （ring 0..8）、scope `bench_left_arm_stationary_enable`。该 scope 的渲染模式为
  stationary_enable，周期 250 Hz（BQ-154），分步激活硬件，不含运动控制器。
  另新增 `policy_refs.bench_left_arm_enable`。该模块不列入 `functional_modules`。
- `enable_manager/config/Alfa_V3_bench_left_arm_enable_policy.yaml`：用 `enable_groups`
  按模块和模式组生成每批 1 轴的批次，时序值与手臂策略相同。
- `units/Alfa_V3-bench-01/`：台架单元。零位是 2026-10-17 读到的 0x6064 候选值，
  `confirmed_by: pending`，在用户确认之前，渲染器拒绝 stationary 组合。
- `rt_control_scope_runtime.launch.py`：如果 scope 没有 inactive 控制器，硬件激活后直接加载
  lifecycle 控制器。原因是 Humble spawner 至少需要一个控制器名。
- 组合器测试改为显式选择 `units/Alfa_V3-01`，不再依赖 glob 顺序选第一个单元。

宿主环境：本机原装 IgH 只带 dc-offset 补丁，`doctor` 拒绝。已按 `hostsetup/igh-install.sh`
重编并安装 stable-1.6 `2f7f884`，带 0001/0002/0003 补丁。临时副本只替换了本机 EtherCAT MAC
`8c:59:3c:14:ff:d3` 和仓库路径。旧安装备份在 `/var/lib/rt-control/igh-backup-20261017`。

## 验证

- 原生完整构建 41 包通过；新 IgH 下重编 41 包通过，`doctor` PASS。
- `pytest rt_control_composer/test` 与 bringup scope/entry 测试：225 passed。
  新增用例包括：候选零位导致 real/mock 都被拒绝；确认零位后，mock 输出 7 轴单批使能和分步激活计划；
  real 拓扑与扫描一致，并用注入的 CLI 输出完成预检的通过、数量错误和接线错误三种情况。
- `robot_hw_ethercat` 55 passed，`tools/tests` 247 passed / 13 skipped，
  `check_v3_branch_contract`、`rt_control_layer_gate`、`validate_machine_profile`（台架单元与 Alfa_V3-01 --all）通过。
- 实机：只读预检（master/slaves/upload）对派生拓扑 PASS。未启动 ROS 图，未使能，未运动。

## 结论与冻结事实

- F1: 台架 9 位拓扑见 evidence；主分支器 port3→1、port2→8，port1 断开。
- F2: J1..J6 固件 150403170，J7 为 550403170；`0x60FE=1`。

## 遗留

- 用户确认台架 J1..J7 物理顺序和零位姿态后，才能把 `confirmed_by` 改为 `user`。
- 主机身份 `host_id`，以及驱动器在 OP 下的状态字行为（PREOP 下读到 `0x0000`）待核对。
- 仓库 `hostsetup/igh-install.sh` 的 EtherCAT MAC 写死为另一台 IPC，多主机部署需要单独处理。
