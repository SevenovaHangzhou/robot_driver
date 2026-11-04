---
id: ecat-axes-20261017-01
area: ecat-axes
title: 单臂无夹爪台架的物理配置与仅使能 scope
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 要求在台架工控机部署 #80，并为单臂无夹爪连接补一个 scope，不破坏分层
commits: [feature/electri-127-bench-left-arm-scope]
env: native
risk: T3
writes: {reset: yes, enable: yes, motion: no, plc: no, esc_diag_counters: cleared}
verified: PARTIAL
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
  stationary_enable，分步激活硬件，不含运动控制器。初版照抄 `arms_stationary_enable` 的 250 Hz（BQ-154）覆盖；
  用户 2026-10-17 决定台架 scope 使用 1 kHz 主周期，已去掉该覆盖。
  另新增 `policy_refs.bench_left_arm_enable`。该模块不列入 `functional_modules`。
- `enable_manager/config/Alfa_V3_bench_left_arm_enable_policy.yaml`：用 `enable_groups`
  按模块和模式组生成每批 1 轴的批次，时序值与手臂策略相同。
- `units/Alfa_V3-bench-01/`：台架单元。零位初版是读到的 0x6064 候选值（`confirmed_by: pending`），
  按 F11 改为 7 轴均为 2^18（262144），由用户确认。
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

- F3: 用户于 2026-10-17 确认台架总线顺序为物理 J1..J7，零位候选姿态即机械零位，
  `confirmed_by` 改为 `user`。
- F4: J1–J2 链路间歇断开。Idle 下 dmesg 记录 340 次，间隔 18.5–205 s（中位 24 s），
  每次约 1.5 s，只剩 0/1/8 三个从站在线。J1 port1 与 J2 port0 的 RX 错误和 lost link 计数都已饱和（0xFF），
  J3..J7 port0 为 0。结论：故障在 J1–J2 段线缆或接头，不是 J2 掉电。IgH 重装前后都有，
  与软件无关。

- F5: 18:15 换掉 J1–J2 线缆。换线后不再断链，清零 ESC 计数 120 s 后仍全为 0。
- F6: 拉 OP（scope 运行入口，当时为 250 Hz，驱动保持禁能），从 launch 开始计时：主站 0.73 s 激活，
  7 个驱动逐个进 OP，第 7 个在 25.89 s 进入。每个驱动 PREOP→OP 耗时依次为 5.75/5.81/1.86/5.79/3.97/0.78/0.77 s。
  J1、J2、J4 报 `Slave did not sync after 5000 ms`，DC 同步等待是主要耗时。
- F7: 启动线程交接给周期线程的时刻，总线出现一次 0 从站应答，随后 3559 个 datagram UNMATCHED，
  并重新扫描 11.4 s。期间 ESC 计数保持 0，所以这是主站时序问题，不是链路问题。恢复后保持 OP，
  60 s 内无丢帧，WKC 计数不再增长，`/joint_states` 250 Hz，状态字 0x1250（Switch On Disabled）。
- F8: 启动线程退出前，`rt_control_thread_affinity.py` 会看到两个 FIFO80 线程并拒绝执行。
  所以周期线程要在交接之后再绑到 CPU14。

- F9: 用户给驱动器断电再上电，J2/J4/J5/J6 的 0x603F=0xA000（手册 7.2.27：主站掉线或退出 OP）被清除。
  上电后原始位置大幅变化：J1 21641→262148，J2 247→262142，J3 212348→494139；J4–J6 本来就在 2^18 附近；J7 +7004。
  读数接近 2^18 的轴其实就在零位（见 F11），之前推测的"上电初始化为半圈"不成立。
- F10: 1 kHz 下复测。控制器周期线程在 launch 后 0.73 s 出现，当即绑定 CPU14，早于硬件激活（0.81 s）。
  7 个驱动在 3.91 s 内全部进 OP，每个 0.41–0.46 s。没有 DC 同步超时，交接时没有重扫。60 s 稳态无丢帧、
  WKC 错误为 0，`/joint_states` 1000 Hz。这次同时改了周期、绑核时机，并且刚上过电，还没有分别归因。
- F11: 用户 21:18 确认：驱动零位为 2^18（524288 的中值），等于机械零位，也等于 URDF 零位，J1..J7 全部适用。
  零位文件改为 7 轴均为 262144。当时读数：J1/J2/J4/J5/J6 偏离 2^18 不超过 8 counts，J3 约 +159°，J7 约 −72°
  （这两个轴当时不在零位姿态）。
- F12: 用户现场要求使能。7 轴均为 CSP（0x6060=8），没有 PP。1 kHz 下 `/rt/enable` 用时 1.256 s
  （每批 1 轴，批间隔 0.2 s），7 轴状态字都是 0x1637（Operation Enabled），原地保持 10 s。
  保持期间位置最大偏移 7–15 counts（不超过 0.01°）。之后 `/rt/disable` 用时 0.010 s，状态字回到 0x1250，
  WKC 错误为 0，dmesg 无新消息。这次没有命令过位置，不算运动验证。
- F13: 用户对照 `robot_alfav3_mobile.urdf`（4d4ad9a，见 integration-20261017-02）确认台架方向：
  J2、J3 与 URDF 正向相反（-1），J1、J4–J7 相同（+1）。`bench_left_arm.yaml` 已写入，`direction_status` 改为用户确认。
  首次点动时需实测核对。
- F14: 有序停止（OP→PREOP，四阶段均 True）后，J1 的 0x603F 再次锁存 0xA000（手册 7.2.27：主站掉线或退出 OP）。
  预检只放行编码器电池码，所以下一次启动会被拒绝；需要裁定处理方式（断电、PREOP 下 SDO 故障复位，或把 0xA000 列为
  启动可复位码交给 enable_manager）。

- F15: 用户决定 0xA000 视为启动可复位码。ZeroErr 两份 profile 增加 `startup_reset.codes: [40960]`，
  渲染器把它并入每轴 `allowed_errors`（J3 另有 0x730F）。实机发现有序停止后 J1 不只是留码，而是处于故障态
  （0x6041=0x0208），所以预检改为：已使能或控制字非零仍拒绝；故障态只有在 0x603F 属于声明码时才放行；
  其余非零错误码仍拒绝。实机复测：J1 故障 + 0xA000 下预检通过，3.93 s 进 OP，enable_manager 报 failed，
  `/rt/reset_fault` 成功后 J1 变为 0x1250（Switch On Disabled），enable_manager 回到 operational，全程未使能。
  这次有序停止后 7 轴均无 0xA000，可见该码是否锁存不固定。
- F16: 同次运行中，主机上另有他人的 mock `ros2_control_node`（Domain 91），本地预绑核脚本误选了它，
  本栈周期线程未在交接前绑到 CPU14，交接时再次出现 AL 状态报文超时与 2304 个 UNMATCHED，WKC 累计 989 后不再增长。
  这从反面支持“交接前绑核可消除交接扰动”（F10）。脚本已改为只匹配本栈产物路径。

- F17: 首次操作台实机运动（休闲档，跟踪误差最大 0.275°）中，用户发现 J2 实际方向与模型相反，F13 的 J2 判断有误；
  J2 改为 +1，现为 J3 = -1、其余 +1。改前已有序停止（四阶段均 True），停止后 7 轴均锁存 0xA000（J1/J2/J7 处于故障态）。

- F18: 2026-10-18 用户在操作台运动中发现 J6 实际方向与模型相反，J6 改为 -1，现为 J3、J6 = -1，其余 +1。
  改前已有序停止（四阶段均 True，最大跟踪误差 0.276°）。

- F19: 2026-10-18 用户澄清：F17/F18 之后，J3、J6 在实机和模型里的转向一致，但正角度对应编码器计数减小方向。
  用户规定：关节正方向 = 编码器计数增大方向。方向系数 -1 不能满足这一点，因为它同时翻转指令和反馈。
  所以改为 7 轴方向系数全部 +1，同时在 robot_description#5 里把左 J3、J6 的模型轴取反（`5ec6b65`，
  tree `4706b7a`），保持实机和模型一致。F18 的 J6 = -1 作废。需要在低速档下实机复核 J3、J6（2–5°）。

- F20: 2026-10-19 为 Runtime-LOG 黑匣子扩展台架零差 TxPDO（0x1A00）：在位置、数字输入、状态字之后增加
  0x606C 实际速度（`velocity`，rad/s，输出侧系数与方向）、0x6077 实际力矩（`torque_permille`，额定力矩千分比，乘方向）、
  0x603F 错误码（`error_code`），共 18 B。新配置 `zeroerr_cia402_csp_diag` 只用于台架模块，整机暂不切换。
  台架固件：J1–J6 软件 150403170、J7 软件 550403170，硬件均为 1005（0x100A/0x1009，PREOP 下只读）。
  实机（失能、无运动）：7 个驱动进入 OP，WKC 错误 0，两种固件的新通道都有读数：速度噪声 ≤ 1.5e-4 rad/s，
  力矩 −2…+1 ‰，错误码 0。速度与力矩的符号和尺度还要在一次小幅运动中复核。
  A/B（600 s）：标准差 0.828 µs，偏差 p99.9 5 µs、最大 9 µs，丢帧 0。

- F21: 2026-10-19 ICube 补丁 0014：RxPDO 通道可声明 `mirror_state_interface`，每周期写完后把出站帧里的值读回、
  换算成接口单位放进状态接口。台架镜像目标位置（`target_position`）与控制字（`control_word_sent`）。
  实机（失能、无运动）：全部 OP，WKC 错误 0；失能时 `target_position` 是插件预加载的原始位置（与实际位置差
  约 4 counts），`control_word_sent` 为 0。A/B（600 s）：标准差 0.501 µs，偏差 p99.9 5 µs、最大 9 µs，丢帧 0。

## 遗留

- 把交接前绑核做进正式启动流程（F10/F16），不再依赖本地脚本。
- 分别归因 F10 的提速和重扫消失（1 kHz 不预绑、250 Hz 预绑各测一次）；把预绑核做进正式启动流程。
- 缩短 DC 同步等待（J1/J2/J4 各 5 s 超时）：核对 SYNC0 周期、shift 与驱动的 DC 窗口。
- J3 是单圈不回绕策略，零位在 2^18 时回绕点在 ±180°，当时 J3 在 +159°，距回绕点约 21°；需确认 J3 的实际运动范围。
- 主机身份 `host_id`，以及驱动器在 OP 下的状态字行为（PREOP 下读到 `0x0000`）待核对。
- 仓库 `hostsetup/igh-install.sh` 的 EtherCAT MAC 写死为另一台 IPC，多主机部署需要单独处理。
