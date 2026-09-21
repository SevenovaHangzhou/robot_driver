---
id: io-power-20260921-01
area: io-power
title: 双 E08 接入八路 A22 超声波
date: 2026-09-21
type: feature
trigger: 用户确认两台 E08 共用 TCP 504，Modbus RTU 站号为 1 和 6，并各接四个 A22
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: ["#45", "io-power-20260917-01", "io-power-20260919-01"]
---

## 背景

既有超声波节点只访问一台 E08 的 unit 1，并发布四路 A22 数据。设备协议确认 E08
站号 2..5 由探头接口保留，第二台 E08 应配置为 unit 6。用户要求两台 E08 共用
`192.168.1.12:504`，总计接入八个 A22；本次不执行设备地址写入或实机通信。

## 改动

- 单值参数 `unit_id: 1` 改为 `unit_ids: [1, 6]`，要求恰好两个不同站号，并拒绝 E08
  保留地址 2..5；通用 Modbus 站号范围仍由传输层校验。
- 每个轮询周期串行读取 unit 1 和 unit 6 各自的 `0x0106..0x0109`，分别映射到
  `channel1..4` 和 `channel5..8`。
- `ultrasonic/raw` 按通道顺序合并八个原始寄存器；Range 话题、frame 参数和诊断扩展到
  八路。诊断 hardware ID 同时标识 E08 unit 与该 E08 内部通道。
- 保持既有批次语义：两次读取都成功后才发布该周期的八路数据；任一请求失败时不发布
  陈旧或不完整测量，仅发布通信错误诊断。
- 两次 E08 请求串行执行，但每次四寄存器连读仍同时触发该 E08 的四个探头；本次不实现
  声学串扰规避调度。
- 驱动发布的视场角元数据及参数校验改为 40 度（`0.6981317008 rad`）；A22 角度等级 2
  由用户后续单独写入，本次驱动保持只读，不写设备配置寄存器。
- 更新包内协议说明、直连只读检查命令、参数与请求编码测试。

## 验证

- 隔离构建 `modbus_tcp_rtu485`、`rt_control_bringup`：PASS，2 packages。
- `node_lifecycle`：PASS；八路 40 度 YAML 可离线启动，非 40 度参数和 E08 保留站号
  `[1, 2]` 被拒绝，测试关闭轮询且未连接硬件。
- `test_modbus_transport`：PASS；覆盖 unit 1/四寄存器与 unit 6/四寄存器的 Modbus TCP
  请求编码、响应解析、异常和超时。测试使用本地 Unix socketpair，需在允许本地
  socketpair 通信的执行环境运行。
- `test_ultrasonic_launch.py`：PASS，4 passed，`ROS_LOG_DIR` 指向 `/tmp`。
- `git diff --check`：PASS。
- `tools/quality_gate.sh`：PASS，297 passed / 13 skipped，门禁覆盖率 83%；本机缺少
  `shellcheck`，由 CI 强制检查。

未连接 `192.168.1.12:504`，未修改 E08 站号或 A22 角度寄存器，未验证第二台 E08 或
新增四只 A22。
本记录不授权写设备、使能或运动。

## 结论与冻结事实

- F1: 两台 E08 共用 TCP 504，驱动站号配置为 unit 1 和 unit 6；E08 站号 2..5 不可用于
  两台适配器寻址。
- F2: 八路 A22 映射为 unit 1 和 unit 6 各四路，ROS 通道顺序为 1..8。
- F3: 一个周期包含两次串行的四寄存器 FC03；只有两次都成功才发布同批八路 Range、
  raw 和通道诊断。每台 E08 内部四个探头仍同时触发。
- F4: ROS `Range.field_of_view` 固定发布 40 度；实机必须另行将八个 A22 配置为角度等级 2，
  软件参数不会改变探头实际检测角度。

## 遗留

- 单独隔离第二台 E08 后将其站号从默认 1 配置为 6，并在重新并总线前只读确认。
- 将八个 A22 的检测角度寄存器配置为等级 2（约 40 度），并逐个回读确认。
- 实机验证两台 E08 共线时的八路读数、周期时延、断线诊断和长期稳定性。
- 评估八个探头的声学串扰；若需要规避，应另行改为逐寄存器分时触发。
- 测量八个探头相对 `base_link` 的安装外参并替换通道占位 frame。
