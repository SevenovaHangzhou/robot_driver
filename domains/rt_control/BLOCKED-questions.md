# rt-control 阻塞与裁决记录

2026-10-29 起重新记录。此前的 161 条（BQ-001 至 BQ-161）已按用户裁定从本文件清除，
原文可从 git 历史取回（清除前最后版本见 `main` 上的 `43d4c64`）。

## 记录规则

- 只记两类：缺少权威事实而阻塞的问题，以及用户作出的裁决。
- **写入前必须经用户确认**：先把拟记录的条目（问题、证据、影响范围、建议）发给用户，确认后再提交。
- 编号从 `D-001` 起递增，不复用旧 `BQ-` 编号。
- 每条格式：

```text
## D-xxx — <一句话标题> [OPEN | RESOLVED yyyy-mm-dd]
- 问题：
- 证据：
- 影响范围：只阻塞哪些任务/模块
- 裁决：（RESOLVED 时填写，注明裁决人）
```

## 条目

## D-001 — 头部使能并入 enable_manager，待实机验证 [OPEN]

- 问题：达妙头部的使能、失能、清错已由 enable_manager 以"握手式设备"方式负责，原
  `damiao_head_controller` 与 `/rt/head/*` 删除（PR #110，arch-s7）。软件测试与 mock 启停已通过，
  实机行为尚未验证。
- 证据：PR #110 的验证记录（enable_manager 98 项测试，含 10 项握手设备用例；head_only mock 启停）。
- 影响范围：只阻塞 #110 及其后续 PR 的合并与头部实机使用；不影响双臂、底盘。
- 待验证项：启动后 IDLE 且两台未使能；`/rt/enable` 两台使能并激活位置控制器；`/rt/disable`
  先停位置控制器再失能；故障锁存进入 FAILED、`/rt/reset_fault` 回到 IDLE；使能中 SIGTERM 先失能再退出。
- 裁决：用户 2026-10-29 要求合并后实机测试；结论待填。

## D-002 — 统一三份 SocketCAN 实现 [OPEN]

- 问题：`robot_hw_can`（达妙，跑在 ros2_control 实时 read/write 中）、`bms_node`（29 位扩展帧，
  失败后重连）、`lpms_nav3_can`（只收、拒绝 can0/can1）各自实现打开、过滤、收发。
- 证据：架构梳理差异表（2026-10-29 会话）；`robot_hw_can/src/socketcan.cpp`、`bms_node/src/bms_node.cpp:245`、
  `lpms_nav3_can/src/lpms_nav3_can_node.cpp:245`。
- 影响范围：不阻塞现有功能；属于 `ARCHITECTURE.md` 第 5 步（抽 infra 库）。
- 裁决：用户 2026-10-29：统一关闭"接收自身发出的帧"；共用部分抽成公共库，后续再做。
  抽取范围、`can0/can1` 保留检查是否进公共库，动手前再确认；涉及实时路径，需要实机验证。
- 细化裁决（用户 2026-10-30）：
  - 自收帧（`CAN_RAW_RECV_OWN_MSGS`）显式关闭，等同 Linux 默认，不改变行为；本机回环
    （`CAN_RAW_LOOPBACK`）保持开启，便于 `candump` 排查。
  - 公共库 `infra_socketcan` 只管传输（打开、过滤、绑定、非阻塞收发、等待、接口序号）；
    重连策略、过滤器构造、协议解码留在各包；打开失败抛异常，收发与等待返回状态、不抛异常。
  - `can0/can1` 保留检查不进公共库，从 `lpms_nav3_can` 删除：接口归属是台套部署事实。

## D-003 — 统一两个 Modbus TCP 客户端 [OPEN]

- 问题：`plc_io_modbus`（长连接、固定 1 s 超时、getaddrinfo、FC01–06）与 `modbus_tcp_rtu485`
  （每次请求重连、可配置超时、只接受 IPv4）各自实现连接、收发与报头校验。
- 证据：架构梳理差异表（2026-10-29 会话）；`plc_io_modbus/src/modbus_client.cpp`、
  `modbus_tcp_rtu485/src/modbus_transport.cpp`。
- 影响范围：不阻塞现有功能；属于 `ARCHITECTURE.md` 第 5 步。`plc_io_modbus` 负责写真空泵与阀，改动需实机验证。
- 裁决：用户 2026-10-29：统一为长连接；超时可配置；地址解析统一用 getaddrinfo；后续再做。
- 细化裁决（用户 2026-10-30）：
  - `modbus_tcp_rtu485` 每次请求重连只是当初为了写得简单，网关可以长连接。
  - 任何错误（含超时）都关闭连接，下一次请求再连，避免迟到回复被下一次请求读到。
  - 读写都不自动重试（与 `plc_io_modbus` 现状一致；写操作结果未知时不得假装成功）。
  - getaddrinfo 加 `AI_NUMERICHOST`，只接受数字地址；超时按端点配置，`plc_io_modbus` 默认仍为 1 s。

