---
id: peripheral-control-20261118-02
area: peripheral-control
title: RS485-ETH-M04 网关下两台 E08 与 LED 的请求节奏、迟到应答与超时口径
date: 2026-11-18
type: fix
trigger: ELECTRI-195 / #248；用户 2026-11-18 在 .40 台架接入两台 E08（HC2）和 LED 控制器（HC0），要求测试并修复 main
commits: [bugfix/modbus-gateway-stale-reply]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - .40 ~/periph-led-ultrasonic-main-20261118/run-*（main 005cc591 实机，只读）
  - .40 ~/periph-led-ultrasonic-fix-20261118/run-*、led-*（本分支实机）
  - 网关配置备份（四口 9600 8N1、通用TCP转RTU、RTU 接收超时 1000 ms）
supersedes: []
related: [peripheral-control-20261110-01]
---

## 背景

台架网关 RS485-ETH-M04（台架 IP 192.168.1.14，原 192.168.1.12 与 DIO 冲突，现场改址）HC2/504 挂两台
DYP-E084F-V2.0（站号 1、6，9600），HC0/502 挂 WE-10x LED 控制器（站号 2、3 在线）。main 的
`ultrasonic_node` 实机 25 s 内站号 1 仅 2 次成功、站号 6 为 0；E08、接线和网关配置用独立脚本读取正常。

## 现场观测（.40，只读，2026-11-18）

- 同一连接上收到应答后立即发下一请求：20 次仅 1 次应答；间隔 10/20/50/100 ms：均 20/20，单次约 135–146 ms。
- 网关把已放弃请求的应答排队，在下一条 TCP 连接上先送出（新连接未发请求即收到旧应答）。main 的
  `infra_modbus` 超时后断开重连，再把事务号不匹配判为错误再断开，此后每次错位一拍，无法自行恢复。
- 离线站号网关不回任何报文，占用 RS485 约 1000 ms（RTU 接收超时）；其间新请求排队，到时才处理。
  驱动超时 500 ms 时，离线站号后的下一请求也超时。
- 每口仅一个客户端；断开后立即重连会被拒绝（connect failed），导致下一个在线站号也报通信错误。

## 改动

- `infra_modbus::Client`
  - 迟到应答：事务号属于最近 64 个已发请求的应答被丢弃，继续等待本次应答；其他事务号仍为帧错误并断开。
  - 回复超时（请求已发完、未收到任何应答字节）首次保持连接，连续第二次断开；连接阶段超时、半帧超时仍断开。
  - `set_min_request_gap()`：同一客户端两次请求之间的最小空闲时间，默认 0（PLC 行为不变）。
- `periph_led_ultrasonic`：两节点新增 `request_gap_ms`（默认 20，0..1000）；随包配置
  `response_timeout_ms` 与 LED `exit_response_timeout_ms` 改为 1200 ms（须大于网关 RTU 接收超时）。
  随包配置的 `gateway_ip` 在 #295 中仍为 192.168.1.12；后续变更改为 192.168.1.14（两节点默认值同改），DIO 保留 .12。

## 验证

- T0（.40 构建）：`infra_modbus` 14 项客户端测试（新增 5 项：新连接上的旧应答、旧事务号窗口、请求间隔、
  连续超时断开、半帧超时断开）；`infra_modbus`、`periph_led_ultrasonic`、`periph_plc_io` 共 27 项测试通过。
- T2 超声（网关覆盖为 192.168.1.14，隔离 ROS 域）：40 s 两台各 124–125 次读取，0 通信错误，周期均值 318 ms；
  300 s 拔站号 6 约 28 s：站号 6 期间 0 条 Range、状态为通信错误，接回后自动恢复；站号 1 期间 0 次错误，
  周期 1.38 s（离线站号占用约 1.2 s）。
- T2 LED（站号 2、3，10% 亮度，用户授权）：写确认 38–59 ms；写离线站号 1 超时 1220 ms 后，下一次写站号 2 成功；
  非法 strip_id 被拒；SIGINT 正常退出 5.1 s（4 个离线站号各 1.2 s），站号 2、3 读回 RGBW 全 0。
- 未验证：LED 实际发光由现场目视，未记录；网关断电/网线断开；六路 LED 全在线时的退出时间。

本记录不授权设备参数修改、使能或运动。

## 结论与冻结事实

- F1: 该网关同一端口连续请求须间隔 ≥10 ms（取 20 ms）；迟到应答会在后续连接上送达，客户端按事务号丢弃。
- F2: 网关对离线站号不回报文并占总线约 1000 ms；驱动超时须大于该值（取 1200 ms），否则在线站号受牵连。
