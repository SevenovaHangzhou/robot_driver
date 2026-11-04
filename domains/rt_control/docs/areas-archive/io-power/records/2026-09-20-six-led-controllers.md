---
id: io-power-20260920-01
area: io-power
title: Modbus LED 控制器由四路扩展为六路
date: 2026-09-20
type: feature
trigger: 用户确认六个 LED 控制器共享 TCP 502，RTU 站号依次为 1..6
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: ["#45", "io-power-20260916-01"]
---

## 背景

既有 LED 驱动仅创建四路订阅并要求四组网关参数。用户确认现有拓扑扩展为六个
LED 控制器，全部使用 TCP 502，Modbus RTU 站号依次为 1、2、3、4、5、6。

## 改动

- LED 控制器数量从 4 改为 6，颜色话题扩展为 `led0/color`..`led5/color`。
- 节点默认参数和 `led_strip.yaml` 改为六个 TCP 502、六个 RTU 站号 1..6。
- 参数校验要求端口和站号数组都必须恰好包含六项。
- RGBW 到 FC16 寄存器的编码、失败不重试和退出保持最后颜色等语义不变。

## 验证

- 隔离 `colcon build --packages-select modbus_tcp_rtu485`：PASS，1 package。
- YAML 离线解析：PASS，端口为 `[502, 502, 502, 502, 502, 502]`，站号为
  `[1, 2, 3, 4, 5, 6]`。
- `node_lifecycle`：PASS；六路 YAML 可启动且旧四项端口配置被拒绝。
- `test_modbus_transport`：PASS；覆盖六路参数、FC16 编码、响应、异常与超时。该测试使用
  本地 Unix socketpair，需在允许本地 socketpair 通信的执行环境运行。
- `tools/quality_gate.sh`：PASS，297 passed / 13 skipped；本机缺少 `shellcheck`，由 CI
  强制检查。
- 未连接网关，未发送 LED 写命令。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 六个 LED 控制器共享网关 TCP 502，RTU 站号依次为 1..6。
- F2: 六路域内颜色话题为 `led0/color`..`led5/color`，保持既有 RGBW/FC16 语义。

## 遗留

- 未执行六路 LED 实机写入、断线或退出保持验证。
