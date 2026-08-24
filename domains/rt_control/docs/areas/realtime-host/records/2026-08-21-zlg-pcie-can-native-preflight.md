---
id: realtime-host-20260821-01
area: realtime-host
title: ZLG PCIe-9140I native CAN 预检迁移
date: 2026-08-21
type: fix
trigger: ELECTRI-104
commits: []
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - ELECTRI-104
supersedes: [BQ-100-native-path, BQ-121-current-hardware]
---

## 范围

`tools/rt_control_native.sh` 不再按两只 CANable USB 序列号准备总线。启动前显式加载
`zpcican`，按 PCI vendor/device、driver 和每个 netdev 的 `dev_id` 找齐四个通道，固定
L0 为 CANopen `can0`、L1 为 BMS `can1`，其余通道保持为未启用的 `pciecan2/3`。

两条生产通道均配置为 500 kbit/s、txqueuelen 128；校验最多等待 5 秒，并要求管理态
UP、ERROR-ACTIVE、bitrate 和队列长度全部一致。未知硬件占用保留名称或身份不符时拒绝
启动，不回退到 CANable2。

## 验证

- TDD RED：最新 `main` 上 focused pytest 为 3 failed / 47 passed，三个失败均来自缺失的
  PCIe 身份、端口映射和 ready-wait 契约。
- TDD GREEN：实现后 `python3 -m pytest -q tools/tests/test_rt_control_native.py` 为
  50 passed；`bash -n tools/rt_control_native.sh` 通过。
- 本地 `tools/quality_gate.sh` 为 192 passed，门禁覆盖率 83%；本机缺 ShellCheck，仍由
  CI 强制执行。
- 目标机只读观测：`can0 dev_id=0x0`、`can1 dev_id=0x1`，两者均为 ZLG
  `10b5:9140`/`zpcican`、500 kbit/s、ERROR-ACTIVE、错误计数为零；两路 RX 均增长。

## 未验证与边界

- 用户启动时暴露的 `can1` 瞬时状态竞态和成功分支裸 `return` 已修正，但修正后的完整
  one-click 启动、使能和有序停止尚未重新执行。
- Docker/current-release 入口仍冻结为旧 CANable 路径，本记录只覆盖 native 启动。
- 未执行 bus-off/restart、压力发送或 24 h 导航负载 HIL；ELECTRI-104 保持未完成。
- 本任务没有发送 NMT/SDO、reset、enable、运动或 PLC 输出。
