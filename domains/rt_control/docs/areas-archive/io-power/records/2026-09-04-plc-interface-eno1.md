---
id: io-power-20260904-01
area: io-power
title: PLC 绑定接口切换为 eno1
date: 2026-09-04
type: fix
trigger: ELECTRI-115
commits: [fix/electri-115-plc-interface]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - ELECTRI-115
supersedes: []
related: [realtime-host-20260904-01]
---

## 背景

新工控机 Native 运行时，PLC 节点持续报告 `timeout` 与 `No route to host`。现场核对确认
`rt_io.yaml` 仍绑定旧接口 `enp4s0`，而到 PLC `192.168.1.88:502` 的当前批准出口为
`eno1`。现场只修改源码配置 symlink 后未重启进程，因此该诊断是下一次启动所需配置事实，
不是已完成的 PLC 通信验收。

## 改动

- `src/rt_control/rt_control_bringup/config/rt_io.yaml` 的 PLC `interface` 从 `enp4s0`
  精确更新为 `eno1`。
- 更新中央硬件配置合同测试与当前 PLC 集成说明。
- 未修改 host/port/unit id、轮询/重连、寄存器、输出映射、PLC node 默认实现或日志策略。

## 验证

- TDD RED：新接口合同在旧 main 上为 1 failed，实际值为 `enp4s0`、期望 `eno1`。
- GREEN：focused contract 1 passed；完整 `tools/tests/test_rt_io_integration.py` 为
  18 passed；YAML 解析与 `git diff --check` 通过。
- `rt_control_bringup` package-select 构建通过，安装后的 `share/rt_control_bringup/config/rt_io.yaml`
  复读为 `eno1`；`tools/quality_gate.sh` 为 214 passed、门禁覆盖率 83%。
- 现场旧 release 的同一配置差异已通过 YAML 解析；Native PID 149664 与 PLC PID 149720
  按用户要求保持运行/使能，没有热加载、重启或发送 Modbus/ROS/SDO。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前新工控机 PLC socket 的活动绑定接口为 `eno1`；`enp4s0` 只保留在旧主机历史证据中。
- F2: `rt_io.yaml` 仍是运行时硬件参数的唯一仓库配置源，本次不增加环境变量或第二份网卡映射。

## 遗留

- 当前 PLC 进程不会热加载 YAML。后续重启可能执行既有 `%MW201 bit0` 远程控制允许修复写入，
  必须另行取得 PLC 写入和进程重启授权后再验证连通性。
- 本轮不运行 Docker、不重启或替换当前 Native 会话，不声明 PLC 实机通信已恢复。
