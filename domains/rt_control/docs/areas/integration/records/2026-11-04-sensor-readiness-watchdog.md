---
id: integration-20261104-06
area: integration
title: PLC/真空等传感器异常经看门狗进入 readiness
date: 2026-11-04
type: behavior
trigger: 用户 2026-11-04：PLC/真空异常要进入 readiness；单次通信超时应自行恢复，类似看门狗
commits: [feature/arch-sensor-readiness-watchdog]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [integration-20261104-05]
---

## 改动

- `integ_diagnostics` 订阅 R-OUT-20 `/rt/sensors/state`。某传感器无效、通信错误或过期连续 3 s（`sensors.WATCHDOG_S`，覆盖两次 1 s 的 Modbus 超时加一个 0.2 s 轮询周期），或 3 s 没有任何状态，才在 readiness `errors` 加一条 WARN（DREE 取自传感器状态，停发为 `RT_DEVICE_FEEDBACK_STALE`）；首个正常状态即移除。不加阻断项：`ready` 不变，`status` 降为 DEGRADED。未配置的传感器不计。
- `periph_plc_io` 在 `/rt/sensors/state` 增加 `vacuum_outputs`（DI/DO 模块对泵与两阀的线圈回读）；读取失败为通信错误。

## 验证

本机：看门狗 3 例、节点 1 例（持续异常出现 WARN、恢复后消失）、相关测试 26 项通过；`periph_plc_io` 节点测试在台架主机隔离 overlay 通过。未接真实 PLC。

## 遗留

- 是否在真空异常时阻断 readiness（拆垛依赖真空）待用户决定；当前只降级。
