---
id: integration-20261104-05
area: integration
title: 真空接口切换到契约 cb379db 的 /rt/vacuum 名称与类型
date: 2026-11-04
type: behavior
trigger: 用户 2026-11-04 给定真空接口对照表；契约 R-IN-04、R-IN-05L/R、R-OUT-05L/R
commits: [feature/arch-vacuum-endpoints]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [integration-20261104-01]
---

## 改动

- 服务：`/plc/vacuum_pump`、`/plc/vacuum_valve/{left,right}`（私有 `SetDigitalOutput`）改为 `/rt/vacuum/pump/set_enabled`、`/rt/vacuum/{left,right}_valve/set_enabled`（公共 `SetEnabled`）。`accepted` 只在线圈回读等于请求值时为 true，`enabled` 为回读值；未配置 `NOT_READY`，通信失败或写后状态未知 `RT_PLC_UNAVAILABLE`。
- 契约 R-IN-04 的停泵条件落实：左右阀回读为释放、左右压力有效且 ≥ -1 kPa，否则拒绝并返回 `RT_POSSIBLE_LOAD_HELD`，不写线圈。
- 压力：`/plc_io/vacuum/{left,right}`（私有 `VacuumSensorState`）改为 `/rt/vacuum/{left,right}/pressure`（`sensor_msgs/FluidPressure`，表压 Pa，负值为真空），无效样本不发布；两路传感器的有效、无效、通信错误、未配置写入 `/rt/sensors/state`。
- 删除私有 `SetDigitalOutput`、`VacuumSensorState`；网页压力显示改订阅新话题。

## 验证

`periph_plc_io` 真实节点对进程内 Modbus TCP 模拟器的测试（停泵互锁、表压换算、无效样本）在台架主机隔离 overlay、域 91 上运行通过（本机跨进程 DDS 发现不可用）；网页 250 项、`tools/tests` 191 项通过。未连接真实 PLC。

## 遗留

- 真空硬件仍 `vacuum_system.configured: false`，地址与传感器身份待现场确认后才可启用。
