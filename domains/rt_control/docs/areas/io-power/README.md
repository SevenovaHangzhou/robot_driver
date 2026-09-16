# io-power — PLC IO、真空执行与 BMS

**范围**：PLC Modbus TCP 读写与寄存器映射、三路输出（左右电磁阀、真空泵）、
真空建立反馈、BMS CAN 帧解析与电池状态发布。
**Owner 包/资产**：`src/rt_control/plc_node`、`src/rt_control/plc_io_modbus`、
`src/rt_control/bms_node`。

不属于本区：`/vacuum/grip` 等公共契约适配（→ contract）、CAN 接口宿主命名与
systemd unit（→ realtime-host）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | 输出 bit0=右阀、bit1=左阀、bit2=共用泵 | [io-power-20260817-01](records/2026-08-17-correct-solenoid-side-mapping.md)#F1 | 有效 |
| 01#F2 | 本次未重新确认左右真空输入 bit，保持现状但仍需闭环复核 | [io-power-20260817-01](records/2026-08-17-correct-solenoid-side-mapping.md)#F2 | 有效 |
| 02#F1 | 当前新工控机访问 `192.168.1.88:502` 的 PLC socket 固定绑定 `eno1`；旧 `enp4s0` 不再是活动配置。 | [io-power-20260904-01](records/2026-09-04-plc-interface-eno1.md)#F1 | PARTIAL（配置/测试通过；运行进程待授权重启） |
| 03#F1 | 分立数字/模拟量模块通过既有 `/plc/*` 私有接口接入公共真空适配器；输出读回与模拟量吸附判定分离。 | [io-power-20260914-01](records/2026-09-14-discrete-analog-vacuum-bridge.md)#F1 | PARTIAL（源码/离线验证；待实机闭环） |

## 记录索引（倒序）

- 2026-09-14 [分立 IO 模块真空桥接](records/2026-09-14-discrete-analog-vacuum-bridge.md) — feature，PARTIAL（阈值 -80 kPa；实机吸附/释放闭环待验证）
- 2026-09-04 [PLC 绑定接口切换为 eno1](records/2026-09-04-plc-interface-eno1.md) — fix，PARTIAL（未重启当前运行进程）
- 2026-08-17 [纠正左右电磁阀输出映射](records/2026-08-17-correct-solenoid-side-mapping.md) — corrective，PASS（T4；输入映射与新镜像待验证）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-RT-IO-001..005、T-020 及其 sensor-frame 后续
- `docs/plc-bms-integration.md`、`docs/plc-bms-commissioning-20260728.md`、
  `docs/plc-bms-merge-hardware-handoff.md`
- 相关 BQ（不完全）：BQ-121（OPEN/DEPLOYMENT：BMS CANable 缺席阻塞 CAN unit）
