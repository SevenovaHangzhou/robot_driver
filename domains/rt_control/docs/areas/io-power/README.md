# io-power — PLC IO、真空执行与 BMS

**范围**：PLC Modbus TCP 读写与寄存器映射、三路输出（左右电磁阀、真空泵）、
真空建立反馈、BMS CAN 帧解析与电池状态发布、Modbus LED IO 与四路超声波测距。
**Owner 包/资产**：`src/rt_control/plc_node`、`src/rt_control/plc_io_modbus`、
`src/rt_control/bms_node`、`src/rt_control/modbus_tcp_rtu485`。

不属于本区：`/vacuum/grip` 等公共契约适配（→ contract）、CAN 接口宿主命名与
systemd unit（→ realtime-host）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | 输出 bit0=右阀、bit1=左阀、bit2=共用泵 | [io-power-20260817-01](records/2026-08-17-correct-solenoid-side-mapping.md)#F1 | 有效 |
| 01#F2 | 本次未重新确认左右真空输入 bit，保持现状但仍需闭环复核 | [io-power-20260817-01](records/2026-08-17-correct-solenoid-side-mapping.md)#F2 | 有效 |
| 02#F1 | 当前新工控机访问 `192.168.1.88:502` 的 PLC socket 固定绑定 `eno1`；旧 `enp4s0` 不再是活动配置。 | [io-power-20260904-01](records/2026-09-04-plc-interface-eno1.md)#F1 | PARTIAL（配置/测试通过；运行进程待授权重启） |
| 03#F1 | 分立数字/模拟量模块通过既有 `/plc/*` 私有接口接入公共真空适配器；输出读回与模拟量吸附判定分离。 | [io-power-20260914-01](records/2026-09-14-discrete-analog-vacuum-bridge.md)#F1 | PARTIAL（源码/离线验证；待实机闭环） |
| 04#F1 | LED 颜色话题仅为域内工程输入，外部消费者需先审查公共契约。 | [io-power-20260916-01](records/2026-09-16-modbus-led-driver.md)#F1 | 有效 |
| 04#F2 | LED 写入失败不自动重试，退出不发送关灯/复位命令，可能保持最后颜色。 | [io-power-20260916-01](records/2026-09-16-modbus-led-driver.md)#F2 | PARTIAL（退出源码/离线；硬件保持待验） |
| 04#F3 | LED YAML 是用户配置而非现场验证证据，设备身份/地址/寄存器映射待验。 | [io-power-20260916-01](records/2026-09-16-modbus-led-driver.md)#F3 | PARTIAL |
| 05#F1 | `modbus_tcp_rtu485` 同包拥有独立 LED 写节点与 E08 超声波只读节点，网络 IO 均不进入 ros2_control 实时环。 | [io-power-20260917-01](records/2026-09-17-modbus-ultrasonic-driver.md)#F1 | 有效 |
| 05#F2 | E08 已在 `192.168.1.12:504`、unit 1 通过 FC03 `0x0106..0x0109` 实测，并成功发布四路 ROS 数据。 | [io-power-20260917-01](records/2026-09-17-modbus-ultrasonic-driver.md)#F2 | PARTIAL（T2 只读；长期稳定性待验） |
| 05#F3 | 默认 frame ID 仅表示 E08 通道，四个物理安装位姿及 Robot Model TF 尚未冻结，不得推导方向语义。 | [io-power-20260917-01](records/2026-09-17-modbus-ultrasonic-driver.md)#F3 | PARTIAL |
| 06#F1 | 四路话题改用 `sensor_msgs/Range`，固定 3.5 m、60 度元数据并使用同一批次完成时间。 | [io-power-20260919-01](records/2026-09-19-ultrasonic-standard-range.md)#F1 | PARTIAL（离线通过；标准消息实机待验） |
| 06#F2 | `0xFFFD` 无目标发布 `+Inf`，诊断为 `OK/no_target`。 | [io-power-20260919-01](records/2026-09-19-ultrasonic-standard-range.md)#F2 | PARTIAL（离线通过；实机待验） |
| 06#F3 | 默认四个 frame 仅表示 E08 通道，装车后必须用实测外参与 `base_link` TF 替换。 | [io-power-20260919-01](records/2026-09-19-ultrasonic-standard-range.md)#F3 | PARTIAL |
| 06#F4 | E084F 一次 FC03 连读四个通道，按已确认配置同时测量；驱动不做逐路发射轮询。 | [io-power-20260919-01](records/2026-09-19-ultrasonic-standard-range.md)#F4 | PARTIAL（源码/离线；实机时序待验） |
| 06#F5 | V3 安装独立 `rt_control_ultrasonic.launch.py`，双臂 runtime 不隐式启动采集，Mock 禁用硬件进程。 | [io-power-20260919-01](records/2026-09-19-ultrasonic-standard-range.md)#F5 | PARTIAL（离线/安装后 Mock 通过；实机待验） |
| 07#F1 | LED 驱动配置为六个控制器，共享 TCP 502，RTU 站号依次为 1..6。 | [io-power-20260920-01](records/2026-09-20-six-led-controllers.md)#F1 | PARTIAL（用户确认配置；实机未验证） |
| 07#F2 | 六路域内颜色话题为 `led0/color`..`led5/color`，每路保持既有 RGBW/FC16 语义。 | [io-power-20260920-01](records/2026-09-20-six-led-controllers.md)#F2 | PASS（源码与离线构建） |

## 记录索引（倒序）

- 2026-09-20 [Modbus LED 控制器由四路扩展为六路](records/2026-09-20-six-led-controllers.md) — feature，PARTIAL（离线构建通过；实机未验证）
- 2026-09-19 [四路超声波切换为标准 Range 接口](records/2026-09-19-ultrasonic-standard-range.md) — feature，PARTIAL（T1；标准消息/参数离线通过，实机与 TF 待验）
- 2026-09-17 [通用 Modbus RTU485 包与四路超声波驱动](records/2026-09-17-modbus-ultrasonic-driver.md) — feature，PARTIAL（T2 只读通信/ROS 发布通过；TF 与长期运行待验）
- 2026-09-16 [四路 Modbus LED 驱动](records/2026-09-16-modbus-led-driver.md) — feature，PARTIAL（T1 离线；身份/寄存器/实机写入与退出保持待验）
- 2026-09-14 [分立 IO 模块真空桥接](records/2026-09-14-discrete-analog-vacuum-bridge.md) — feature，PARTIAL（阈值 -80 kPa；实机吸附/释放闭环待验证）
- 2026-09-04 [PLC 绑定接口切换为 eno1](records/2026-09-04-plc-interface-eno1.md) — fix，PARTIAL（未重启当前运行进程）
- 2026-08-17 [纠正左右电磁阀输出映射](records/2026-08-17-correct-solenoid-side-mapping.md) — corrective，PASS（T4；输入映射与新镜像待验证）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-RT-IO-001..005、T-020 及其 sensor-frame 后续
- `docs/plc-bms-integration.md`、`docs/plc-bms-commissioning-20260728.md`、
  `docs/plc-bms-merge-hardware-handoff.md`
- 相关 BQ（不完全）：BQ-121（OPEN/DEPLOYMENT：BMS CANable 缺席阻塞 CAN unit）
