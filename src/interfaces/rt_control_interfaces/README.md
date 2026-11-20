# RT-Control private interfaces

This package is owned and consumed only by RT-Control: `PlcIoState` and `EmergencyStopState`. Other domains use the public
`robot_interfaces` packages instead.

## 急停输入观测

`EmergencyStopState` 为 RT-Control 域内输入广播，话题 `/plc/emergency_stop/state`；
语义、消费者失联/锁存要求见 [periph_emergency_stop](../../rt_control/periph_emergency_stop/README.md)。
不是公共跨域契约，也不是物理停止确认。
