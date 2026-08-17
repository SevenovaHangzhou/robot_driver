# PLC / BMS 同容器集成说明

更新时间：2026-07-28

## 结论

PLC 与 BMS 已并入现有 `rt-control` 启动域，不再创建 `rt-io` 容器或独立启动脚本。生产 Compose 仍只有一个
`rt-control` 服务，仍以安装后的 `rt_control_start` 作为容器 PID 1；PLC、BMS 与现有控制进程暂时共用同一
`cpuset`。这是为近期联调接受的临时方案，不代表完成了 WCET、调度干扰或生产实时性验收。

`can_bus_guard` 整包不进入本实现。BMS 是 SocketCAN 被动读取者，不负责创建、重命名、配置或独占 `can1`。

## BMS 接口

节点只接收标准数据帧 `0x3FC`：

| 数据 | 字节 | 换算 |
| --- | --- | --- |
| 总电压 | byte 0-1，大端无符号数 | 原始值 × 0.1 V |
| SOC | byte 4 | 百分数 ÷ 100，ROS 值域为 0.0-1.0 |

唯一发布接口为：

| 话题 | 类型 | 周期 | 有效字段 |
| --- | --- | --- | --- |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 5 s | `voltage`、`percentage` |

未使用的浮点字段发布 `NaN`，不再发布电流、容量、告警、中文 JSON 或独立 SOC 话题。超过 3 秒未收到有效
`0x3FC` 时，`voltage` 和 `percentage` 均为 `NaN`，`present=false`，不会继续伪装旧值为新数据。

宿主必须预先保证 `can1` 已命名、UP 且为 500 kbit/s。容器只有被动读取所需的 `NET_RAW`，没有
`NET_ADMIN`，因此不会在容器内修改 CAN 位率或链路状态。

## PLC 接口与映射

PLC 使用 Modbus TCP，默认 `192.168.1.88:502`、unit id `1`，绑定宿主网口 `enp4s0`。

| 寄存器 / 位 | 语义 | 1 的含义 |
| --- | --- | --- |
| `%MW200 bit0` / `QX0.0` | 右臂电磁阀命令 | 开 |
| `%MW200 bit1` / `QX0.1` | 左臂电磁阀命令 | 开 |
| `%MW200 bit2` / `QX0.2` | 共用真空泵命令 | 开 |
| `%MW201 bit0` | PLC 远程控制允许 | 允许 |
| `%MW210 bit0` / `IX0.6` | 左臂真空状态 | 真空已建立 |
| `%MW210 bit1` / `IX0.7` | 右臂真空状态 | 真空已建立 |
| `%MW211 bit0-bit2` | 三路实际输出状态 | 对应输出已开 |
| `%MW212` | IO 报警字 | 原样发布 |

2026-08-17 现场逐点测试纠正了原左右输出假设：bit0=右阀、bit1=左阀。该测试只确认
`%MW200/%MW211` 输出侧，未重新确认 `%MW210 bit0/bit1` 的左右真空输入身份；输入定义暂时
保持不变，正式闭环验收仍需按实际左右真空建立状态复核。

节点连接和重连后检查 `%MW201 bit0`；若为 0，则写回 `原值 | 0x0001` 并读回确认。回读全字必须与目标值
一致，因此非目标 bit 的意外变化也会使修复失败。轮询时持续检查该位；修复失败时仍尽可能发布只读 PLC 状态，
但三个写服务会返回失败。

写入口只保留以下三个 `std_srvs/srv/SetBool` 服务：

- `/plc/left_solenoid`
- `/plc/right_solenoid`
- `/plc/vacuum_pump`

已删除批量 `/plc/command` 和三个 Bool 命令话题，避免多发布者覆盖。每次服务调用都先读取 `%MW200`，只修改
目标 bit 并保留其余 15 bit；然后在最多 1 秒内同时等待 `%MW200` 命令回读和 `%MW211` 实际状态的目标 bit
一致。超时返回 `success=false`，不做补偿写或自动反向动作。启动、断连、重连和退出都不会把现有输出强制清零。

唯一 PLC 状态话题为 `/plc/io_state`，类型 `rt_control_interfaces/msg/PlcIoState`，周期 0.5 秒。消息包含连接与
新鲜度、左右真空建立、左右电磁阀、真空泵、`io_alarm` 和错误文本。断连或数据超过 1.5 秒时
`data_fresh=false`；消费者必须同时检查 `connected` 与 `data_fresh`。

## 启动与容器边界

硬件参数的唯一仓库配置源是
[`rt_io.yaml`](../../../src/rt_control/rt_control_bringup/config/rt_io.yaml)。普通源码 launch 默认关闭两个 IO
节点，避免 Mock 或开发命令意外访问真实 PLC/CAN：

```bash
ros2 launch rt_control_bringup rt_control.launch.py \
  use_mock_hardware:=true \
  start_plc:=false \
  start_bms:=false
```

需要明确直连硬件时才传入：

```bash
ros2 launch rt_control_bringup rt_control.launch.py \
  start_plc:=true \
  start_bms:=true
```

生产 [`compose.yaml`](../../../docker/compose.yaml) 在同一个 `rt-control` 服务中设置
`RT_CONTROL_START_PLC=true` 和 `RT_CONTROL_START_BMS=true`。它没有覆盖 `command` 或 `entrypoint`，所以原有
有序失能与信号转发链保持不变。

PLC 与 BMS 节点各自处理普通 Modbus/CAN 断线并按配置重连，launch 不再对进程本身使用无条件 respawn。原因是
ROS context 关停与 respawn 存在竞态：节点退出后可能在容器关停期间被重新拉起，导致 PID 1 无法结束。节点现在把
`ExternalShutdownException` 作为正常退出并释放 socket；若 Python 进程意外崩溃，则保留错误证据并有序重启整个
rt-control，不做容器内部的局部进程复活。

首次候选提交 `e4fed685bfa4485c210ad038c804a331b4801d88`（镜像
`sha256:998c5a1e9e5f70f7e09f1f2a8c316bbc3714bd9815e68f6b870130a332542c06`）已通过功能读取，但因上述关停竞态在
100 秒后被 Docker 强制终止为 137，现已拒绝。

关停修复候选提交为 `d415c0c2c75917a9545a4a2f87487718de8622a2`，镜像 ID 为
`sha256:01bd550b068fccb9158b007067e55c30eed7d7d7253ef9179dfdf6d9be9a11c2`。该镜像已在开启 PLC/BMS、使用 Mock
硬件且 controller manager ready 的条件下 2 秒内 exit 0，无 traceback、respawn 或 `UNCLEAN_SHUTDOWN`。目标机
随后完成只读重复停止、三路输出逐点 ON/OFF 和完整一键 start→READY→stop，全部 exit 0；详见
[PLC / BMS 与一键启动实机验收](plc-bms-commissioning-20260728.md)。

## 联调检查

```bash
ros2 topic echo /battery_state
ros2 topic echo /plc/io_state
ros2 service call /plc/left_solenoid std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/right_solenoid std_srvs/srv/SetBool "{data: false}"
ros2 service call /plc/vacuum_pump std_srvs/srv/SetBool "{data: true}"
```

协议解析、位保留、命令/实际位回读、干净镜像、目标机 PLC/CAN 读取和三路输出均已验证。独立 BMS HMI 目视对照、
左右实体身份、共用泵气路效果和真空传感器建立状态仍需 PLC / 电气人员在正式工艺联调中复核；不要把寄存器验证
扩写为尚未观察的机械/气路验收。
