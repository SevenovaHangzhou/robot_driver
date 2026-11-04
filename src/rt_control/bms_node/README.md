# bms_node

金凤凰 V1.1 双电池监控节点。节点通过 SocketCAN 以 250 kbit/s 查询地址 `0x01` 和
`0x02`，不配置 CAN 接口，也不发送 `D9/DA`，因此不会控制 BMS 的充放电 MOS。

## USB-CAN 与 SocketCAN

电池使用专用 `gs_usb` USB-CAN，固定为 Linux SocketCAN 接口 `can1`。当前批准的适配器
USB 序列号为 `004D00675230500720333159`；`bms_node` 启动时会核对该序列号，防止误用其他
CAN 总线。CAN 参数为 250 kbit/s、发送队列长度 128。

首次安装或 USB-CAN 插拔后，使用仓库脚本按序列号固定接口并配置 SocketCAN：

```bash
cd /home/sev_v3/Robot/rt-control/v3/robot_driver
sudo ./hostsetup/can-install.sh --start
```

也可以在已经确认目标适配器就是 `can1` 时手动配置：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 250000
sudo ip link set can1 txqueuelen 128
sudo ip link set can1 up
```

启动 BMS 前检查接口：

```bash
udevadm info -q property -p /sys/class/net/can1 | grep '^ID_SERIAL_SHORT='
ip -details -statistics link show can1
```

预期序列号为 `004D00675230500720333159`，接口为 `UP`、CAN 状态为 `ERROR-ACTIVE`、
波特率为 `250000`。然后启动节点：

```bash
source /opt/ros/humble/setup.bash
source /home/sev_v3/Robot/rt-control/v3/robot_driver/install/setup.bash
ros2 run bms_node bms_node --ros-args \
  --params-file /home/sev_v3/Robot/rt-control/v3/robot_driver/src/rt_control/bms_node/config/bms_node.yaml
```

在另一个终端检查两块电池是否返回状态：

```bash
source /opt/ros/humble/setup.bash
source /home/sev_v3/Robot/rt-control/v3/robot_driver/install/setup.bash
ros2 topic echo /bms/primary/status_chinese --once
ros2 topic echo /bms/secondary/status_chinese --once
```

如需观察原始扩展 CAN 帧，可并行运行 `candump can1`。状态话题持续显示“数据缺失或超时”
时，检查电池供电、CAN_H/CAN_L、终端电阻以及 BMS 地址 `0x01`、`0x02`。

节点从专用 `can1` 原始 SocketCAN socket 接收报文，并在用户态只接受配置地址返回的
`0x90/0x91/0x92/0x93/0x94/0x95/0x98` 扩展数据帧；RTR、错误帧、非 8 字节帧以及其他
CAN ID 均会被丢弃。

## 电池开关

`bms_node` 不提供开关服务，也不创建 PLC 客户端。调用方直接使用原有
`plc_io_modbus` 节点提供的两个 `rt_control_interfaces/srv/SetDigitalOutput` 服务：

- `/plc/battery/primary`：主电池继电器；
- `/plc/battery/secondary`：副电池继电器。

请求中的 `enabled=true` 表示闭合，`enabled=false` 表示断开。只有 PLC 写入成功并且线圈
回读等于请求值时，服务才返回 `OUTCOME_CONFIRMED`。

两路继电器配置位于 `plc_io_modbus/config/plc_io_modbus.yaml` 中，使用同一个
`plc_io_modbus.digital.module` host、port 和 unit_id，但必须配置不同的 `do_address`。
当前主、副电池 DO 地址分别为 `0`、`1`，且 `battery_relays.configured=true`。修改地址后
需要重启 `plc_io_modbus`。两个节点需要在两个终端中分别启动，并分别加载各自的配置文件。

终端 1：启动 PLC 继电器控制节点：

```bash
source /opt/ros/humble/setup.bash
source /home/sev_v3/Robot/rt-control/v3/robot_driver/install/setup.bash

ros2 run plc_io_modbus plc_io_modbus --ros-args \
  --params-file /home/sev_v3/Robot/rt-control/v3/robot_driver/src/rt_control/plc_io_modbus/config/plc_io_modbus.yaml
```

终端 2：启动双电池 CAN 状态节点：

```bash
source /opt/ros/humble/setup.bash
source /home/sev_v3/Robot/rt-control/v3/robot_driver/install/setup.bash

ros2 run bms_node bms_node --ros-args \
  --params-file /home/sev_v3/Robot/rt-control/v3/robot_driver/src/rt_control/bms_node/config/bms_node.yaml
```

直接控制示例：

```bash
ros2 service call /plc/battery/primary \
  rt_control_interfaces/srv/SetDigitalOutput "{enabled: true}"
ros2 service call /plc/battery/secondary \
  rt_control_interfaces/srv/SetDigitalOutput "{enabled: false}"
```

## 状态话题

每块电池同时发布标准数值消息和中文文本：

| 电池 | 标准 `sensor_msgs/BatteryState` | 中文状态 | 中文故障 |
| --- | --- | --- | --- |
| 主电池 | `/battery_state` | `/bms/primary/status_chinese` | `/bms/primary/fault_chinese` |
| 副电池 | `/battery_state/secondary` | `/bms/secondary/status_chinese` | `/bms/secondary/fault_chinese` |

中文状态包含电量、电压、电流、最高/最低单体温度、MOS温度、板卡温度、温差、充放电状态、
充电 MOS 状态和放电 MOS 状态。中文故障逐位展开 `0x98` 中协议规定的所有告警/故障，
并附加 Byte7 故障码；未收到或过期会明确显示“未收到故障状态”或“故障状态通信超时”。

协议没有规定多字节端序和 MOS 状态原始值。`multi_byte_order=auto` 只在 SOC 能唯一判断
端序时锁定。`mos_closed_raw`/`mos_open_raw` 默认均为 `-1`，此时文本显示原始值但不猜测
开闭；厂家或抓包确认后必须同时配置为两个不同的字节值。

电池话题、周期、超时、CAN 地址和 MOS 映射位于 `config/bms_node.yaml`。PLC 数字量模块及
两路继电器 DO 地址位于 `../plc_io_modbus/config/plc_io_modbus.yaml`。

## 构建和测试

在 `robot_driver` 工作区根目录执行。先加载 ROS 2 Humble，再加载 V3 已安装的接口依赖：

```bash
cd /home/sev_v3/Robot/rt-control/v3/robot_driver

source /opt/ros/humble/setup.bash
source /home/sev_v3/Robot/rt-control/v3/install/setup.bash

colcon build --symlink-install --packages-up-to bms_node plc_io_modbus
colcon test --packages-select bms_node plc_io_modbus
colcon test-result --verbose
```

如果未加载 `/home/sev_v3/Robot/rt-control/v3/install/setup.bash`，构建接口包时可能提示找不到
`robot_rt_control_interfaces`。

编译完成后，启动节点前加载本工作区的安装环境：

```bash
source /home/sev_v3/Robot/rt-control/v3/robot_driver/install/setup.bash
```
