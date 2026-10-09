# rt_force_torque_broadcaster

ROS 2 Humble 的六维力传感器 ros2_control broadcaster 插件：
`rt_force_torque_broadcaster/ForceTorqueBroadcaster`。
从硬件导出的 state interfaces 获取样本并发布话题，总线访问由已有 EtherCAT
硬件层负责，本包不单独访问设备。

## 当前配置依据：2026-09-10 DOCX V1.1

用户指定 `三代机电气件通讯协议/力传感器/三代机/前置产品信息/六维力传感器-EtherCAT通讯协议V1.1(1).docx`。
左右配置现采用Rx对象0x7000:01..08、Tx对象0x6000:01..09，恢复完整九项state。
映射容器仍是RxPDO 0x1600 / TxPDO 0x1A00；DOCX表中的6000/7000是对象记录地址。
ControlCode保持固定零；七个保留REAL输出采用uint32零字编码，32位零与REAL 0.0一致。

DOCX规定力/力矩100000对应1 N / 1 N·m，参数比例现为0.00001，小数位5。
StatusCode标为保留，不按故障状态解析。Temper也标为保留，注明内部未接PT1000时
为满量程；文档没有定义满量程数值或温度比例，不能用raw/10换算，也不能据raw=0
断言没有接PT1000。文档固件为P140000100-3.6.1.3-ECG，观察实机为3.6.1.0，
版本适用性、安装frame及标定仍待确认，calibration_valid仍为false，不发布有效wrench。

下方实测为历史raw通信证据；本次仅更新配置并完成离线测试，不声称新比例已经实机标定。

## 三代机双蓝点传感器：实际话题通信已验证

2026-10-07，双 P140000100 力传感器在六从站台架上完成真实 EtherCAT DC/1ms
采集与另一 ROS 进程订阅验证。左右原始数据、九路命名状态均已实测成功。
这里确认的是通信和原始数据读取；工程单位、物理坐标系、标定和长期运行仍待确认。

| 配置项 | 左侧 | 右侧 |
| --- | --- | --- |
| 绝对总线位置 | 2 | 4 |
| sensor_name | left_wrist_force_sensor | right_wrist_force_sensor |
| controller_name | left_wrist_force_sensor_broadcaster | right_wrist_force_sensor_broadcaster |
| profile | bluepoint_p140000100_left | bluepoint_p140000100_right |
| raw_topic | /rt_control/left_wrist/raw | /rt_control/right_wrist/raw |
| sensor_state_topic | /rt_control/left_wrist/state | /rt_control/right_wrist/state |
| wrench_topic | /rt_control/left_wrist/wrench | /rt_control/right_wrist/wrench |

配置来源：

- [双传感器 YAML](../rt_control_bringup/config/bluepoint_force_sensors.draft.yaml)
- [左从站 PDO 配置](../robot_hw_ethercat/config/slaves/bluepoint_p140000100_left.yaml)
- [右从站 PDO 配置](../robot_hw_ethercat/config/slaves/bluepoint_p140000100_right.yaml)
- [台架启动入口](../rt_control_bringup/launch/rt_control_force_sensors.launch.py)

设备身份为 Vendor `0xA1`、Product `0x8081`、Revision `1`。
RxPDO `0x1600` 映射 `0x7000:01..08`，TxPDO `0x1A00` 映射 `0x6000:01..09`。
台架入口只配置两个传感器模块，使用固定零输出，不加载电机控制模块。
换接线或使用完整机器人拓扑时，必须重新确认位置与配置。

## 构建与启动

从 `robot_driver` 仓库根目录执行：

```bash
bash tools/build_force_sensor_bench.sh
source /opt/ros/humble/setup.bash
source /tmp/rt-control-force-bench/install/setup.bash
export ROS_DOMAIN_ID=12
```

`/tmp/rt-control-force-bench/install/setup.bash` 是隔离构建生成的 ROS 环境文件，
用于让当前终端找到已构建的包和插件。它不是传感器数据文件；修改源码后需要重建。
默认构建目录位于 `/tmp`，若目录被清理，重新运行构建脚本。

已确认接线且 master0 空闲时，启动真实通信：

```bash
ros2 launch rt_control_bringup rt_control_force_sensors.launch.py \
  use_mock_hardware:=false sync_mode:=dc
```

启动前会检查主站、链路、六从站数量以及位置 2/4 的身份与 PDO。
该入口默认使用 Mock，必须显式设置 `use_mock_hardware:=false` 才读取实机。
不能与另一个占用 master0 的主控进程同时运行；已有采集运行时直接订阅即可。
在启动终端按 Ctrl-C 正常停止。

## 按 YAML 查看九路实际数据

另开终端，加载同一环境并使用与采集进程相同的 ROS 域。本次实测使用域 12：

```bash
source /opt/ros/humble/setup.bash
source /tmp/rt-control-force-bench/install/setup.bash
export ROS_DOMAIN_ID=12

# 各读取一条消息；去掉 --once 可连续查看
ros2 topic echo /rt_control/left_wrist/state --once
ros2 topic echo /rt_control/right_wrist/state --once

# 检查消息类型、发布者和接收频率
ros2 topic info /rt_control/left_wrist/state -v
ros2 topic hz /rt_control/left_wrist/state
```

`sensor_state_topic` 为可选参数，消息类型是
`control_msgs/msg/DynamicJointState`。消息与 YAML 对应如下：

| 消息字段 | YAML / 含义 |
| --- | --- |
| header.stamp | 控制器更新时的 ROS 时间戳 |
| header.frame_id | 台架原始数据局部标签，不代表已确认的安装坐标系 |
| joint_names[0] | sensor_name |
| interface_values[0].interface_names | value_interfaces 后接 auxiliary_interfaces，保留完整名称 |
| interface_values[0].values | 与上述名称逐项对应的实际原始值 |

左侧九路顺序如下，右侧将名称前缀替换为 `right_wrist_force_sensor`：

| 数组下标 | 完整接口名称 |
| --- | --- |
| 0 | left_wrist_force_sensor/channel_1_raw |
| 1 | left_wrist_force_sensor/channel_2_raw |
| 2 | left_wrist_force_sensor/channel_3_raw |
| 3 | left_wrist_force_sensor/channel_4_raw |
| 4 | left_wrist_force_sensor/channel_5_raw |
| 5 | left_wrist_force_sensor/channel_6_raw |
| 6 | left_wrist_force_sensor/status_code_raw |
| 7 | left_wrist_force_sensor/sample_counter_raw |
| 8 | left_wrist_force_sensor/temperature_raw |

`values` 使用 double 传递，32 位有符号通道和无符号状态/计数均能精确表示。
解析时按名称匹配即可，不需要把 YAML 中的接口名称当作独立 ROS 话题。

原六路话题仍可使用，类型为 `std_msgs/msg/Int32MultiArray`：

```bash
ros2 topic echo /rt_control/left_wrist/raw --once
ros2 topic echo /rt_control/right_wrist/raw --once
```

`raw.data[0..5]` 对应 `channel_1_raw..channel_6_raw`。
`layout.dim: []` 表示没有声明数组维度标签，`data_offset: 0` 表示有效数据从首项开始；
该话题不包含状态、计数或温度，查看这些数据使用 `/state`。

## 实测结果

九路命名话题独立订阅 3 秒：左侧收到 **2635** 条，右侧收到 **2634** 条。
每条消息的传感器名、接口名称、九项长度均与 YAML 一致；左右分别观察到
1323、1320 个不同采样计数，证明该短窗口内数据持续更新。
以下为当次末条样本，重新读取时数值会变化：

| 接口尾名 | 左侧 | 右侧 |
| --- | ---: | ---: |
| channel_1_raw | -137358183 | 67240 |
| channel_2_raw | 118931187 | 231762 |
| channel_3_raw | 173395650 | 70211 |
| channel_4_raw | 3435708 | -931 |
| channel_5_raw | -2164927 | 245 |
| channel_6_raw | 41045982 | 1885 |
| status_code_raw | 0 | 0 |
| sample_counter_raw | 3290120 | 3290106 |
| temperature_raw | 0 | 0 |

此前六路 `/raw` 独立订阅 5 秒收到左 4956、右 4955 条；正常停机后主站回到
Idle，所有从站回到 PREOP。九路版本构建成功，13 项 C++ 测试、6 项参数转换
测试通过；质量门禁 207 项通过、13 项跳过。

当前 `calibration_valid: false`，因此不发布有效 `/wrench`。
新DOCX提供了比例，但其固件适用性和实机标定仍待确认，当前发布值仍是raw。
温度字段未定义可用换算；状态码为保留字段，0不能解释为已确认的无故障状态。
短时计数变化不等于已完成闭环新鲜度、长期 DC 稳定性或整机运行验收。

详细证据见[实机通信记录](../../../domains/rt_control/docs/areas/ecat-axes/records/2026-10-07-bluepoint-raw-bench-topics.md)。


## DOCX 配置复测结果（2026-10-07）

新配置9包构建成功，真实DC/1ms、ROS域12启动，独立订阅5秒左右state各5005条，raw左5004/右5005条；接口名称/九项长度正确，计数与时间戳无倒退，wrench无消息。计数各变化2502次，约500次/秒，不把1kHz话题发布当作1kHz独立新样本。状态/温度保留字段均0，不解释为0℃或设备无故障。

左末条raw六项[-137359099,118953426,173473178,3436116,-2164148,41050758]；右[66141,243502,85604,-1482,268,2832]。按DOCX比例暂算左Mz=410.50758Nm，超过选型资料120Nm，故通信成功但左侧物理值/零点/量程/版本适用性仍需核查。未清零、未标定、未发布有效wrench。测试后正常停止，主站Idle、全部从站PREOP。
