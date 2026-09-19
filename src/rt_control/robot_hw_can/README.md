# robot_hw_can

`robot_hw_can` 是 V3 头部云台的原生 SocketCAN 硬件适配器，控制两台 DM-J4310-2EC V1.1。
它不使用 CANopen、CiA402 或 JTC；电机内部位置速度模式 (`CTRL_MODE=2`) 负责 T 型加减速，
ROS 侧只发送最终位置目标和速度上限。

## V3 运行边界

- 逻辑关节固定为 `head_joint` 和 `head_pitch_joint`，对应 V3 Robot Model 的 yaw/pitch。
- 总线固定为 PCIe L2 的 `can2`，当前记录的 CAN ID 是 `1/2`，Master ID 是 `0x11/0x12`。
- 正常位置命令由 `forward_command_controller/ForwardCommandController` 转发；
  `damiao_head_controller/HeadManagerController` 独占 `damiao_head_control/enable_request`
  和 `reset_generation`，提供 `/rt/head/enable`、`/rt/head/disable`、
  `/rt/head/reset_fault`，并负责故障锁存、清错和 `/diagnostics`。
- 位置控制器启动时保持 inactive。只有 manager 先确认两台电机都返回本次使能后的 `ERR=1`，
  才激活位置控制器；位置控制器停止后，manager 才确认两台 `ERR=0`。
- 普通位置帧的发送频率由 `command_rate_hz` 限制，默认设计目标为 250 Hz；使能、失能、清错和
  故障停车帧不受普通位置命令降频影响。

该包不写 CAN 波特率、不改电机模式/ID、不改零位，也不自动保存 flash。宿主必须在启动前把
`can2` 配置为电机要求的 1 Mbps；真实机械零位、方向、关节限位、速度和 CAN 看门狗仍须现场确认。

## ACC/DEC/MAX_SPD

DM-J4310-2EC 官方寄存器为：

| 参数 | 寄存器 | 符号/单位 |
| --- | --- | --- |
| ACC | `0x04` | 正浮点，Krad/s^2 |
| DEC | `0x05` | 负浮点，Krad/s^2 |
| MAX_SPD | `0x06` | 正浮点，rad/s，转子侧 |

可以通过 CAN 设置，不要求使用调试助手。`damiao_parameter_tool` 会读取并确认模式 2，
失能并确认 `ERR=0`，通过 `0x7FF` 写入三个寄存器，等待 `0x55` 回显，再读回校验。默认只
保留在 RAM 中，掉电后失效；只有明确传入 `--save` 时，才发送 `0xAA 0x01` 保存命令并等待回执。

示例只展示命令格式，数值必须由厂家/标定记录确认：

```bash
ros2 run robot_hw_can damiao_parameter_tool \
  --can-interface can2 --motor-id 1 --master-id 17 \
  --acceleration-krad-s2 0.025 \
  --deceleration-krad-s2 -0.05 \
  --maximum-speed-rad-s 20 \
  --confirm WRITE_DAMIAO_PARAMETERS
```

第二台使用 `--motor-id 2 --master-id 18`。只有在失能、急停和机械隔离条件已确认时才可使用
`--save`；flash 擦写有寿命限制，不能放进启动流程。工具异常会尽力再次发送失能，但软件诊断
不能代替 STO、急停和机械限位。

## 配置字段

V3 head YAML 必须包含：

```yaml
can_interface: can2
configure_timeout_ms: 100
feedback_timeout_ms: 50
transition_timeout_ms: 250
command_rate_hz: 250.0
disabled_poll_interval_ms: 20
max_rx_frames_per_cycle: 32
joints:
  - name: head_joint
    can_id: 1
    master_id: 17
    min: TBD
    max: TBD
    velocity_limit: TBD
    acceleration_krad_s2: TBD
    deceleration_krad_s2: TBD
    maximum_speed_rad_s: TBD
  - name: head_pitch_joint
    can_id: 2
    master_id: 18
    min: TBD
    max: TBD
    velocity_limit: TBD
    acceleration_krad_s2: TBD
    deceleration_krad_s2: TBD
    maximum_speed_rad_s: TBD
```

`head_can_config.py` 会拒绝缺失字段、旧关节名、错误符号、重复 ID、超过 1 kHz 的发送频率和未确认
的 `can_interface`。不要把 `TBD` 写入实际运行配置。

## 验证

无硬件包级验证：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_hw_can
colcon test --packages-select robot_hw_can
colcon test-result --verbose
```

V3 head-only Mock 入口：

```bash
ros2 launch rt_control_bringup rt_control_head_runtime.launch.py \
  head_can_config:=/absolute/path/to/head.yaml use_mock_hardware:=true
```

该入口应看到 `damiao_head_manager` active、`head_position_controller` inactive；它不使能任何真实电机。
实机前还必须完成 CAN2 链路、ID/Master ID、ACC/DEC/MAX_SPD 回读、机械零位/方向/限位、低速单轴、
双轴同步、断线、故障锁存和人工清错验收。
