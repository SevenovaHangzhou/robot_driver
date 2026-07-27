# rt-control 一键启动

> 仅适用于已经部署好的 `ar@192.168.0.40`。脚本会访问真实 EtherCAT/CANopen，
> 并自动调用 `/rt/enable`；不能在开发电脑、其他工控机或无现场授权时运行。

> **交付前置：** 脚本已锁定候选镜像 `e4fed685…`，但该候选尚未在目标机完成受控
> “启动→READY→stop”首跑。rt-control 负责人完成本页要求的分阶段实机验收前，联调同事不得自行启动。

> **T-020 TF 发布边界：** 候选镜像已包含新 TF 根链，并已通过无设备 Mock；目标机只读复核尚未完成，
> 因而暂不能把候选镜像结果写成目标机已上线。

> **PLC/BMS 发布边界：** 候选镜像与 launcher 锁已完成本机构建和静态验收；一键启动会额外要求 `can1`
> 的批准序列号、UP/500 kbit/s 和 `0x3FC` 帧，并等待 `/plc/io_state`、`/bms/battery_state` 有效后再自动使能。
> PLC/BMS 实机验证尚未完成，接口细节见 [PLC / BMS 同容器集成](plc-bms-integration.md)。

## 启动

在这份仓库的根目录执行一条命令：

```bash
./tools/rt_control_ipc.sh
```

脚本检查通过后会显示现场确认文字。确认实体急停可用、机器人和履带周围无人且允许
执行器上电，然后输入：

```text
ENABLE_RT_CONTROL
```

看到下面这行才表示可以让其他域开始联调：

```text
READY: rt-control 已启动并完成 /rt/enable。
```

## 这条命令自动完成什么

1. 锁定当前账号、主机名、PREEMPT_RT 内核和隔离 CPU 14；
2. 核对不可变 release、镜像 tag 和镜像 ID，禁止启动未验收镜像；
3. 核对 Docker、IgH、16 个 EtherCAT 位置、两只 CANable 序列号、500 kbit/s、Node 2/3 心跳和 BMS `0x3FC`；
4. 要求一次现场使能确认；
5. 启动同一个 rt-control 容器，等待 controller、总线、PLC 状态和 BMS 电压/SOC ready；
6. 自动调用 `/rt/enable`，确认 JTC active、EtherCAT OP 和 `/joint_states` 有数据。

如果任一步失败，脚本返回 `FAIL`；只要容器已经开始启动，它会尝试先调用
`/rt/disable`，再有序停止 rt-control。脚本**不会自动调用 `/rt/reset_fault`**。

## 联调同事常用的另外三条命令

查看状态：

```bash
./tools/rt_control_ipc.sh status
```

持续查看日志，按 `Ctrl+C` 只会退出日志查看，不会停止控制：

```bash
./tools/rt_control_ipc.sh logs
```

联调结束后整组失能并有序停止：

```bash
./tools/rt_control_ipc.sh stop
```

停止成功会显示 EtherCAT 已到 Idle/Inactive、16 个从站全 PREOP。PLC/BMS 与控制栈同属
`robot-rt-control-1`，没有第二个 `robot-rt-io-1`；该命令不会停止或重启其他域容器。

## READY 后可用的当前工程接口

| 功能 | 接口 |
| --- | --- |
| 14 轴轨迹 | `/dual_arm_jtc/follow_joint_trajectory` |
| 履带速度 | `/cmd_vel` |
| 关节状态 | `/joint_states` |
| 里程计 | `/diff_drive_controller/odom` |
| 动态 TF（新镜像） | `/tf`，`tf2_msgs/msg/TFMessage` |
| 静态 TF（新镜像） | `/tf_static`，`tf2_msgs/msg/TFMessage` |
| PLC 状态（新镜像） | `/plc/io_state`，`robot_interfaces/msg/PlcIoState` |
| BMS 电压/SOC（新镜像） | `/bms/battery_state`，`sensor_msgs/msg/BatteryState` |
| PLC 三路控制（新镜像） | `/plc/left_solenoid`、`/plc/right_solenoid`、`/plc/vacuum_pump` |
| 诊断 | `/diagnostics` |
| 手工失能 | `/rt/disable` |

新镜像中的本体链为 `odom → base_footprint → base_link → 本体/传感器连杆`；`map → odom`
由 perception/定位负责。导航与视觉不得重复发布 rt-control 已拥有的边。

这些名称当前可用于联调，但域间公共契约尚未冻结。接口类型和限制见
[开发进度与联调准入](integration-readiness-summary.md)。

## 出现 FAIL 时

- 不要连续重复运行启动命令；
- 不要手工执行 `docker compose`、`ros2 control switch_controllers` 或循环 fault reset；
- 运行 `./tools/rt_control_ipc.sh logs`，把完整 `FAIL`、日志时间和现场现象交给
  rt-control 负责人；
- 如果存在人员或设备风险，使用实体急停/STO，不要等待软件命令。

一键命令的设计目的，是让同事不需要记忆 Compose 参数和 ROS 生命周期顺序；它不是
急停、安全 PLC 或自动故障恢复系统。
