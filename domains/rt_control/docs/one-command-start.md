# rt-control 一键启动

> 仅适用于已经部署好的 `ar@192.168.0.40`。脚本会访问真实 EtherCAT/CANopen，
> 并自动调用 `/rt/enable`；不能在开发电脑、其他工控机或无现场授权时运行。

> **当前发布候选：** `V0.10`。镜像以 GitHub Release 附件发布，目标机应先校验
> `rt-control-V0.10-<git-sha>.tar.gz.sha256`，再 `docker load` 得到 `rt-control:V0.10`。
> 目标机受控“启动→READY→stop”验证完成后，`~/rt-control-current` 才能作为同事固定入口。

> **T-020 TF 发布边界：** 前序镜像已在目标机完成 TF 只读复核；本发布候选延续
> `base_footprint → base_link` 静态边，并仍不发布 `odom → base_footprint`。

> **接口边界：** 当前发布候选输出 `/wheel/odom`，不发布 `odom → base_footprint`；该边由导航唯一发布。
> 真空公共接口为 `/vacuum/grip`、`/vacuum/state`、`/vacuum/pump/set_enabled`，状态只暴露 `left/right`
> 布尔吸附结果，不暴露压力值。

> **PLC/BMS 发布边界：** 前序镜像已完成 PLC/BMS 读取、三路输出逐点 ON/OFF 和关停复测；本发布候选的一键启动仍会要求
> `can1` 的批准序列号、UP/500 kbit/s 和 `0x3FC` 帧，并等待 `/plc/io_state`、`/battery_state` 有效后再自动使能。
> 详细证据见 [PLC / BMS 与一键启动实机验收](plc-bms-commissioning-20260728.md)。

## 启动

SSH 登录这台工控机后，复制下面两行：

```bash
cd ~/rt-control-current
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
启停: /control/set_enabled
FJT: /whole_body_jtc/follow_joint_trajectory
底盘: /cmd_vel_safe
真空: /vacuum/grip
```

## 这条命令自动完成什么

1. 锁定当前账号、主机名、PREEMPT_RT 内核和隔离 CPU 14；
2. 核对 `V0.10` 不可变 release 操作副本和本地 `rt-control:V0.10` 镜像是否存在；
3. 核对 Docker、带 fixed-PDO 保护的 IgH、18 个 EtherCAT 位置（Hub 0/13、X503 14/15）、两只 CANable 序列号、500 kbit/s、Node 2/3 心跳和 BMS `0x3FC`；
4. 要求一次现场使能确认；
5. 启动同一个 rt-control 容器，等待 controller、总线、PLC 状态和 BMS 电压/SOC ready；
6. 自动调用 `/rt/enable`，确认 JTC active、EtherCAT OP 和 `/joint_states` 有数据。

如果任一步失败，脚本返回 `FAIL`；只要容器已经开始启动，它会尝试先调用
`/rt/disable`，再有序停止 rt-control。普通启动路径**不会自动调用 `/rt/reset_fault`**。

## 主接触器急停掉电后的恢复

仅在主接触器已经重新上电、现场安全条件重新成立后运行：

```bash
./tools/rt_control_ipc.sh recover-power-loss
```

脚本先对旧会话最佳努力失能并销毁旧容器，确认 EtherCAT master 已经 Idle/Inactive，随后才要求输入
`RECOVER_RT_CONTROL`。确认后它会启动全新会话，等待总线/controller 稳定，只调用一次全组
`/rt/reset_fault`，逐轴确认非激磁状态，再只调用一次 `/rt/enable` 并确认 14 轴 Operation Enabled 和 JTC active。

四个 Ti5 按 BQ-115 允许在标准 `0x0000` 下以 `Ready To Switch On (0x0021)` 作为已确认非激磁终态；其余
10 轴严格要求 `Switch On Disabled (0x0040)`。任一步失败都会停止并清理新会话，不循环 reset/enable。
该命令不是急停检测、STO 或自动复电功能；目标机切换到 `V0.10` 并完成单独实机验收前不要使用。

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

停止成功会显示 EtherCAT 已到 Idle/Inactive、18 个从站全 PREOP。PLC/BMS 与控制栈同属
`robot-rt-control-1`，没有第二个 `robot-rt-io-1`；该命令不会停止或重启其他域容器。

## READY 后可用的当前工程接口

| 功能 | 接口 |
| --- | --- |
| 14 轴轨迹 | `/whole_body_jtc/follow_joint_trajectory` |
| 履带速度 | `/cmd_vel_safe`，`geometry_msgs/msg/Twist`，0.5 s 超时 |
| 关节状态 | `/joint_states`，仅 14 个 EtherCAT 机械轴，实频 125 Hz |
| 原始轮速里程计 | `/wheel/odom` |
| 动态 TF | `/tf`，`tf2_msgs/msg/TFMessage`；rt-control 不发布 `odom → base_footprint` |
| 静态 TF | `/tf_static`，`tf2_msgs/msg/TFMessage`；包含 `base_link → lidar_main` 及其他固定本体/传感器边 |
| PLC 状态 | `/plc/io_state`，`rt_control_interfaces/msg/PlcIoState` |
| BMS 电压/SOC | `/battery_state`，`sensor_msgs/msg/BatteryState`，5 s 周期 |
| 真空动作 | `/vacuum/grip`，`robot_rt_control_interfaces/action/VacuumGrip`；通道 `left/right`，布尔吸附验证；当前 `grip_profile_id=default` |
| 真空状态 | `/vacuum/state`，`robot_rt_control_interfaces/msg/VacuumState`；无 `pressure_pa` |
| 真空泵维护 | `/vacuum/pump/set_enabled`，`robot_rt_control_interfaces/srv/SetPumpEnabled` |
| PLC 底层调试 | `/plc/left_solenoid`、`/plc/right_solenoid`、`/plc/vacuum_pump` |
| 安全摘要 | `/control/safety_state`，`robot_rt_control_interfaces/msg/SafetyState` |
| 就绪状态 | `/rt_control/readiness`，`robot_system_interfaces/msg/DomainReadiness` |
| 诊断 | `/diagnostics` |
| 公共启停 | `/control/set_enabled`，`robot_rt_control_interfaces/srv/SetControlEnabled`；`true` 时可自动清一次 resettable fault 后使能，`false` 只调用 `/rt/disable` |
| 手工失能 | `/rt/disable` |

rt-control 的本体链从 `base_footprint → base_link → 本体/传感器连杆` 开始；导航域发布最终 `/odom` 和唯一动态
`odom → base_footprint`，`map → odom` 仍由定位侧负责。`/wheel/odom` 是消息 topic，不是 TF frame。导航未启动时
缺少 `odom → base_footprint` 是预期行为，其他域不得补发重复边。

这些名称已对齐公共 `robot_interfaces` 契约 0.7.0。接口类型和限制见 vendored
`robot_interfaces/contract/views/rt_control.md` 和
[开发进度与联调准入](integration-readiness-summary.md)。

## 出现 FAIL 时

- 不要连续重复运行启动命令；
- 不要手工执行 `docker compose`、`ros2 control switch_controllers` 或循环 fault reset；
- 运行 `./tools/rt_control_ipc.sh logs`，把完整 `FAIL`、日志时间和现场现象交给
  rt-control 负责人；
- 如果存在人员或设备风险，使用实体急停/STO，不要等待软件命令。

一键命令的设计目的，是让同事不需要记忆 Compose 参数和 ROS 生命周期顺序；它不是
急停、安全 PLC 或自动故障恢复系统。
