# rt-control 一键启动

> 仅适用于已经部署好的 `ar@192.168.0.40`。脚本会访问真实 EtherCAT/CANopen，
> 并自动调用 `/rt/enable`；不能在开发电脑、其他工控机或无现场授权时运行。

> **交付前置：** 新脚本当前已完成静态策略验证，但尚未作为一个组合命令在实机首跑。
> rt-control 负责人必须先把本提交的 `tools/` 部署到该 IPC，并在现场完成一次受控
> “启动→READY→stop”验收；验收通过后，联调同事才按本页使用。

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
3. 核对 Docker、IgH、16 个 EtherCAT 位置、CANable 序列号、500 kbit/s 和 Node 2/3 心跳；
4. 要求一次现场使能确认；
5. 启动 rt-control，等待 controller 和总线 ready；
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

停止成功会显示 EtherCAT 已到 Idle/Inactive、16 个从站全 PREOP。该命令只处理
`robot-rt-control-1`，不会停止或重启 `robot-rt-io-1` 和其他域容器。

## READY 后可用的当前工程接口

| 功能 | 接口 |
| --- | --- |
| 14 轴轨迹 | `/dual_arm_jtc/follow_joint_trajectory` |
| 履带速度 | `/cmd_vel` |
| 关节状态 | `/joint_states` |
| 里程计 | `/diff_drive_controller/odom` |
| 诊断 | `/diagnostics` |
| 手工失能 | `/rt/disable` |

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
