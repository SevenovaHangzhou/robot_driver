# PLC / BMS 与一键启动实机验收记录

日期：2026-07-28

目标机：`ar@192.168.0.40`

任务：T-RT-IO-005

需求：REQ-RT-004、REQ-IF-005、REQ-DEP-001、REQ-DEP-002、REQ-TEST-001

裁决：BQ-124

## 1. 结论

关停修复版已经在目标机完成 PLC/BMS 读取、三路 PLC 输出逐点开关、14 轴自动使能保持和一键有序停止。当前交付可供
同机工程联调使用：一键启动得到 `READY`，停止后容器 `ExitCode=0`，EtherCAT 回到 Idle/Inactive、16 个从站全
PREOP，CAN0/CAN1 无错误，PLC 三路命令和实际输出均回到关闭状态。

本记录不把以下人工观察写成已经完成：BMS 数值与独立 HMI 的目视对照、左/右执行器物理身份及共用泵气路效果。
协议值、PLC 命令寄存器和 PLC 实际输出寄存器已经验证；上述现场物理对应关系仍应在正式工艺联调时由电气/机械人员
复核。

## 2. 交付身份

| 项目 | 固定值 |
| --- | --- |
| 功能源码 | `d415c0c2c75917a9545a4a2f87487718de8622a2` |
| 发布锁 | `7bc8f16e903ef7174f60ebe9613f6eeed3a96185` |
| 镜像 | `rt-control:d415c0c2c75917a9545a4a2f87487718de8622a2` |
| 镜像 ID | `sha256:01bd550b068fccb9158b007067e55c30eed7d7d7253ef9179dfdf6d9be9a11c2` |
| 镜像大小 | `552557165 bytes`，约 `527.0 MiB` |
| 不可变运行副本 | `/home/ar/rt-control-releases/d415c0c2c75917a9545a4a2f87487718de8622a2/robot` |
| 不可变操作副本 | `/home/ar/rt-control-operators/7bc8f16e903ef7174f60ebe9613f6eeed3a96185/robot` |
| 固定操作入口 | `/home/ar/rt-control-current`，指向上述操作副本 |

运行源码和操作锁使用两个 SHA 是有意设计：镜像身份继续精确对应已构建的功能提交，后续文档/launcher 锁提交不会让
同一 tag 指向不同内容。代价是发布记录必须同时保存两个 SHA，且更新 `rt-control-current` 前必须重新核对二者和镜像
ID，禁止把它随意指向开发工作区。

## 3. 候选拒绝与修复闭环

首次候选 `e4fed685bfa4485c210ad038c804a331b4801d88` 能正确读取 PLC/BMS，但停止时两个 Python 节点收到
`ExternalShutdownException` 后退出，launch 又在 ROS context 关停期间将它们重生。Docker 等待 100 秒后只能强制
终止，退出码为 137，因此该候选已拒绝，证据保存在：

```text
/var/lib/rt-control/validation/run-19-plc-bms-e4fed685/
```

修复版不再让 launch 无条件 respawn PLC/BMS 节点，并将 `ExternalShutdownException` 作为正常关停。普通 CAN/Modbus
断线仍由节点内部重连处理。受影响四包构建、49 项包测试、26 项仓库测试和带 IO 的 controller-ready Mock 均通过；
修复版目标机重复停止约 3 秒完成、退出码 0，没有 traceback、进程死亡、respawn 或 `UNCLEAN_SHUTDOWN`：

```text
/var/lib/rt-control/validation/run-20-plc-bms-shutdown-fix-d415c0c/
```

## 4. PLC / BMS 读取

修复版实测 `/plc/io_state` 为 `connected=true`、`data_fresh=true`、三路输出关闭、两路真空未建立、报警字 0；发布
频率为 2 Hz。`/bms/battery_state` 能从 CAN1 标准帧 `0x3FC` 得到有效数据，复测样本为 `48.6 V`、SOC `0.23`、
`present=true`，发布频率为 0.2 Hz。CAN1 为 500 kbit/s、ERROR-ACTIVE，只有 `bms_node` 发布该 ROS 话题。

目标机实扫 RMW 为 `rmw_fastrtps_cpp`。TF 同时验证了静态 `base_footprint → base_link` 的精确高度 `0.202094 m`，
以及可查询的组合变换 `odom → base_link`；该变换没有恢复已删除的 `world` 父节点。

## 5. 三路输出逐点开关

测试时未调用 `/rt/enable`，一次只改变一个目标 bit。每个 ON/OFF 请求均同时由 `%MW200` 命令字和 `%MW211` 实际
输出字确认，其他输出位保持不变：

| 服务 | ON 结果 | OFF 结果 |
| --- | --- | --- |
| `/plc/left_solenoid` | `success=true`，MW200/MW211=`0x0001` | `success=true`，均回 `0x0000` |
| `/plc/right_solenoid` | `success=true`，MW200/MW211=`0x0002` | `success=true`，均回 `0x0000` |
| `/plc/vacuum_pump` | `success=true`，MW200/MW211=`0x0004` | `success=true`，均回 `0x0000` |

测试全过程 `%MW201=0x0001`，`%MW212=0`。末态 `/plc/io_state` 再次报告三路输出均关闭。左阀第一次命令在 ROS CLI
参数解析阶段失败，没有形成 service request，寄存器保持基线；修正引号后按上表通过。这是测试命令错误，不是 PLC
或节点失败，原始记录未删改：

```text
/var/lib/rt-control/validation/run-21-plc-output-d415c0c/
```

## 6. 一键使能、保持与停止

通过锁定操作副本执行生产一键命令，全部前置检查通过后输入批准短语。脚本自动调用 `/rt/enable`，返回
`ok=true, stage=success`，并输出：

```text
READY: rt-control 已启动并完成 /rt/enable。
```

保持阶段没有发送 FJT 或 `/cmd_vel`：FJT action client 数为 0，`/cmd_vel` publisher 数为 0。四个 controller 全部
active；14 个执行轴保持当前位置，其中 Updown 样本为 `0.4606892395 m`。EtherCAT 为 Operation/Active、16 个从站
在线，累计 `Lost frames=402` 且观察窗口增量为 0；CANopen Node 2/3 heartbeat 均为 Operational。容器实扫值为
CPU14、`CAP_SYS_NICE`/`CAP_IPC_LOCK`/`CAP_NET_RAW`，采样约 4.00% CPU、644.5 MiB 内存。

随后执行同一个入口的 `stop`，`/rt/disable` 返回成功，容器退出码为 0。末态为：

- EtherCAT Idle/Inactive，16 个从站全 PREOP，`Lost frames` 仍为 402，Tx error 为 0；
- CAN0/CAN1 均为 500 kbit/s、ERROR-ACTIVE，restarted/bus-errors/arbit-lost/error-warn/error-pass/bus-off 全 0；
- PLC MW200=`0x0000`、MW201=`0x0001`、MW210/MW211/MW212=`0x0000`；
- 关停日志没有 traceback、process died、respawn、FATAL 或 `UNCLEAN` 标记。

完整证据及 SHA-256 清单位于：

```text
/var/lib/rt-control/validation/run-22-one-command-d415c0c/
```

## 7. 已知边界

- PLC/BMS 暂时与实时控制共用 CPU14。30 秒读取阶段 CPU14 平均 idle 约 95.63%，但这只是当前负载样本，不关闭未来
  motion/GPU 联合负载、jitter 或 WCET 验收。
- EtherCAT 物理段既有累计丢帧为 402；本次输出和一键使能窗口没有新增。线缆/接头/端口修复与 30 分钟零增量空跑
  仍是独立支线任务。
- BMS 适配器缺席时是否允许履带控制独立启动仍由开放的 BQ-121 管理。本次两只批准序列号的 CANable 都在场，不能用
  这次成功替代该架构裁决。
- `ShellCheck` 本机和目标机均未安装；其余本地门禁已通过，ShellCheck 仍由远程 CI 强制执行。
- 旧的重复 `robot-rt-io-1` 容器在归档 inspect/log 后已停止并移除，避免重复 PLC/BMS 发布。旧镜像和
  `/home/ar/rt-control-deploy/robot/docker/compose.io.yaml` 仍保留，可追溯但不得再次并行启动。
