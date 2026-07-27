# XMC Updown 首次整组使能记录 — 2026-07-27

## 结论

这是 T-019 的**部分实机证据，不是 T-019 完成**。镜像
`rt-control:95527cc88700a50a90e193ac9afa63a00d05e907` 在
`ar@192.168.0.40` 上完成了 16-position EtherCAT 启动、XMC 固定 PDO 注册、
当前位置预装载、整组 fault reset、五批 14 轴使能和整组失能。没有发送
FJT、`/cmd_vel` 或任何有意运动目标。

使能和失能本身成功，但本次测试发现两个阻塞后续运动验收的高风险问题：

1. OP 运行期间仍周期性出现成组 EtherCAT datagram timeout，WC/lost-frame
   计数继续增长；
2. 退出时 ros2_canopen polling callback 与 driver cleanup 发生生命周期竞态，
   `ros2_control_node` 在 `LelyDriverBridge::get_id()` 段错误，导致 EtherCAT
   未走到正常 hardware deactivate 就被进程释放。

因此当前版本可以证明 SW 5.11 固定 PDO 和 14 轴使能路径可工作，不能证明
持续通信、运动或联合优雅退出已经通过。

> 后续状态：本页保留首次镜像的原始失败证据。BQ-119/BQ-122 已由 corrective
> image `4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d` 的三轮实机启停关闭；
> `0x10F1:02=250` 首镜像门禁也已通过。完整结果见
> [CANopen 有序清理与 EtherCAT 同步容忍实机复测](canopen-shutdown-sync-tolerance-commissioning-20260727.md)。

## 发布物与现场前提

- Git/T-017：`95527cc88700a50a90e193ac9afa63a00d05e907`；
- image ID：`sha256:3116276ae004ec85bda8407d1ec98819a00310e77083cca0877c04211550d3ac`；
- 目标内核：`5.15.0-1032-realtime`，控制 CPU 14；
- EtherCAT：16 positions，position 15 为 XMC `0x34E/0x445566/rev0`；
- XMC 固件：HW 1.0 / SW 5.11，固定 Rx/Tx 为 19/24 bytes；
- CAN：唯一在场的 CANable serial 为批准的 rt-control
  `004D00675230500720333159`。

第二只 BMS CANable 不在场，导致 `rt-control-can-names.service` 按现有严格
策略失败。只为本次受控测试，在逐字确认现有 `can0` serial 后，以
`systemctl start --job-mode=ignore-dependencies can0.service` 启动原 unit；unit
仍使用 500 kbit/s 和 txqueuelen 128，没有修改 systemd 文件。测试结束后已
停止该 unit，`can0` 恢复 DOWN。

## 启动与固定 PDO 证据

启动日志逐项注册 position 15：

- RxPDO：`6040,6071,60FF,607A,6081,6060,2302`；
- TxPDO：`6041,603F,6078,606C,6064,6061,6000,2300,2301,60FD`。

约 57 秒后驱动报告 `All configured EtherCAT slaves are OP with a complete
working counter`。运行时 master 诊断为 link=1、slaves_responding=16，XMC
slave 15 为 AL=8、Switch On Disabled；CAN Node 2/3 为 PV Operation Enabled，
命令保持零速。

启动清理按既有 BQ-115 证据发现 `left_joint3` statusword
`4616 = 0x1208`，enable manager 进入 `FAILED/fault_requires_reset`。一次整组
`/rt/reset_fault` 返回 `ok=true, stage=success`，随后状态回到 IDLE。

## 使能、保持和失能

一次 `/rt/enable` 返回：

```text
ok=true failed_batch=-1 failed_joint="" status_word=0 stage=success
```

使能后：

- enable manager=`ENABLED`；
- `dual_arm_jtc`=active；
- XMC slave 15=`OperationEnabled`，原始 statusword `34359 = 0x8637`；
- Updown 使能前位置 `0.460704345703125 m`，使能后
  `0.46070419311523436 m`，差 `-0.00000015258789064 m`，等于 1 count；
- 未发送轨迹或速度命令，未观察到目标位置跳变。

发现 WC 仍增长后立即执行一次 `/rt/disable`，返回 `ok=true`。之后 JTC
inactive、enable manager=IDLE、XMC=Switch On Disabled。

## EtherCAT 通信异常证据

启动前 IgH `Lost frames=391`，最终为 `399`。rt_diagnostics 的
`wc_error_count` 在控制栈稳定后观测为 705，运行中依次升到 710、712。
内核日志在 OP 阶段记录八次成组 timeout，单次为 3 或 4 datagrams，例如：

```text
21:07:43 EtherCAT WARNING 0: 3 datagrams TIMED OUT!
21:08:03 EtherCAT WARNING 0: 4 datagrams TIMED OUT!
21:11:59 EtherCAT WARNING 0: 3 datagrams TIMED OUT!
```

启动阶段还出现 `850 datagrams TIMED OUT`、多个从站 DC sync 超时和
`0x001B Sync manager watchdog` 恢复过程。四个 Ti5 position 2/3/8/9 仍会
收到 IgH 对固定 PDO remap 的尝试并以 `0x06010002 read-only object` 拒绝，
尽管当前映射与目标一致且最终能进入 OP。本段记录首轮镜像的 400 ms 边界；
随后用户已在 BQ-120 明确接受短抖动并批准新构建改为 nominal 1000 ms。该变更的
好处是减少无效同步故障，弊端是真实连续同步故障的驱动侧反应最多再延后约
600 ms；它不消除、隐藏或关闭这里记录的通讯丢帧根因。

## 联合退出故障

PID 1 的退出前失能成功：

```text
rt_control shutdown disable result: ok=true stage=already_disabled
```

随后 controller manager 在 `canopen_mobile_axes` deactivate 中清理
`left_track_joint` 时，一个仍在 MultiThreadedExecutor 中运行的 polling
timer 访问已经失效的 Lely bridge：

```text
NodeCanopen402Driver::poll_timer_callback()
NodeCanopenBaseDriver::poll_timer_callback()
LelyDriverBridge::get_id()
Segmentation fault (Address not mapped to object [0x2d8])
```

`ros2_control_node` 退出码为 `-11`，但 ROS launch/PID 1 最终让容器显示
exit 0。进程崩溃后 EtherCAT 被动 release，position 1..15 先后出现
`0x001B Sync manager watchdog`，最终才全部回到 PREOP。CAN Node 2/3 最终
心跳为 `0x04`（Stopped），CAN 错误计数为 0；EtherCAT master 最终 inactive，
16 个位置全部 PREOP。

后续窄补丁已把 CANopen 基类顺序改为先停 callback/thread 再释放 bridge，并在
driver cleanup 前取消、join 专用 MultiThreadedExecutor。此文仍保留首轮崩溃原始
证据；补丁只有在新镜像重复启停无 SIGSEGV 且 EtherCAT 明确有序退出后才算实机闭环。

## 后续门禁与方案权衡

在再次上电测试前至少完成：

1. 修正 ros2_canopen 退出顺序，使 polling timer/callback 先取消并从 executor
   排空，再释放 Motor402/Lely bridge。好处是针对实际 use-after-free 根因且
   保留上游状态机；弊端是需要维护并发生命周期补丁和重复压力退出测试。
   直接先停整个 executor 的方案更彻底，但可能让现有 cleanup 中投递到
   executor 的任务永远无法完成，死锁风险更高。
2. 继续定位周期性 datagram timeout 的来源，至少联查 NIC IRQ/CPU affinity、
   OP 周期调度、DC/WC 统计和物理链路。用户随后明确接受短通信抖动风险并批准把
   `0x10F1:02` 提高到 `250`（nominal 1000 ms）；好处是减少短抖动导致的无效同步
   故障，弊端是真实连续同步故障最多约晚 600 ms 才由驱动升级。该值不会消除或
   隐藏 lost/WC 统计。后续只读 CRC 检查已把根因优先定位到
   `slave 1 port 1 -> slave 2 port 0` 物理段，具体线缆/接头/端口仍待断电检查和
   30 分钟空跑确认。
3. 上述 corrective image 已完成三轮重复启停，其中一轮覆盖
   “启动→reset→enable→保持→disable→stop”，lost frame 保持 399、无 `0x001B`
   退出级联且进程/容器均干净退出。下一项仍是经现场批准的 XMC/14 轴低速小位移
   CSP/FJT 运动验收。
