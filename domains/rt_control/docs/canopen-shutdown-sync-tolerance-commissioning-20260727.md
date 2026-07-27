# CANopen 有序清理与 EtherCAT 同步容忍实机复测 — 2026-07-27

## 结论

T-019 corrective image
`rt-control:4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d` 在
`ar@192.168.0.40` 完成三轮完整启动/停止。已实机关闭两个确定性软件问题：

1. ros2_canopen polling callback 在 driver bridge 释放后继续执行导致的
   `LelyDriverBridge::get_id()` use-after-free；
2. 完整栈退出时 CANopen 清理先发生、EtherCAT 周期先停止而未 deactivate，导致
   14 个运动从站出现 `0x001B Sync manager watchdog`。

三轮容器均 exit 0，`ros2_control_node` 均干净退出；退出窗口没有
`SIGSEGV`、`UNCLEAN_SHUTDOWN`、`0x001B` 或 datagram timeout。第一轮执行了一次
全组 reset、14 轴 enable、原位保持和 disable；其余轮次没有使能。三轮均未发送
FJT、`/cmd_vel` 或其他运动目标。

## 发布物与依赖

- 仓库分支：`feature/rt-control-implementation`；
- 代码提交：`4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d`；
- 镜像 ID：`sha256:09c8a979c536955d160bc92c60e4531627f13b62a75b00ee108e6ef332226898`；
- IgH 标签提交：`2f7f884f1c7d377c02a7d627eb06512126a0e50e`；
- 目标发布目录：
  `/home/ar/rt-control-releases/4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d/robot`；
- 目标 cpuset：`14`；控制周期保持 250 Hz。

镜像按冻结顺序应用 ros2_canopen `0001/0002/0003` 三个窄补丁。前两道
callback 防线分别保证派生类 timer/thread 先停、专用 executor cancel/join 后才释放
driver；退出 helper 在同一个 30 s 总 deadline 内把 EtherCAT 明确降为 inactive，之后
wrapper 才转发 SIGINT。

## 同步容忍与首镜像门禁

全部九份共享 EtherCAT profile 的 `0x10F1:02` 从 `100` 改为 `250`。4 ms
周期下 nominal 容忍由约 400 ms 增至约 1000 ms；不改变 250 Hz、DC、PDO、完整
WC 启动门禁或 diagnostics WARN 语义。

首次启动对全部 14 个运动从站逐一执行 `uint16` 只读上传，九台 ZeroErr、四台
Ti5 和 XMC 均返回 `0x00fa/250`，且没有 `0x10F1:02` startup abort，随后所有
配置运动从站进入 OP/WC-complete。Ti5 YAML 仍按供应商 ESI 使用 `uint32`，因此
实机 2-byte 上传与 ESI 32-bit 的 BQ-117 类型矛盾并未被这次成功写值消除。

好处：已观察到的 12–16 ms 成组短抖动不再容易触发驱动本地同步故障或整组掉使能。

弊端（已接受风险）：真实连续同步故障最多约晚 600 ms 才由驱动对象升级；该设置
不会消除或隐藏 IgH lost frame/WC 统计，也不是急停、STO 或墙钟 watchdog。

## 三轮结果

| 轮次 | 运行范围 | 完成态 | 退出结果 |
| --- | --- | --- | --- |
| 1 | 16-position OP/WC-complete；14 轴 reset、enable、原位保持、disable；无运动命令 | XMC 与其余 13 轴成功失能 | helper 顺序完整，约 1.9 s 停止，容器 exit 0，无 UAF/`0x001B` |
| 2 | 完整启动并待机；不 enable、不运动 | `already_disabled` | helper 顺序完整，约 1.8 s 停止，容器 exit 0，无 UAF/`0x001B` |
| 3 | 14 个运动从站全部 OP；不 enable、不运动 | `already_disabled` | helper 顺序完整，约 3.0 s 停止，容器 exit 0，无 UAF/`0x001B` |

每轮都出现以下有序证据：

```text
rt_control shutdown disable result: ok=true
rt_control shutdown controllers quiesced: enable_manager,joint_state_broadcaster
resource_manager: 'deactivate' hardware 'ecat_arms'
EthercatDriver: System successfully stopped!
rt_control shutdown EtherCAT hardware state: inactive
resource_manager: 'deactivate' hardware 'canopen_mobile_axes'
CanopenSystem: Exiting spin thread...
left_track_joint: Cleanup
right_track_joint: Cleanup
```

第 3 轮完整 OP 后退出的最终状态为 EtherCAT master `Idle/Active: no`、16 个
位置全部 PREOP。CAN `can0`/`can1` 均保持 500 kbit/s、ERROR-ACTIVE，CAN 错误、
drop、bus-off 均为 0。`robot-rt-control-1` 保持 Exited (0)，无关的
`robot-rt-io-1` 保持运行。

## 通信统计与剩余风险

本轮开始前 IgH `Lost frames=399`，三轮结束后仍为 `399`，短窗口内没有新增
datagram timeout 或 WC/同步级联。这支持按已接受风险进入下一阶段最小低速验收，
但不代替 T-009 的 30 分钟联合空跑。

停机后的只读分层检查进一步把 BQ-120 定位到具体物理段：

- `ethercat crc`：slave 2 port 0 为 `CRC=8, PHY=6`，后续从站仅转发同一批
  `FWD=8`；
- `ethercat graph`：该入口就是 `slave 1 port 1 -> slave 2 port 0`；
- 计数对应：8 个 CRC 错误与首次运行 `Lost frames 391->399` 的增量完全一致；
- 主站侧：I210 PCIe 链路为 2.5 GT/s x1 正常状态，PCI/AER 三类错误均为 0，
  IgH Tx errors 为 0，timeout 时段没有 link-down。

因此首要维护对象是 slave 1 到 slave 2 的线缆、接头和相邻两端口。当前不能从
计数单独区分四者中的具体元件，必须断电检查/替换后复电空跑验证。CRC 没有在线
清零，避免破坏原始证据。

调度检查还发现 IgH module 被宿主额外设为 `run_on_cpu=12`，而 GRUB 只隔离
CPU 14；CPU 12 仍承载普通 timer/softirq 和 `enp4s0` IRQ。这是未写入仓库 host
contract 的次级风险，但三轮在相同配置下没有新增错误，且物理 CRC 与历史 lost
增量一一对应，所以本次不同时修改 CPU 归属。先修复单一物理变量并完成 30 分钟
OP 空跑，再独立评审主站线程 CPU 策略。

仍未完成：

- slave 1 port 1 到 slave 2 port 0 的线缆/接头/端口检查或替换，以及复电后 30 分钟 CRC/Lost/WC 零增长空跑；
- XMC/手臂/Turn 的受控低速 FJT 与机械方向、比例、限速验收；
- 履带方向/比例、heartbeat/EMCY 和断链后的机械停车验证；
- Ti5 `0x10F1:02` ESI 32-bit 与实机 2-byte upload 的厂商确认；
- BMS CANable 缺席时 systemd unit 是否允许 rt-control 独立启动的生产裁决。

## 证据归档

目标机证据目录：

```text
/var/lib/rt-control/validation/run-17-bq119-bq122-4fc8414/
```

目录包含三轮 container inspect/log、关键 service 返回、OP/最终 slave 状态、kernel
窗口、CAN 统计、CRC/拓扑、NIC/PCI/AER/IRQ/CPU 调度快照及最终 Docker 状态。`evidence-manifest.sha256` 已在目标机执行
`sha256sum -c` 校验通过。
