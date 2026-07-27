# 14 轴 FJT 最小低速运动记录 — 2026-07-27

## 结论

镜像 `rt-control:4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d` 在
`ar@192.168.0.40` 完成了首轮 14 轴最小低速 FJT。13 个旋转轴各执行一次
`0.5 degree` 外移和一次回程，EtherCAT Updown 执行一次 `0.05 m` 外移和一次
回程；每个单程的插补时间均为 `2 s`。28 个完整 14 轴 FJT goal 全部成功，客户端
退出码为 0，随后整组 `/rt/disable`、Compose 有序停止和最终总线检查均成功。

这证明当前镜像能够完成 14 轴完整 goal 的逐轴 CSP 小位移往返、当前位置回程、
标准失能和有序主站退出，但**不是 T-019 全部实机验收完成**。本次没有执行多轴
同时运动、第一点超差拒绝、Fault/EMCY/断链注入、30 分钟 OP 空跑或生产负载轨迹。
现场操作者已确认急停条件和当前位姿两个方向均无碰撞；日志只能证明编码器坐标方向
与命令一致，尚未收到逐轴实体运动方向、噪声和振动的人工观察记录。

**持续风险：** BQ-120 的物理链路异常在本轮再次出现。IgH `Lost frames` 从 399
增加到 402，slave 2 port 0 从 `CRC=8, PHY=6` 增加到 `CRC=11, PHY=8`，下游
转发计数由 8 增加到 11。内核共记录三次 `4 datagrams TIMED OUT`，其中一次发生在
FJT 运动窗口内。在已批准的 `0x10F1:02=250` 配置下没有驱动掉使能，所有 action 仍成功，
但这不等于通信根因已消失；`slave 1 port 1 -> slave 2 port 0` 线缆、接头或端口的
维护和 30 分钟复测仍然必须保留。

## 受控输入和执行策略

- 现场输入：旋转轴单程 `0.5 degree = 0.008726646259971648 rad`，Updown 单程
  `0.05 m`，每个单程 `2 s`。
- 每个 goal 都包含固定顺序的全部 14 轴：`right_joint1..6,left_joint1..6,turn,updown`；
  没有使用 partial goal。
- 每次只改变一个轴，其他 13 轴随 goal 保持；该轴完成外移后立即以另一个完整 goal
  回到本轮起点，再进入下一轴。
- 外移方向选择朝对应软限位区间中点，以增加本轮余量。好处是能把单轴异常定位到
  明确 goal 并减小首次联合运动能量；弊端是没有覆盖反方向首次外移、轴间耦合或
  14 轴同时运动负载。
- 每个 positions-only goal 的第一点为采样当前位置、`time_from_start=0.2 s`，第二点为
  目标、`time_from_start=2.2 s`，因此两点之间为 2 秒线性插补。名义速度为旋转轴
  `0.25 degree/s`、Updown `0.025 m/s`。这种最小 positions-only 轨迹没有验证生产
  轨迹的速度/加速度时间参数化，不能代替 motion 域的轨迹生成。
- 客户端在每段记录 action feedback、移动轴最大绝对跟踪误差和终点误差；任一 goal
  拒绝、action 失败、越限或反馈方向与命令相反都会中止后续 goal。

临时客户端没有加入生产包或重建镜像；归档副本 SHA-256 为
`22ee9ec5793b1e47928ca93b49ad4ae1d6c1e1cf2bed93bdd74f080efa71609a`。它只用于本次
经批准的 commissioning，测试后已从目标机 `/tmp` 删除。

## 运动结果

- 28/28 段成功，14/14 轴均完成外移和回程；每段收到 44 个 action feedback，合计
  1232 个，整轮客户端耗时 `71.139714 s`。
- 旋转轴移动轴最大跟踪误差出现在 `right_joint3` 外移：
  `0.00284364579484 rad = 0.162928902 degree`。
- 旋转轴最大终点绝对误差同样出现在 `right_joint3` 外移：
  `0.00172785891837 rad = 0.0989990236 degree`。
- Updown 最大跟踪误差为回程的 `0.00184546718425 m`，最大终点绝对误差为回程的
  `0.000252685546875 m`。
- 整轮完成位置与整轮开始位置相比，最大旋转轴差为
  `0.0014620754378 rad = 0.0837707519 degree`，Updown 差为
  `0.00003662109375 m`。逐段客户端以最新实际位置生成非移动轴保持点，故该差值是
  28 段累计后的实测回位差，不应解释为绝对标定误差。

上述数值只做测量记录。冻结要求没有为这次最小运动规定新的跟踪误差合格阈值，
因此不能因为 action 返回 success 就自行宣布生产动态精度验收通过。

## 资源和调度记录

从 FJT 开始前采集的 180 秒 `mpstat/pidstat/vmstat` 窗口覆盖整轮运动以及随后的
保持/检查阶段：

- ros2_control 实时更新线程实扫为 `SCHED_FIFO/80`，运行在 CPU 14；
- CPU 14 平均 busy `4.945%`、1 秒峰值 `23.230%`；
- IgH 当前绑定的 CPU 12 平均 busy `0.315%`、峰值 `1.980%`；
- 全机平均 busy `0.714%`、峰值 `1.550%`；
- `ros2_control_node` 进程平均 CPU `3.49%`，RSS `591157 KiB`、`1.81%`；
- `vmstat` 的最大 runnable 为 6，blocked、swap-in 和 swap-out 均为 0。

补充的 Docker 宽窗口采样包含启动、等待、运动和停止，非零样本平均 CPU
`4.533%`、峰值 `40.020%`，内存平均 `573.020 MiB`、峰值 `601.900 MiB`。
该数据只能描述资源占用，未测量 4 ms 循环 deadline latency 或 jitter，不能替代
T-009 的实时延迟和 30 分钟联合负载验收。

## 通信、失能和退出

启动后 14 个运动从站均为 OP/WC-complete，hub position 13 按设计保持 PREOP。
三次成组 datagram timeout 分别出现在目标机时间 `09:18:44`、`09:22:21` 和
`09:26:48`；最后一次位于 FJT 窗口。运动后 14 个运动从站仍为 OP，没有 action
拒绝、驱动 Fault 或整组掉使能。

运动完成后的 `/rt/disable` 返回 `ok=true, stage=success`，JTC 变为 inactive，
enable manager diagnostics 为 `IDLE/success`。随后 Compose stop 返回 0，当前运行日志
无 CANopen use-after-free、SIGSEGV、`0x001B` 或 `UNCLEAN_SHUTDOWN`；容器最终
`ExitCode=0`。退出瞬间 IgH 仍记录一次 `Failed to receive AL state datagram:
Datagram initialized` 和一次 master-FSM datagram skipped，但最终 master 为
Idle/Inactive、16 个从站全 PREOP。CAN0/CAN1 均为 500 kbit/s、ERROR-ACTIVE，
bus error、drop、bus-off 均为 0；`robot-rt-io-1` 保持运行。

## 证据和剩余门禁

目标机完整证据目录为：

```text
/var/lib/rt-control/validation/run-18-fjt-low-speed-4fc8414/
```

其中包括逐段 `fjt-run.jsonl`、客户端和 hash、`fjt-summary.txt`、CPU/进程/内存原始
采样、启动前/运动后/停机后 EtherCAT 与 CAN 状态、内核和当前容器日志、服务响应及
`evidence-manifest.sha256`。清单已用 `sha256sum -c` 校验通过。

后续仍需：

1. 现场操作者补记 14 轴实体方向、异响和振动观察结论；
2. 维护 `slave 1 port 1 -> slave 2 port 0` 物理段并完成至少 30 分钟零增量 OP 空跑；
3. 单独批准后验证第一点超差拒绝、Fault/EMCY/断链及恢复；
4. 使用 motion 生成的限速/限加速度生产轨迹验证多轴协调和实时 deadline/jitter。
