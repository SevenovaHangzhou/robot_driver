# rt-control Docker 部署与性能验证

> **当前结论：部署成功，可以供同机其他域开展联调。** 这里的“可以联调”表示容器、
> 实时控制、硬件通信、14 轴 FJT 和有序退出已有实机证据；不表示域间接口已经冻结，
> 也不表示整机生产安全验收已经完成。

本文只适用于当前工控机 `ar@192.168.0.40`。新工控机、不同内核或不同硬件拓扑不能
直接照抄本页数值。

## 1. 已部署的交付物

| 项目 | 当前值 |
| --- | --- |
| 发布版本 | `V0.10` |
| 实机运行源码 | GitHub tag `V0.10` 指向的提交 |
| 镜像 | `rt-control:V0.10` |
| 镜像归档 | GitHub Release `V0.10` 附件 `rt-control-V0.10-<git-sha>.tar.gz` 与 `.sha256` |
| 镜像大小 | 发布构建后记录 |
| 不可变运行副本 | `/home/ar/rt-control-releases/V0.10/robot` |
| 固定操作入口 | `/home/ar/rt-control-current` → `V0.10` 操作副本 |
| 操作系统/内核 | Ubuntu 22.04.5 LTS，`5.15.0-1032-realtime`，PREEMPT_RT |
| EtherCAT 主站 | IgH stable-1.6，commit `2f7f884f1c7d377c02a7d627eb06512126a0e50e` |
| ROS | ROS 2 Humble；已部署 V0.10 为 `ROS_DOMAIN_ID=0`；仓库 BQ-141 后由部署输入 `0..232`；`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |

运行副本由 GitHub Release `V0.10` 对应源码导出，不含 `.git`。开发仓库未显式设置
`RT_CONTROL_IMAGE_TAG` 时仍按 Git HEAD 生成临时镜像 tag；正式操作副本会显式传入
`RT_CONTROL_IMAGE_TAG=V0.10`，并要求目标机已从 Release 附件导入 `rt-control:V0.10`。

V0.10 的 Domain 0 是已部署快照，不因源码文档修改而自动变更。BQ-141 之后的新 release 必须在
发布清单中记录 `RT_CONTROL_ROS_DOMAIN_ID`，与其他域保持一致后再原子切换。

当前镜像已经包含 `/wheel/odom`、TF 所有权调整、PLC/BMS、`/control/set_enabled`、真空公共接口、
`/control/safety_state` 和 `/rt_control/readiness`。本页表格描述本次发布候选；目标机受控验证结果以
`PROGRESS.md` 最新记录为准。

## 2. Docker 内实现了什么

```mermaid
flowchart LR
    M[motion 等同机域<br/>当前工程接口] --> ROS[ROS 2 / 部署统一 Domain]

    subgraph C[rt-control 容器]
        B[rt_control_bringup]
        J[14 轴 JTC]
        D[履带 diff-drive]
        E[使能与故障管理]
        P[PLC IO]
        BM[BMS 状态]
        S[关节/里程计/诊断]
        H[EtherCAT + CANopen]
        B --> J
        B --> D
        B --> E
        J --> H
        D --> H
        E --> H
        P --> S
        BM --> S
        H --> S
    end

    ROS --> J
    ROS --> D
    E --> ROS
    S --> ROS
    H --> EC[16-position EtherCAT]
    H --> CAN[CANopen Node 2/3]
    P --> PLC[Modbus TCP PLC]
    BM --> BCAN[CAN1 / 0x3FC]
```

容器已经实现：

- 250 Hz `ros2_control` 实时控制，更新线程为 `SCHED_FIFO/80`；
- 12 个手臂关节、Turn、EtherCAT Updown 共 14 轴完整 FJT/CSP 执行；
- 左右履带 CANopen PV 与 `diff_drive_controller` 速度执行；
- 14 轴分批使能、整组失能、整组故障复位和任一轴故障后的整组处理；
- 公共 `/control/set_enabled` 服务适配到内部 `/rt/reset_fault`、`/rt/enable` 和 `/rt/disable`；
- `/joint_states`、里程计、`/tf`、`/tf_static`、FJT feedback/result 和当前工程版 `/diagnostics`；
- `/plc/io_state`、三路 PLC 单点输出服务和 `/battery_state`；
- 当前实际位置自动预装载，JTC 第一轨迹点一致性与 EtherCAT 反馈新鲜度检查；
- PID 1 停机包装器：先失能、停控制器和 EtherCAT，再清理 CANopen 与 ROS 进程。

容器启动会访问真实 EtherCAT，并激活 CANopen 履带硬件路径；它不是无硬件的普通
软件容器。14 个 EtherCAT 轴则由 `/rt/enable` 统一使能。

## 3. 重要运行配置

下面是已部署容器的实扫值，不是面向其他机器的推荐默认值。

| 配置 | 当前值 | 作用 |
| --- | --- | --- |
| 网络/IPC | `host` / `host` | 与同机 ROS 2 域通信并访问宿主 SocketCAN |
| CPU | `cpuset=14` | 使用已隔离的独立 P-core |
| 权限 | `CAP_SYS_NICE`、`CAP_IPC_LOCK`、`CAP_NET_RAW` | 实时调度、内存锁定和被动读取 CAN1；没有 `NET_ADMIN` |
| ulimit | `rtprio=98`、`memlock=-1` | 允许实时线程和锁定内存 |
| 设备 | `/dev/EtherCAT0:/dev/EtherCAT0` | 唯一直接映射的硬件设备 |
| 重启策略 | `unless-stopped` | 异常退出后的 Docker 级恢复策略 |
| 停止宽限 | `100 s` | 为失能和总线有序退出留出时间 |
| DDS 配置 | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=${RT_CONTROL_ROS_DOMAIN_ID:-0}`、`ROS_LOCALHOST_ONLY=0` | Domain 由部署显式配置（BQ-141）；无 DDS XML 挂载；Fast DDS 默认 UDP+共享内存传输 |

控制参数摘要：

| 项目 | 当前值 |
| --- | --- |
| 控制周期 | 250 Hz / 4 ms |
| `/joint_states` | 配置值 100 Hz；250 Hz 调度量化后的契约与 mock 实频为 125 Hz，发布镜像仍需复测 |
| `/wheel/odom` | 50 Hz；不发布 `odom → base_footprint` TF |
| RSP 活动本体 TF | 上限 50 Hz，独立于 `/joint_states` 发布频率 |
| 固定本体/传感器 TF | `/tf_static` transient-local；含 `base_footprint → base_link` |
| FJT 关节顺序 | `right_joint1..6,left_joint1..6,turn,updown` |
| FJT partial goal | 禁止 |
| 第一轨迹点误差 | 旋转轴 `1 degree`；Updown `0.05 m` |
| EtherCAT 反馈最大年龄 | `500 ms` |
| 底盘速度 | 线速度 `±0.3 m/s`；角速度 `±0.3 rad/s` |
| 底盘加速度 | 线/角均为 `±0.6`；jerk limit 未启用 |
| 履带无命令超时 | `0.5 s`，按普通减速约束归零 |

Updown 的机械范围为 `[0.0,0.8] m`，比例为 `6553600 counts/m`。配置文件中的
`0.3 m/s`、`0.5 m/s²` 是 motion 轨迹生成和验收上限；当前 JTC 不会自行完成
生产轨迹时间参数化。

## 4. 测试与性能结论

| 验证项 | 结论 |
| --- | --- |
| 镜像构建 | 24 个相关包完成干净构建；IgH、ICube、ros2_canopen 和 JTC 上游版本/补丁均锁定 |
| 配置与 Mock | 14 个 EtherCAT 轴、两条履带、controller 和生命周期接口均能加载；迁移/仓库门禁通过 |
| 实机通信 | 16-position EtherCAT 可到 OP/WC-complete；CANopen Node 2/3 可到 Operational |
| 实机使能 | 14 轴 reset/enable/hold/disable 路径通过；JTC 只在使能成功后 active |
| 最小运动 | 未改变控制核心的前代 `4fc8414…` 已完成 28/28 个完整 14 轴 FJT goal；当前镜像完成无命令自动使能保持，未重复发送运动 |
| PLC/BMS | 48.6 V/SOC 0.23 样本有效；PLC 2 Hz 状态有效；左阀、右阀、泵逐路 ON/OFF 的命令位和实际位一致 |
| 有序退出 | 当前镜像重复只读停止、逐点输出停止和完整一键停止均 exit 0；没有 Python respawn、CANopen UAF、SIGSEGV 或 `0x001B` 退出级联 |
| 最终状态 | 容器 `ExitCode=0`，EtherCAT Idle/Inactive、16 个从站全 PREOP；CAN0/1 ERROR-ACTIVE 且错误计数为 0；PLC 输出全关 |

未改变控制核心的 `4fc8414…` 运动与保持测试给出 180 秒性能窗口：

| 指标 | 结果 |
| --- | --- |
| CPU 14 平均/峰值 busy | `4.945% / 23.230%` |
| IgH 所在 CPU 12 平均/峰值 busy | `0.315% / 1.980%` |
| 全机平均/峰值 busy | `0.714% / 1.550%` |
| `ros2_control_node` 平均 CPU | `3.49%` |
| `ros2_control_node` RSS | `591157 KiB`，约 `577.3 MiB` |
| blocked / swap-in / swap-out | `0 / 0 / 0` |

上述结果说明当前硬件上的资源余量足以支撑已测的 250 Hz 逐轴低速 FJT。它没有覆盖
motion/GPU 同时满负载时的 deadline/jitter，因此不能替代后续联合负载验收。

当前 IO 镜像另做了 30 秒同核观测：CPU14 平均 idle `95.63%`（busy `4.37%`），容器采样约 4.54% CPU、
765.8 MiB；完整一键保持时单点样本为 4.00% CPU、644.5 MiB。PLC/BMS 当前共用 CPU14 是明确接受的临时联调
策略，这些样本不能关闭调度干扰、WCET 或代表性联合负载验收。

## 5. 已知通信现象

EtherCAT 仍偶发成组短丢包。最近一次 FJT 测试中：

- IgH `Lost frames` 从 399 增加到 402；
- slave 2 port 0 的 `CRC` 从 8 增至 11，`PHY` 从 6 增至 8；
- 三次 `4 datagrams TIMED OUT` 中一次位于 FJT 窗口；
- 没有造成 action 失败、驱动掉使能或控制中断。

现有证据把首要嫌疑定位到 `slave 1 port 1 -> slave 2 port 0` 的线缆、接头或相邻
端口。该风险已获准不阻塞当前低速联调，但仍需维护物理连接并完成 30 分钟
CRC/Lost/WC 零增量复测，不能把“未影响本次控制”写成“通信已经修复”。

## 6. 证据入口

- 14 轴运动：`/var/lib/rt-control/validation/run-18-fjt-low-speed-4fc8414/`；
- CANopen 清理与同步容忍：`/var/lib/rt-control/validation/run-17-bq119-bq122-4fc8414/`；
- 首次 IO 候选与性能/TF：`/var/lib/rt-control/validation/run-19-plc-bms-e4fed685/`（候选因关停失败已拒绝）；
- 关停修复复测：`/var/lib/rt-control/validation/run-20-plc-bms-shutdown-fix-d415c0c/`；
- PLC 三路输出：`/var/lib/rt-control/validation/run-21-plc-output-d415c0c/`；
- 完整一键使能/停止：`/var/lib/rt-control/validation/run-22-one-command-d415c0c/`；
- 详细原始记录：[14 轴 FJT 最小低速运动](fjt-14axis-low-speed-commissioning-20260727.md)、
  [CANopen 有序清理与 EtherCAT 同步容忍](canopen-shutdown-sync-tolerance-commissioning-20260727.md)。

开发完成度和联调边界见 [rt-control 开发进度与联调准入](integration-readiness-summary.md)；
同事日常启动只需阅读 [rt-control 一键启动](one-command-start.md)。
