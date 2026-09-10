# realtime-host — 实时调度与宿主

**范围**：CPU 隔离（isolcpus/nohz_full/CPU14）、PREEMPT_RT 内核、IgH 安装与模块参数、
线程亲和性与污染检查、cyclictest 门禁、CAN 接口命名与 systemd unit、宿主身份校验。
**Owner 包/资产**：`hostsetup/`、`tools/rt_cpu_contamination_check.sh`、
`tools/rt_control_thread_affinity.py`、`tools/canopen_heartbeat_watch.sh`、
`tools/canopen_sdo_archive.sh`。

不属于本区：容器 cpuset 与 compose 变量（→ release-deploy）、总线协议内容
（→ ecat-axes / canopen-chassis）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| HOST-DC-EXPERIMENT-1 | 100 us候选三轮失能启动全OP为15.084至17.684秒，恢复1 ms基线为25.524秒；存在改善迹象但比较条件有差异，默认版本保持1 ms，现场已恢复并失能。 | [realtime-host-20260909-01](records/2026-09-09-electri-97-offset-trigger-100us-candidate.md)#F1-F3 | PASS（本次失能启动）；长期/使能工况UNVERIFIED |
| HOST-STARTUP-RT-2 | 硬件初始化循环发生在正常更新线程创建之前；Native 临时为该调用线程配置 CPU14/FIFO80，退出恢复后再绑定真正的更新线程。IgH 使用报文发送时间配对初始 DC offset。 | [realtime-host-20260908-02](records/2026-09-08-electri-97-dc-offset-and-startup-scope.md)#F1-F3 | PARTIAL（初始化RT已验证；交接由 [ecat-axes-20260909-01](../ecat-axes/records/2026-09-09-electri-97-continuous-handoff.md) 补齐，初始DC等待仍未关闭） |
| HOST-STARTUP-RT-1 | Native 在通信就绪检查前配置 EtherCAT-OP FIFO79、更新线程及 CANopen loop 的 CPU14 亲和性；FIFO79 要求 hrtimer 构建，并通过已加载 Build ID 校验。 | [realtime-host-20260908-01](records/2026-09-08-electri-97-startup-realtime-order.md)#F1-F3 | PARTIAL（Native 三轮启动通过；DC 启动等待及驱动故障仍未解决） |
| HOST-IDENTITY-1 | 当前真实硬件 launcher 锁定 `user` / `localhost` / `6.8.1-1057-realtime`；旧 `ar-Default-string` 身份仅保留为历史证据。 | [realtime-host-20260904-01](records/2026-09-04-current-ipc-identity.md) | ACTIVE |
| HOST-CAN-1 | `localhost` 的 native CAN 入口按 PCI 身份和 `dev_id` 绑定 ZLG PCIe-9140I：L0=`can0`（CANopen）、L1=`can1`（BMS），L2/L3 保持备用；不以偶然的 `canX` 枚举号或 CANable USB 序列号识别。 | [realtime-host-20260821-01](records/2026-08-21-zlg-pcie-can-native-preflight.md), [realtime-host-20260904-01](records/2026-09-04-current-ipc-identity.md) | ACTIVE（native；Docker 发布入口尚未迁移） |
| HOST-SCHED-1 | CPU14 上 controller update 固定 FIFO80；`EtherCAT-OP` 由 Native launcher 设置并复核为 FIFO79；`ktimers/14` 仅在 FIFO1 且 `PF_KTHREAD` 为真时豁免。 | [realtime-host-20260904-02](records/2026-09-04-native-runtime-thread-gates.md)#F2-F3, [realtime-host-20260904-03](records/2026-09-04-ktimers-procfs-corrective.md)#F1-F2, BQ-142 | PASS（T3 Native；PR CI/Docker 待完成） |

## 记录索引（倒序）

- [2026-09-09 ELECTRI-97 初始offset 100us候选与失能实机对照](records/2026-09-09-electri-97-offset-trigger-100us-candidate.md)
- [2026-09-08 ELECTRI-97 DC 偏移配对与初始化线程调度](records/2026-09-08-electri-97-dc-offset-and-startup-scope.md)
- [2026-09-08 Native 启动实时配置前置与 IgH hrtimer 前提](records/2026-09-08-electri-97-startup-realtime-order.md)
- [2026-09-04 ktimers procfs 身份判定纠错](records/2026-09-04-ktimers-procfs-corrective.md)
- [2026-09-04 Native 运行线程门禁修正](records/2026-09-04-native-runtime-thread-gates.md)
- [2026-09-04 当前工控机身份锁迁移](records/2026-09-04-current-ipc-identity.md)
- [2026-08-21 ZLG PCIe-9140I native CAN 预检迁移](records/2026-08-21-zlg-pcie-can-native-preflight.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-009 相关、T-DEV-NATIVE-002 中 CPU14 调度门禁系列
- `docs/host-setup-record.md`（含 30 分钟 cyclictest 记录）、`hostsetup/grub-rt.md`
- 相关 BQ（不完全）：BQ-064、BQ-090、BQ-093、BQ-096、BQ-098（BLOCKED：NVIDIA PCIe
  隔离解除门禁）、BQ-099
