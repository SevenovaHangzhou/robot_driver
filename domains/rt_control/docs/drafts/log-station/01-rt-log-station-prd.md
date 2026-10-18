# RT-Control 日志站（Log Station）需求核对与产品建议（草案 v0.1）

> 状态：讨论稿，未立项、未分配任务号，不是 `docs/areas` 正式记录。
> 依据：2026-10-18 对 `robot_driver` 分支 `claude/adoring-edison-l6szip` 的源码阅读，
> 该分支 HEAD `fd07453` 与 `origin/main` 一致（0 落后）。硬件事实以 **V3 `Alfa_V3` 配置**
> （`rt_control_machine_config/machines/Alfa_V3/*`）为准：双臂 14 轴全部为 ZeroErr eRob，
> 台架 `bench_left_arm` 为 7 轴 ZeroErr。Ti5、`alfa_v1` variant、履带 CANopen 属于二代遗留，
> 仓库只保留作迁移对照（`tools/diff_legacy.py`、`rt_control_ti5_pdo_prepare.py`），本文不把它们当现役硬件。
> 未在实机上执行任何命令。所有"现状"均可按文中路径复核。

---

## 0. 一句话结论

你要的不是"一个日志站"，而是三件耦合在一起的东西：

1. **飞行记录仪（黑匣子）**：1 kHz 控制环内无锁采样 → 环形缓冲 → 故障触发前后快照落盘，回答"电机为什么掉线/报故障"。
2. **控制性能实验台**：标准化激励（阶跃/正弦/chirp）+ 全速率跟随误差记录 + 离线指标报告，回答"PID/前馈/插补改了之后到底好了多少"。
3. **启动时间线**：从 launch 到 OP、到使能完成的分阶段事件流，回答"启动为什么慢/卡在哪一步"。

三者共用同一条**不进 DDS、不进 RT 线程以外任何锁**的采集通道，仓库里已有现成模式可复用。
BQ-116 已明确裁决"持久故障史应由独立黑匣子需求实现，不能塞回当前实时控制器"，所以这件事在治理上是有路的，不与既有裁决冲突。

---

## 1. 代码核对：现在有什么、缺什么

### 1.1 现有的观测链路（都不满足 1 kHz 级分析）

| 现有组件 | 速率 | 内容 | 对你的需求的差距 |
| --- | --- | --- | --- |
| `rt_internal_state_broadcaster` → `/rt_internal_state_broadcaster/dynamic_joint_states` | 50 Hz（`rt_control_composer/config/controller_defaults.yaml`） | 全部 state interface：position、status_word、al_state、wc_error_count、process_data_age_ms | 50 Hz 是 1 kHz 的 1/20，看不到单周期现象；走 DDS，发布在 CM 非 RT 线程但序列化成本仍在同进程 |
| `rt_diagnostics` → `/diagnostics` | 1 Hz（`rt_diagnostics_node.cpp`） | master link/slaves_responding/WC、每从站 AL+CiA402 状态、CANopen EMCY 归一化 | 只有"当前状态"，没有历史；成功恢复后故障字段会被清掉（BQ-116） |
| JTC `controller_state` | 50 Hz（`state_publish_rate: 50.0`） | reference/feedback/error | 50 Hz，仅位置，不含状态字/时序 |
| rolling controller 公共状态 | 50 Hz（`state_publish_period_ms: 20`） | session 状态、reject code | 同上 |
| `tools/rt_perf_capture.py` | 180 s 窗口，外部 pidstat/mpstat | CPU14 busy、RSS、CAN 错误计数、WC 增量 | 进程级资源占用，不是控制环内部时序 |
| `tools/igh_startup_trace.py` | 一次性 kprobe | IgH 从站配置状态机时序 | 需要 root + 调试符号，重，不适合日常 |
| ecat_icube 补丁 0011 `SendTrace` | 启动阶段 | 发送间隔 1 µs 直方图（16001 bins） | 只在 handoff 阶段导出，不是运行期常驻 |
| Docker/ROS 文本日志 | 事件 | RCLCPP 日志 | 没有结构化、没有和控制数据对齐的时间戳 |

### 1.2 PDO 里到底有什么（决定"力矩变化"能不能看）

| 设备 | TPDO 现状（来源） | 有速度 | 有力矩/电流 | 有错误码 0x603F |
| --- | --- | --- | --- | --- |
| ZeroErr eRob（V3 双臂 14 轴 / 台架 7 轴，profile `zeroerr_cia402_csp`） | 0x6064 位置、0x60FD digital_inputs、0x6041 状态字（`profiles/zeroerr_cia402_csp.yaml`，`use_slave_pdo_defaults: false`，即 PDO 由我们映射、可改） | 否 | 否 | 否 |
| XMC Updown（SW5.11 固定 PDO） | 0x6041、0x603F、0x6078 电流、0x606C 速度、0x6064（`slaves/xmc_updown_sw511.yaml`） | 是 | 是（但未映射 state_interface） | 是（未映射） |
| Kinco 舵轮 8 轴 | 0x6041、0x6064、0x606C、0x6061（`kinco_fd_cyclic.yaml`） | 是 | 否 | 否 |
| 达妙头部（CAN） | position/velocity/effort/fault_code/两路温度/feedback_age（`damiao_system.cpp`） | 是 | 是 | 是 |
| ZeroErr PP 夹爪（草案） | 含 0x606C、0x6077 | 是 | 是 | 否 |

**结论**：
- V3 手臂 14 个 ZeroErr CSP 轴现在**没有**速度、力矩/电流、错误码进 PDO。"力矩变化"在手臂上目前只能靠**位置差分**估算速度、无法得到力矩；这是硬事实缺口，不是软件能绕过的。
- 好消息是 ZeroErr 的 PDO 是我们自己映射的（`use_slave_pdo_defaults: false`），加 0x6077 实际力矩、0x606C 实际速度、0x603F 错误码在协议上可行，每轴 TPDO +8 字节、14 轴 +112 字节。但这是 protocol/hw-config 区的硬件事实变更，需要 BQ 裁决并实机核验 PDO 长度、DC、WC；二代 Ti5 那种固定 PDO 的限制在 V3 上不存在。
- 底盘 Kinco 有速度，没力矩；CSV 模式下"跟随误差"是速度误差，有数据。
- 头部达妙什么都有，可以作为 MVP 的第一个"全字段"验证对象。

### 1.3 故障发生时，软件现在知道什么

- `enable_manager` 每个 RT 周期比较受管轴状态字（V3 `Alfa_V3_enable_policy.yaml`，显式批次）；第一个 Fault/非预期状态触发 `startEmergency()`，记录 `stage/failed_joint/status_word`（`enable_manager_controller.cpp:738-756`）。**只记录状态字，不记录错误码；只记录第一个出错轴，不记录其余轴同周期状态；没有前序历史。**
- 0x603F 错误码当前只在两个地方被读：`topology_preflight.py`（启动前 SDO）和 Updown 固定 PDO（未导出）。运行期**读不到**：IgH 补丁 `0003-preop-only-coe.patch` 让主站拒绝非 PREOP 的 CoE 请求，所以 OP 状态下不能靠 SDO 补读错误码。
- 实际案例已经体现了这个缺口：V3 ZeroErr J3/J4 的 `0x730F`（多圈编码器无电池，2026-09-19 记录）是靠事后用 CLI 读 SDO 才知道原因；二代 Ti5 的 `0x7500`（BQ-115）同理，虽已退役但说明这类问题反复出现。
- EtherCAT 物理层证据（IgH `Lost frames`、slave port CRC/PHY 计数、`datagrams TIMED OUT`）只在 commissioning 文档里人工抄录（`docker-deployment-performance-summary.md` §5），没有程序化采集，更没有和故障时刻对齐。
- 掉线类问题的根因候选至少有四层：线缆/端口（CRC）、DC 同步（0x001A、offset）、PDO watchdog 超时（Kinco 100 ms）、驱动器本地保护（0x603F）。现在没有任何一条数据流能把这四层放在同一条时间轴上。

### 1.4 控制环时序现在知道什么

- BQ-025 裁决：**不**给 controller_manager 打补丁来导出 overrun 计数，不从发布话题推断。所以周期时序只能在**控制器自己的 `update()` 内**测，这恰好是日志站的落点。
- 已有的 RT 安全模式可直接复用：
  - `RollingSnapshotExchange`：三槽无锁交换，`alignas(64)`、`static_assert(is_always_lock_free)`（`rolling_snapshot.hpp`）；
  - `ChassisMailbox<T,N>`：固定容量 SPSC 环（`chassis_runtime.hpp`）；
  - `SendTrace` 直方图（补丁 0011）。
- 现有性能基线（250 Hz 时代，旧机）：CPU14 busy 均值 4.9%、峰值 23%；阈值均值 ≤10%、峰值 ≤40%（`testing/thresholds.yaml`，BQ-134）。日志站的 RT 侧预算必须在这个盘子里。

### 1.5 硬约束（设计必须满足，来自 AGENTS.md / BQ）

| 约束 | 来源 | 对日志站的含义 |
| --- | --- | --- |
| RT 路径禁止分配、锁、阻塞 IO、文件/网络、日志刷屏 | 域 AGENTS §3.6 | 采样只做定长 memcpy 进预分配环；落盘、压缩、序列化全部在非 RT 线程 |
| 不新增 `rt_watchdog`、不恢复心跳/旁路监听 | 域 AGENTS §3.6 | 日志站**只读**，不得参与任何停机判定；"观测到异常"不等于"触发动作" |
| 不给 controller_manager 打补丁导出 overrun | BQ-025 | 时序在控制器 `update()` 内自测 |
| IgH 构建 `--disable-rt-syslog` | BQ-102 | 不能指望内核实时日志，物理层计数走 sysfs/CLI 非 RT 采集 |
| 诊断成功后清理失败字段，历史交给黑匣子 | BQ-116 | 黑匣子是独立需求，合法 |
| 功能包不得反向依赖 bringup；域内包不被其他域依赖 | 域 AGENTS §3.2 | 日志站作为 `src/rt_control/` 下独立包，以 ros2_control controller 形式接入，只 claim state interface |
| 不猜硬件事实 | 根 AGENTS §6 | PDO 扩展（力矩/错误码）必须 BQ 立项，不能在日志站里"顺手"加 |
| CPU14 隔离，housekeeping = 0-13,15-27 | `hostsetup/grub-rt.md` | 写盘线程显式绑到 housekeeping 核，且要过 `rt_cpu_contamination_check.sh` |
| 软件诊断不替代急停/STO | 根 AGENTS §6 | 产品文案里不能出现"日志站保护机器人" |

---

## 2. 需求核对：你说的 vs 我理解的

| 你说的 | 我理解的真实需求 | 需要你确认的偏差 |
| --- | --- | --- |
| "针对启动调试" | 启动**时间线**：每一阶段（master 请求、从站 PREOP/SAFEOP/OP、DC 等待、PDO 配置、controller 加载/激活、sanitize、分批使能）的起止时间和失败点，可以跨多次启动比较 | 你更关心"启动慢"（时长）还是"启动失败在哪一步"（因果）？两者采集方式不同 |
| "电机控制性能：跟随误差、运动曲线、力矩变化" | 每轴每周期的 **指令/实际/误差** 三元组 + 状态字 + 时序；力矩在手臂上需要 ZeroErr PDO 扩展 | 力矩是否接受分两步：先只做位置/速度，PDO 扩展另立 BQ？ |
| "电机突然掉线、报故障，不知道为什么" | 故障前 N 秒全速率快照 + 故障后错误码补读 + 同时刻 EtherCAT 物理层计数 | 你遇到的掉线更多是**AL 状态掉出 OP / WC 不完整**（通信层）还是**CiA402 进 Fault**（驱动器层）？决定 P0 先采哪一层 |
| "PID、前馈、轨迹插补无法量化" | 标准激励 + 可重复实验 + 指标报告（RMS/峰值误差、相位滞后、超调、整定时间、带宽） | 当前 ZeroErr 的位置/速度环在驱动器内部，CSP 模式下 ros 侧只给位置；你要调的"PID"是驱动器参数（SDO）还是 ros 侧外环/前馈？ |
| "监控程序本身占性能、解码影响控制" | 真正的瓶颈是 **RT 线程上的任何非定长操作** 和 **DDS 序列化**；解码/绘图应完全离线或在 housekeeping 核 | 同意把"实时可视化"降级为 P1，P0 只保证"采得全、采得准、不影响 1 kHz"？ |
| "做得成熟，个人项目可复用" | 核心库零 ROS 依赖（schema + 环形缓冲 + 写盘器），ROS controller 只是薄适配；文件格式用开放标准（MCAP），能被 PlotJuggler/Foxglove 直接打开 | 是否接受 MCAP 作为唯一落盘格式？ |

---

## 3. 产品建议：分层架构

```text
┌──────────────── RT 线程（CPU14, FIFO80, 1 kHz）────────────────┐
│ FlightRecorderController::update()                              │
│   读 loaned state/command interface → 定长 Frame → SPSC 环      │
│   自测本周期 t_wake/t_read_done/t_update_done 写入 Frame 头     │
│   成本目标：p99 < 10 µs，零分配、零锁、零系统调用               │
└───────────────────────────────┬─────────────────────────────────┘
                                │ 无锁环（预分配，例如 30 s × 1.2 KB = 36 MB）
┌───────────────────────────────▼─────────────────────────────────┐
│ Drainer 线程（housekeeping 核, SCHED_OTHER）                     │
│   连续模式：按配置抽稀（如 100 Hz）持续写 MCAP                    │
│   触发模式：收到 trigger 时把环里前 N 秒 + 后 M 秒全速率 dump     │
│   触发源：CiA402 状态跳变、AL 掉出 OP、WC 不完整、enable_manager  │
│           进入 EMERGENCY/FAILED、rolling reject、手动服务、实验开始 │
└───────────────────────────────┬─────────────────────────────────┘
                                │ MCAP 文件（+ 边车 JSON 元数据）
┌───────────────────────────────▼─────────────────────────────────┐
│ 非 RT 事件采集器（独立进程/脚本）                                  │
│   启动时间线事件、ethercat CLI 快照（master/slaves/CRC 计数）、     │
│   故障后 PREOP 阶段补读 0x603F/0x1003、/diagnostics、服务调用记录  │
│   全部打 CLOCK_MONOTONIC 时间戳，与 Frame 同时基                   │
└───────────────────────────────┬─────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────┐
│ 离线分析（Python，纯函数，走 release_test_runner 同一套风格）       │
│   跟随误差统计 / 阶跃响应 / 频响 / 时序抖动直方图 / 故障时间线合成  │
│   实验 A/B 对比报告（markdown + png），可进 docs/areas 记录        │
│ 查看器：PlotJuggler/Foxglove 直接开 MCAP；P1 再做网页 viewer        │
└─────────────────────────────────────────────────────────────────┘
```

### 3.1 为什么不是 `ros2 bag` 直接录

- 要 1 kHz 发布就得在 CM 进程里序列化 16 轴 × 十几个字段，`realtime_tools::RealtimePublisher` 虽然把 publish 挪到非 RT 线程，但仍是 DDS 路径、仍在同一进程、仍可能丢；而且还要再起一个 bag 进程订阅。
- 环形缓冲 + 触发 dump 是黑匣子的本质；bag 没有"只在故障时把前 10 秒全速率吐出来"的能力。
- 但 bag/MCAP 作为**文件格式**是对的：离线工具链成熟。所以结论是"自己采，用 MCAP 存"。

### 3.2 为什么做成 ros2_control controller 而不是改硬件插件

- controller 只 claim state interface（可选 claim 不写的 command interface 用于读指令值），不碰总线、不改 PDO，符合"功能包 → bringup"依赖方向和 §3.6 不恢复旁路监听的裁决。
- 和 `enable_manager`、`rt_diagnostics` 同级，不需要进 ecat_icube 补丁链（补丁链已有 13 个，再加会提高上游升级成本）。
- 代价：拿不到硬件插件内部的 WC 原始值、DC 偏移、`ecrt_master_send` 前后时刻。这些要么已经以 state interface 导出（WC、al_state、process_data_age），要么需要**极小**的补丁再导出 2-3 个 sensor state interface（如 `ethercat_domain/dc_offset_ns`、`cycle_send_ns`）。建议 P0 不加补丁，P2 再评估。

### 3.3 数据模型（草案）

**每周期 Frame 头（≈128 B）**
- `seq`、`t_mono_ns`（CM 给的 time）、`t_wake_ns`、`t_read_done_ns`、`t_update_done_ns`（自测）
- `period_actual_ns`、`wc_error_count`、`process_data_age_ms`、`slaves_responding`、`link_up`
- `al_state` 位图（V3 双臂环 18 位：14 轴 + 2 耦合器 + 2 X503；台架按实际从站数）
- `enable_phase`（enable_manager 当前 Phase，需它导出一个 state interface 或由 recorder 订阅其 50 Hz 诊断并打标，P0 可先用后者）
- `experiment_id`（非 RT 侧设置的原子变量，RT 侧只读）

**每轴每周期（手臂 CSP，≈56 B）**
- `cmd_position`、`act_position`、`act_velocity_est`（recorder 不算，离线差分；或 PDO 扩展后用 0x606C）
- `status_word`、`control_word`、`mode_display`（若有）
- 预留：`act_torque`、`error_code`（PDO 扩展后填，之前写 NaN，schema 不变）

**每轴每周期（底盘 Kinco）**
- CSP 转向：`cmd_position`、`act_position`、`encoder_angle`（外置编码器）、两者差
- CSV 驱动：`cmd_velocity`、`act_velocity`、`sent_velocity`、`feedback_sequence/sent_sequence`（已有 state interface）
- `status_word`、`mode_display`、`mode_switch_ready/ack/error`、`command_fresh`

**每轴每周期（达妙头部）**
- 已有全字段：position/velocity/effort/fault_code/两温度/feedback_age/enabled

**事件（非 RT，变长，MCAP 独立 channel）**
- CiA402 状态跳变（每轴，含前后状态字）
- AL 状态跳变、WC 从 complete 变 incomplete 的首个周期
- enable_manager Phase 跳变、service 调用请求/响应（含 stage/failed_joint/status_word）
- rolling reject code、swerve `ControlStatus` 跳变、slip 标志
- 故障后补读：0x603F、0x1003（预定义错误域，有几条读几条）、0x1001
- IgH/sysfs：`Lost frames`、每从站 port CRC/PHY/forwarded 计数（周期抓取，1 Hz 足够）
- 启动阶段事件（见 3.5）

**元数据（边车 JSON）**
- 源码 SHA、镜像 tag、machine/unit/profile/scope、controller 参数快照、PDO 布局哈希、宿主身份（`verify-host.sh` 输出）、实验描述

### 3.4 性能预算（可验收的数字）

| 项 | 估算 | 验收方式 |
| --- | --- | --- |
| 每周期 Frame 大小 | 头 128 B + 16 轴 × 56 B ≈ 1.0–1.2 KB | 编译期 `static_assert(sizeof(Frame) <= ...)` |
| 数据率（全速率） | ≈1.2 MB/s，72 MB/min | 不连续全速率落盘；连续只抽稀 |
| 环形缓冲（30 s 前触发） | ≈36 MB，启动时一次性分配并 mlock | RSS 阈值 700 MiB 内（BQ-134） |
| RT 侧增加耗时 | 读 ~200 个 double + 1 次 memcpy，目标 p99 < 10 µs（1 ms 周期的 1%） | recorder 自己记录每周期自耗时，离线出直方图；对比启用/禁用两次 180 s 窗口的 CPU14 busy |
| 分配/锁 | 0 | `static_assert(is_always_lock_free)`；代码审查；可选 `mallopt`/`LD_PRELOAD` 分配陷阱在 mock 下跑 |
| 写盘线程 | housekeeping 核、SCHED_OTHER、`nice 10`、zstd 1 级 | `rt_cpu_contamination_check.sh --mode strict` 通过；`rt_control_thread_affinity.py` 不需要知道它 |
| cyclictest 门禁 | 不变：> 100 µs 即 FAIL（BQ-064） | 录制中跑 TC-RT-01 |

### 3.5 启动时间线（启动调试专项）

- 定义统一的 `StartupStage` 枚举和一个极简的事件 sink（非 RT，写 stderr 结构化行或 Unix socket），由 launch、`scope_lifecycle.py`、EtherCAT 插件（已有 RCLCPP 日志点，补时间戳即可）、`enable_manager`（Phase 跳变）各自上报。
- 非 RT 采集器在固定检查点抓 `ethercat master`、`ethercat slaves`、`ethercat pdos` 输出和 sysfs 计数，形成"启动快照序列"。
- 离线合成甘特图：每次启动一行，阶段耗时可跨次对比（比如 ELECTRI-97 的 DC offset 100 µs vs 1 ms 实验，就不用再手抄 15.084 s / 25.524 s 了）。
- 启动失败时，自动把最后一个成功阶段 + 失败阶段的 CLI 快照打包。

### 3.6 控制性能实验台（量化专项）

- **实验**是一等公民：`experiment_id` + 描述 + 参数快照（哪一版 PID/前馈/插补、哪一组 SDO）+ 激励定义。
- 标准激励库（复用 `rt_control_arm_console/trajectory.py` 的 S 曲线生成器）：单轴阶跃（小幅）、正弦扫频（chirp）、梯形往返、多轴协调直线。全部经公共 FJT/rolling 接口下发，不绕过限位和首点检查。
- 指标（离线纯函数，带单元测试）：
  - 跟随误差 RMS / 最大值 / 分位数，按轴按实验
  - 指令-实际互相关得到的**纯滞后周期数**（这是看前馈是否起作用最直观的指标）
  - 阶跃：上升时间、超调、2% 整定时间、稳态误差
  - 扫频：幅频/相频、-3 dB 带宽、相位裕度估计
  - 速度纹波（差分后带通能量）、周期抖动直方图
  - 底盘：轮速跟随、转向到位时间、编码器 vs 电机角差分布、slip 标志率
- A/B 报告：两个 `experiment_id` 的指标并排 + 叠图，输出 markdown，可直接粘进 `docs/areas/arm-control` 记录。

### 3.7 可复用性（个人项目）

- 分三层包：
  1. `flight_recorder_core`（纯 C++17，无 ROS）：Frame schema 描述、SPSC 环、MCAP 写入（可直接用 mcap C++ 头文件库）、触发器接口。
  2. `rt_flight_recorder`（ros2_control controller + 非 RT 事件采集器）：薄适配，字段映射通过 YAML 声明（"把 `left_joint1/status_word` 放到轴 7 的 status_word 槽"），不写死机型。
  3. `flight_recorder_tools`（Python）：读 MCAP、指标、报告、时间线合成；与 `tools/release_test_runner.py` 风格一致（纯函数、文本入 dict 出）。
- 字段映射用 YAML + schema 校验，复用 `rt_control_composer` 的 renderer/registry 思路，让"换一台机器/换一套驱动"只改配置。
- 文件格式 MCAP + 自定义 schema（建议 JSON schema 描述的扁平 struct，或 protobuf），PlotJuggler/Foxglove 零成本可视化。

---

## 4. 分阶段路线图（建议）

| 阶段 | 交付 | 验证层级 | 价值 |
| --- | --- | --- | --- |
| **P0 黑匣子 MVP**（先做） | recorder controller（位置/状态字/时序/WC/AL）+ 环形缓冲 + CiA402/AL/WC/enable_manager 触发 dump + MCAP 落盘 + 故障后 PREOP 阶段补读 0x603F/0x1003 的脚本 + 离线"故障时间线合成"工具 | T1 Mock 全功能；T3 不使能实机验证采集与自耗时；T4 带载只读 | 下一次掉线就有前 30 s 全速率数据和错误码 |
| **P0.5 启动时间线** | StartupStage 事件 + CLI 检查点快照 + 甘特图 | T3 | 启动问题可跨次对比 |
| **P1 实验台** | experiment 标记、标准激励库、指标与 A/B 报告 | T4 低速 | PID/前馈/插补可量化 |
| **P1.5 实时查看（可选）** | 复用 arm_console 的 SSE 服务端，在 housekeeping 核从抽稀流推 10–50 Hz 到浏览器 | T1 | 调试时看曲线；不是 P0 |
| **P2 PDO 扩展** | 新 BQ：ZeroErr 加 0x6077/0x606C/0x603F；Updown 导出已有 0x6078/0x606C/0x603F；Kinco 评估 0x6077 | T3→T4 | 手臂力矩可见 |
| **P2.5 物理层对齐** | 1 Hz 抓 IgH sysfs/CLI 计数进事件流；可选补丁导出 DC offset | T3 | 掉线能分到线缆/同步/驱动器三类 |
| **P3 长稳** | 连续抽稀录制 + 按天滚动 + 自动清理；与 TC-LS-01/02 并轨 | T2 | 漂移、温升、累积故障趋势 |

**建议 P0 的第一个落地对象是台架左臂 7 轴（`bench_left_arm` scope）和达妙头部**：前者是你当前的联调主线、有 arm_console 可发激励；后者字段最全，能把"力矩/错误码/温度"这条链先跑通，等 PDO 扩展后手臂直接复用。

---

## 5. 需要你（作为需求方）回答的问题

1. **掉线的主要形态**：过去几次"电机突然掉线"里，`/diagnostics` 最后是 `EtherCAT slave snapshot is stale`（通信层）、`CiA402 fault`（驱动器）还是 `not in the expected state`（非 Fault 掉使能，BQ-059）？有没有保存过的 Docker 日志能给我 2–3 个样本？这决定 P0 触发器的优先级。
2. **力矩的时间表**：是否同意手臂力矩走 P2（独立 BQ + PDO 扩展 + 实机重验），P0 先不等它？
3. **"PID"指什么**：你要调的是驱动器内 SDO 参数（位置环/速度环增益、前馈系数），还是 ros 侧 rolling/JTC 外的外环与插补？前者需要实验台同时记录 SDO 快照；后者只需要记录 controller 参数。
4. **连续录制还是只触发**：是否接受"默认只开环形缓冲 + 触发 dump，连续录制仅在实验/长稳时手动开"？这是把 RT 预算和磁盘控制住的关键。
5. **落盘位置与保留策略**：实机上写哪个目录（现有验证证据在 `/var/lib/rt-control/validation/`）、保留多少 GB、是否要进 Docker volume？容器默认最小权限，这会涉及挂载变更。
6. **格式**：MCAP 是否可以作为唯一格式？（好处：PlotJuggler/Foxglove 直接开，Python 读库成熟；坏处：多一个 C++ 头文件库依赖，需要进 `deps.repos` 或 apt 锁版本。）
7. **包归属**：建议归 integration 区（与 `rt_diagnostics` 同级），离线工具归 platform 区的 `tools/`；你同意吗？这决定记录写在哪个 `docs/areas`。
8. **是否要实时网页曲线**：如果你日常调参离不开"边动边看"，P1.5 要提前；如果能接受"跑完实验看报告"，就后置。

---

## 6. 风险与不做的事

- **不做**任何基于日志的自动停机/复位；日志站是只读观测者（§3.6、BQ-041/059 已有独立的停机所有者）。
- **不做**运行期 SDO 读错误码（主站拒绝非 PREOP CoE）；错误码在 PDO 扩展前只能在失能/PREOP 后补读。
- **不**从 50 Hz 话题推断周期性能（BQ-025 明确禁止）。
- **风险**：环形缓冲 36 MB + mlock 会抬高 RSS，需在 700 MiB 阈值内核算（当前 577 MiB 是旧机 250 Hz 数据，V3 1 kHz 下需重测）。
- **风险**：每周期读 ~200 个 loaned interface 的 `get_value()` 在 Humble 是虚调用+double 拷贝，10 µs 预算要实测，不能靠估。
- **风险**：recorder 作为 controller 加入 `controllers.yaml` 会触碰 composer 金样和 bringup 观察指标（platform-20261014-02 的"bringup 触碰率 ≤30%"）；建议作为 opt-in 控制器，默认不加载。

---

## 7. 2026-10-18 需求方回答与由此产生的调整

| 问题 | 回答 | 对方案的影响 |
| --- | --- | --- |
| 启动调试关心什么 | 只看失败在哪一步；启动时长问题已解决 | 甘特图/跨次耗时对比降为可选。启动方向改为"失败步骤捕获"：进程级用 play_launch（见 §8），EtherCAT 阶段级用少量 StartupStage 事件 + 失败时刻的 `ethercat` CLI 快照。**不需要 1 kHz** |
| 力矩 PDO | 好加 | PDO 扩展（0x6077 力矩、0x606C 速度、**0x603F 错误码**）从 P2 提前为 **P0 前置项**。0x603F 进 PDO 直接解决"OP 下读不到错误码"，是单项价值最高的改动。仍需一条 hw-config/protocol 记录与台架 PDO 长度/WC 复验 |
| 掉线形态 | 通信层 stale 与驱动器 Fault 都遇到过 | 触发器两层都做：AL 掉出 OP / WC 不完整 / process_data_age 超限，以及 CiA402 Fault / 非预期状态。物理层 CRC/Lost frames 按 1 Hz 事件通道采 |
| 调什么 | 驱动器 SDO 参数和 ros 侧前馈都要调 | 实验元数据必须同时记录：驱动器 SDO 参数集（只能在 PREOP 读，**每次启动只读一次**，写进边车 JSON；运行期零成本，不进 RT 周期）和 controller 参数（启动时读一次）。调参工作流固定为：PREOP 改 SDO → OP → 标准激励 → 对比报告 |
| 实时绘图 | 事后绘图/解码即可 | P1.5 实时网页曲线移出路线图 |
| 格式 | 接受 MCAP | 定稿 MCAP + 自定义 schema；查看器用 PlotJuggler/Foxglove |

### 7.1 三个方向是否都需要 1 kHz

| 方向 | 需要 1 kHz？ | 理由 |
| --- | --- | --- |
| 启动失败定位 | **不需要** | 阶段事件是毫秒级、几十个点；失败原因在日志、退出码、CLI 快照里，不在周期数据里 |
| 故障黑匣子 | **需要**（仅故障前后窗口） | 要回答"哪个轴先出事、WC 不完整是否先于 Fault、位置是否先跳变"，14 轴之间的先后差只有 1–3 个周期；50 Hz 下这些先后关系全部丢失。但字段很少（状态字、AL、WC、age、指令/实际位置），≈1 KB/周期，环形缓冲即可，不用连续落盘 |
| 控制性能量化 | **需要**（实验期间全速率） | 前馈起不起作用体现在 1–2 个周期（1–2 ms）的相位滞后上；跟随误差峰值、速度纹波、力矩饱和都是单周期现象。驱动器内环比 1 kHz 更快，但我们只能看到 PDO 速率，所以 1 kHz 已是上限也是必需 |

结论：**一个 1 kHz 采集器，三个消费者**；启动方向是唯一不消费周期数据的，它消费事件流。

## 8. play_launch 的定位

`play_launch`（PyPI 0.18.3，2026-10-17；NEWSLabNTU/play_launch）是 `ros2 launch` 的替代运行器：按节点分目录保存 stdout/stderr/pid/status/cmdline、2 s 间隔采 CPU/内存/IO、Web UI 看日志和 `/diagnostics`、`check` 做 launch 契约 CI、可配 per-node `cpu_affinity`/`nice`、helper 可给 RT 调度能力。

**对本仓库有用的部分（价值有限，非必需）**：原生开发时的启动失败定位（方向 1）。它把"哪个进程、什么时候、以什么退出码死掉、死前最后几行 stderr"自动归档，比现在从 Docker 文本日志里翻便宜很多，可以直接替代我原来打算自己写的进程级启动时间线。`check` 还能把 launch 契约放进 CI。

**不能承担的部分**：它工作在进程级、2 s 粒度，和 1 kHz 控制数据完全是两层；方向 2、3 它帮不上。

**在本仓库使用的边界（必须守住）**：
- 只用于 native 开发入口，不进生产容器。生产 PID 1 必须是 `rt_control_start`（域 AGENTS §3.7），它负责停机前 `/rt/disable`；play_launch 的 SIGTERM 顺序与 ros2 launch 的差异文档未说明，未验证前不能让它接管关停链。
- 关闭 auto-restart/respawn。仓库已裁决不做自动局部恢复，验收记录里"没有 Python respawn"是明确的通过条件。
- 不用它的 `cpu_affinity`/`--sched`，线程亲和和 FIFO 已由 `rt_control_thread_affinity.py` 与 launcher 管理，双头管理会打架并触发 `rt_cpu_contamination_check.sh`。
- `setcap` 给 helper 加 `CAP_SYS_PTRACE`/`CAP_SYS_NICE` 属于扩大宿主权限，按根 AGENTS §6 要有需求和评审；开发机可以，目标机先不做。
- 成熟度风险：项目很新（0.17.0 被 yank，仓库 6 星），锁定版本使用，出问题时随时能退回 `ros2 launch`。
