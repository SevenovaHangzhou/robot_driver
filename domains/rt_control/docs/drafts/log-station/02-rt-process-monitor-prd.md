# rt_process_monitor 产品文档（v0.1 草案）

> 定位：RT-Control 日志站的"进程层"子项目。独立于 1 kHz 飞行记录仪，先行交付。
> 状态：需求定稿前讨论稿，未立项、未分配任务号。
> 依据：2026-10-18 对 `robot_driver@fd07453`（= `origin/main`）的源码阅读；未在实机执行。

---

## 1. 一句话

一个纯标准库 Python、只读、跑在 housekeeping 核上的旁观者，回答两个问题：
**"这次启动每个进程/控制器/总线阶段各在什么时刻发生、在哪一步失败"** 和
**"运行期每个进程、每个线程各吃了多少 CPU/内存、跑在哪个核、什么调度策略"**。

## 2. 背景与需求

### 2.1 你的需求（原话 → 需求陈述）

你是做机械臂与底盘控制的工程师，日常在台架和工控机上以 native 方式反复启停整套 RT-Control 栈
（ros2_control_node、十来个 spawner/adapter/外设节点）做联调。这个工具要解决的是你在这个过程中
反复遇到、但现在只能靠翻日志和手敲命令解决的三件事：

| 你的原话 | 需求陈述 | 验收时怎么判断 |
| --- | --- | --- |
| "针对启动调试……看启动时候失败在哪一步" | **N1 启动失败定位**：一次启动没起来时，不翻多个日志文件就能知道：哪个进程最先退出、退出码是什么、退出前 EtherCAT 走到了哪一步（进 RT scope / 全部从站 OP / handoff）、哪些控制器已经激活、哪些还没 | 给一份失败启动的日志，工具输出的时间线第一眼就指出失败进程和它前面最后一个成功阶段 |
| "我确实想看下我各进程启动时序" | **N2 启动时序可视化**：每个进程的启动时刻、各 EtherCAT 阶段和各控制器激活时刻，按时间排成一张图，能看出先后、并行和等待；改了 launch 或配置后能和上一次比 | 一张甘特图，横轴相对时间，能读出"ros2_control_node 起来后 X 秒从站全 OP、再 Y 秒 whole_body_jtc 激活" |
| "……和占用性能这些" | **N3 运行期资源画像**：每个进程和每个线程各占多少 CPU、多少内存、跑在哪个核、是 FIFO 还是普通调度；能分辨"控制环本身"和"旁边那些非实时节点"各吃了多少 | 一张每进程 CPU 曲线、一张 RSS 曲线、一张 ros2_control_node 线程表，能回答"CPU14 上除 update 线程还有谁"和"哪个节点内存在涨" |
| "监控程序本身就比较占用性能……不会因为监控导致影响控制性能" | **N4 零侵入**：工具本身不能碰 RT 核、不能改任何进程的调度，自身开销要能被量化并写进报告 | 工具把自己也采进数据；采样期间 `rt_cpu_contamination_check.sh` 通过 |
| "不用调亲和性""轻量""先用 Python 做起来" | **N5 轻量、只读、零依赖**：标准库 Python，不装包、不编译、不要 root、不接管启动流程；和 play_launch 那种带 Web UI、respawn、亲和性管理的方案划清界线 | `python3 tools/rt_process_monitor.py ...` 直接跑；代码里没有任何写系统的调用 |
| "之后能够作为个人项目复用" | **N6 可复用**：进程匹配和阶段正则是配置不是硬编码；输出是通用 CSV/JSONL，能被 PlotJuggler、pandas 直接读；时间戳同时带 epoch 和 CLOCK_MONOTONIC，将来能和 1 kHz 飞行记录仪对齐 | 换一套 launch 只改 YAML；输出能在 PlotJuggler 打开 |

不在本工具范围、由日志站其它部分承担的：电机为什么掉线/报故障（飞行记录仪 + 0x603F 进 PDO）、
跟随误差与力矩曲线（1 kHz 记录仪）、PID/前馈量化（实验台）。

### 2.2 为什么现有手段不够

| 现有手段 | 对 N1–N6 的差距 |
| --- | --- |
| ros2 launch 的 `launch.log` + 各节点 stdout + Docker 文本日志 | 信息其实都在，但散在多个文件、时间戳格式不一，没有人把它们排成一条时间线（N1、N2 靠人工） |
| `tools/rt_perf_capture.py` | 只采 `ros2_control_node` 一个进程、固定 180 s、依赖 pidstat/mpstat，没有线程级（N3 不满足） |
| `tools/rt_control_thread_affinity.py` | 能枚举线程，但它的职责是**修改**亲和性，是执行器不是观测器，不能常开（违反 N4） |
| `tools/rt_cpu_contamination_check.sh` | 只给 CPU14 一个通过/失败，不给曲线（N3 部分） |
| play_launch | 覆盖 N1–N3 的进程级部分，但自带 respawn、亲和性管理、CAP 提权和 Web UI，与仓库"PID 1 必须是 rt_control_start""不做自动恢复"的裁决冲突，且项目很新（N4、N5 不满足） |

### 2.3 成功标准（一句话）

下一次台架启动失败时，你只跑一条命令就能说出"是哪个进程、在 EtherCAT 的哪一步之后、以什么退出码死的"；
下一次怀疑"是不是旁边节点把机器吃满了"时，你有一张按进程、按线程、按核的曲线，而不是 `top` 截图。

## 3. 目标用户与场景

用户：在台架/工控机上做 native 开发的控制工程师（当前就是你）。

| 场景 | 想得到的答案 | 现在怎么做 | 用本工具 |
| --- | --- | --- | --- |
| S1 启动失败 | 哪个进程以什么退出码先死、之前 EtherCAT 走到哪一步、哪些 controller 已激活 | 翻 launch.log 和各节点 stdout | `timeline` 一条命令出按时间排序的事件表，失败事件高亮 |
| S2 启动顺序核对 | 各进程启动相对时刻、EtherCAT OP 时刻、各 spawner 激活时刻、使能完成时刻 | 手抄 | 甘特图 + JSONL |
| S3 运行期资源画像 | 每进程 CPU/RSS 曲线；ros2_control_node 每线程在哪个核、FIFO 还是 OTHER | pidstat 单进程 | `sample` 持续采样 → CSV → `report` |
| S4 跨次对比 | 改了 launch/参数后启动时序和资源有没有变 | 没有 | 两份输出目录的 diff 表（P1） |
| S5 为 1 kHz 记录仪对齐时基 | 进程/线程事件能和控制数据放到同一时间轴 | 没有 | 每条样本同时写 epoch 和 CLOCK_MONOTONIC |

## 4. 非目标（明确不做）

- 不修改任何进程的亲和性、优先级、nice；不发信号、不重启、不 respawn。
- 不进生产容器默认启动；不替代 `rt_control_start`（PID 1 与停机前失能链不变）。
- 不订阅 ROS 话题、不依赖 rclpy；不采 1 kHz 控制数据（那是飞行记录仪的事）。
- 不做 Web UI、不做实时曲线；只产静态 HTML 报告和数据文件。
- 不需要 root、不需要 CAP_SYS_PTRACE（同用户读 `/proc/<pid>/io` 即可；跨用户时该列留空并标注）。
- 不猜测 CPU 拓扑：隔离核从 `/sys/devices/system/cpu/isolated` 读，为空则不绑核。

## 5. 数据来源（全部只读、零侵入）

| 数据 | 来源 | 精度 | 说明 |
| --- | --- | --- | --- |
| 进程启动/退出事件 | `$ROS_LOG_DIR` 或 `~/.ros/log/<run>/launch.log` 中 `process started with pid [N]`、`process has finished cleanly [pid N]`、`process has died [pid N, exit code K, ...]` | 微秒级 epoch | ros2 launch 原生写出，零改动 |
| 阶段事件 | 同一日志中节点 stdout 行按正则匹配：EtherCAT `startup realtime scope entered/restored`、`All configured EtherCAT slaves are OP with a complete working counter`、`EtherCAT cyclic handoff maintenance started`、`EtherCAT activation failed`、`Timed out waiting ...`；spawner `Loaded <ctrl>`、`Configured and activated <ctrl>`；bringup `RT_SCOPE_RT_CHECK`、`RT_SCOPE_HARDWARE_ACTIVE_DISABLED`、`RT_SCOPE_DISABLE`、`RT_SCOPE_SHUTDOWN`；任意 `[ERROR]`/`[FATAL]` 行 | 微秒级 epoch | 正则表放 YAML，可扩展；默认表随仓库维护 |
| 进程诞生时刻（校验用） | `/proc/<pid>/stat` 第 22 字段 starttime + `/proc/stat` btime | 10 ms（CLK_TCK=100） | 与 launch.log 互相校验，发现代差即报 |
| 进程/线程 CPU | `/proc/<pid>/stat`、`/proc/<pid>/task/<tid>/stat` 的 utime/stime 差分 | 1 tick=10 ms | 采样间隔 1 s 时分辨率 1% |
| 所在核、调度策略、RT 优先级 | 同上第 39/41/40 字段 | 即时 | 用于核对 CPU14 上只有 FIFO80 update 线程 |
| RSS、上下文切换 | `/proc/<pid>/status` VmRSS、voluntary/nonvoluntary_ctxt_switches | KiB / 次 | nonvoluntary 增长是被抢占的信号 |
| IO | `/proc/<pid>/io` read_bytes/write_bytes | 字节 | 同用户可读；否则留空 |
| 机器事实 | 主机名、内核、CLK_TCK、isolated cpus、nproc | 一次 | 写入 run 元数据 |

## 6. 输出

每次运行一个目录（默认 `./rt_process_monitor/<YYYYMMDD-HHMMSS>/`，可指向 `/var/lib/rt-control/validation/<run>/`）：

| 文件 | 内容 | 消费者 |
| --- | --- | --- |
| `run.json` | 主机、内核、CLK_TCK、isolated cpus、匹配模式、采样间隔、开始/结束时刻（epoch + monotonic） | 报告、对比 |
| `process_births.jsonl` | 每个被发现进程：pid、ppid、comm、cmdline、starttime(epoch)、首次发现时刻 | 时间线 |
| `process_metrics.csv` | 每秒每进程每线程一行：ts_epoch、ts_mono、pid、tid、comm、thread_name、cpu、policy、rtprio、cpu_percent、rss_kib、vol_ctxt、nonvol_ctxt、read_bytes、write_bytes | 报告、PlotJuggler、pandas |
| `startup_timeline.jsonl` | 事件：ts、t_rel_s、kind（process_started/exited/stage/error）、source、pid、exit_code、stage_id、message | 报告、对比 |
| `report.html` | 单文件静态 HTML：启动甘特图 + 阶段标记、每进程 CPU% 曲线、每进程 RSS 曲线、汇总表、失败事件表 | 人 |
| `summary.txt` | 终端可读的汇总（和 report 同数据） | 人、记录文档粘贴 |

## 7. CLI（草案）

```bash
# 采样：匹配默认进程集，1 s 间隔，Ctrl-C 或 --duration 结束；自动绑到非隔离核、nice 10
python3 tools/rt_process_monitor.py sample --out <dir> [--interval 1.0] [--duration 600] \
    [--pattern REGEX ...] [--launch-log auto|<path>]

# 时间线：只读 launch.log（+ 可选 births），不需要进程还活着
python3 tools/rt_process_monitor.py timeline --launch-log auto --out <dir> [--stages stages.yaml]

# 报告：从目录里的 csv/jsonl 生成 report.html 与 summary.txt
python3 tools/rt_process_monitor.py report <dir>

# 对比（P1）：两次运行的启动时序与资源汇总差异表
python3 tools/rt_process_monitor.py diff <dir_a> <dir_b>
```

默认进程匹配集（按 cmdline 匹配，兼容 python 节点 comm=python3）：
`ros2_control_node`、`robot_state_publisher`、`rt_diagnostics`、`spawner`、`scope_hardware_start`、
`scope_capabilities`、`control_enable_adapter`、`module_state_adapter`、`position_action_adapter`、
`rt_status_adapter`、`vacuum_adapter`、`plc_node`、`plc_io_modbus`、`bms_node`、`lpms_nav3_can`、
`ultrasonic_node`、`led_strip_node`、`rt_control_arm_console`、`ros2 launch`、`rt_control_start`。

## 8. 报告设计要点

- **一张图一个轴**：甘特图、CPU%、RSS 分三张图，不做双轴。
- 进程颜色按首次出现顺序固定分配 8 个分类色（蓝/橙/青/黄/品红/绿/紫/红），超过 8 个进程的折入"其它"；颜色跟随进程名，筛选不重排色。
- 甘特图：每进程一行，条从 `process started` 到退出或采样结束；阶段事件画竖线并直接标注；`died` 用红色状态标记 + 文字，不只靠颜色。
- 曲线 2 px，不在每个点上标数字；图下附汇总表（avg/peak CPU、peak RSS、线程数、出现过的 cpu 集合与策略）作为表格视图。
- SVG 元素带 `<title>` 作悬停提示；浅色/深色两套 CSS token。
- `ros2_control_node` 的线程表单列：thread_name、cpu 集合、policy、rtprio，用于一眼核对"CPU14 上只有 FIFO80 的 update 线程"，与 `rt_cpu_contamination_check.sh` 的口径一致但不替代它。

## 9. 性能与安全预算（可验收）

| 项 | 目标 | 验收方法 |
| --- | --- | --- |
| 对 CPU14 的影响 | 0（进程不在该核运行，不读写该核任何线程属性） | 采样期间 `rt_cpu_contamination_check.sh --cpu 14 --mode strict` 通过；`taskset -p` 自检写入 run.json |
| 自身 CPU | housekeeping 核上 ≤ 1% 均值（20 进程 × 60 线程、1 s 间隔） | 工具把自己也采进 metrics，报告里单列 |
| 自身内存 | ≤ 40 MiB RSS | 同上 |
| 写盘 | 每秒一次 append + flush，≤ 50 KB/s | 文件大小 / 时长 |
| 时间戳一致性 | launch.log 事件与 /proc starttime 差 ≤ 50 ms | 工具自动校验并在 summary 标注最大偏差 |
| 不写系统 | 代码中没有 `sched_setaffinity(其它pid)`、`kill`、`os.nice(负)`、`/proc` 写 | 单测用假 `/proc` 根目录运行全部采样路径；代码审查 |

## 10. 架构约束与归属

- 归 **platform** 区 `tools/`，与 `rt_perf_capture.py` 并列；指标名沿用其口径（`cpu14_busy_*`、`*_rss_mib`），不 import 其代码。
- 纯标准库（`os`、`re`、`json`、`csv`、`time`、`signal`、`argparse`、`html`），不引入 psutil/pandas/matplotlib；`stages.yaml` 用 PyYAML（仓库门禁已要求）。
- 所有 `/proc`、`/sys`、日志读取函数接受根路径参数，便于用临时目录做单测；解析器全部是"文本入、dict 出"的纯函数。
- 第二档测试（协议/数值）：launch.log 解析（含 died/exit code、comm 含空格的 stat 行）、CPU 百分比差分、starttime→epoch 换算、时间线相对时刻、汇总统计、报告包含的进程名与失败行。采样循环用假 `/proc` 根跑 2 轮。
- 记录：`docs/areas/platform/records/<date>-rt-process-monitor.md`，PROGRESS 时间线一行，platform README 冻结事实表加行；首版 verified=UNVERIFIED（T0），实机采样后补 T2 记录。

## 11. 分阶段

| 阶段 | 内容 | 退出条件 |
| --- | --- | --- |
| P0 | `sample`、`timeline`、`report`、`summary.txt`；默认模式与阶段表；单测；质量门禁 | 本地 pytest 通过；在台架跑一次 native 启动，报告能回答 S1/S2/S3 |
| P1 | `diff`；阶段表外置 YAML；可选把 `ethercat master/slaves` CLI 快照按阶段抓取并存档 | 两次启动 diff 表可读 |
| P2 | 与 1 kHz 飞行记录仪共享时基：把本工具 JSONL/CSV 作为 MCAP 的低速 channel 导入 | 同一 PlotJuggler 会话里能叠加看 |

## 12. 风险与开放问题

- ros2 launch 的 `launch.log` 行格式以 Humble 为准，升级 ROS 发行版需回归解析器测试（固件文本随测试入库）。
- 节点 stdout 若被 `output=log` 而非 `both/screen`，阶段行不会进 launch.log；首版默认按当前 V3 launch 的 `output="both"` 读取，发现缺失时在 summary 标注"阶段源不可用"。
- Docker 内运行时 `/proc` 只能看到容器内进程，宿主侧采样需要在宿主运行工具并指定容器日志目录；首版只承诺 native。
- 你是否需要把 `/diagnostics` 里 enable_manager 的 `state` 变化也进时间线？那需要一个 `ros2 topic echo` 旁路进程，会引入 rclpy 依赖，首版建议不做，由飞行记录仪的事件通道承担。
