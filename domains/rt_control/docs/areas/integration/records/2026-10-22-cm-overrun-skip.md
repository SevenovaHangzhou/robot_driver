---
id: integration-20261022-01
area: integration
title: controller manager 节点改用自带版本，落后时跳过错过的周期（移植 Jazzy overruns.manage）
date: 2026-10-22
type: fix
trigger: ELECTRI-127；台架双臂 scope 每次启动在交接到控制循环时连发约 3000–5000 帧，总线崩溃约 15 s 后恢复，右臂留下 0xA000
commits: [feature/electri-127-cm-overrun-skip]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PASS
evidence: []
supersedes: []
related: [integration-20261019-01]
---

## 现象

台架双臂 scope（左臂 1–7 号、右臂 8–14 号，16 个从站）两次启动都出现同样的过程：

- 16 个从站进入 OP 十几毫秒后，主站报“0 个从站应答”，同一秒内 36–74 个数据报超时、3076–5368 个未匹配；
- 主站重新扫描，14 个驱动器报 AL 0x001A（同步错误），掉到 SAFEOP+ERROR，约 15 s 后重新进入 OP；
- 右 J2、右 J4 留下 0xA000，enable_manager 进入 FAILED。

驱动发送轨迹（`RT_CONTROL_ECAT_TRACE_FILE`）显示：启动循环 766 帧、交接维护 2 帧都按 1 ms 发送，WKC 满 42；
控制循环的前 256 帧在 4.1 ms 内发出，帧间隔 16–20 µs，WKC 全为 0。

## 根因

1. ros2_control 2.53.1（Humble，apt 安装）的 `ros2_control_node` 主循环是“读 → 更新 → 写 → `next += period` → `sleep_until(next)`”，
   没有落后处理。某个周期被挡住 N ms，之后会背靠背补跑 N 个周期，每个周期都发一帧 EtherCAT。
   Humble 分支至今如此；Jazzy 和 rolling 已有 `overruns.manage`（默认开），落后时跳到下一个节拍，计时改用 steady_clock。
2. 分阶段硬件激活走 `ResourceManager::set_component_state()`，它在整个 `on_activate()` 期间持有 `resources_lock_`；
   每个周期的 `ResourceManager::read()`/`write()` 也要拿这把锁。EtherCAT 驱动在 `on_activate()` 里跑几秒启动循环，
   主循环就被挡几秒（台架左臂 3.4–12.3 s，双臂 6.5–6.6 s）。
3. 左臂 scope 没出事，是同核同优先级的 FIFO 调度把补跑“藏”住了：台架的 `early_pin.py` 把更新线程钉到 CPU14，
   和交接维护线程同核、同为 FIFO 80。补跑循环不睡眠，维护线程拿不到 CPU、交不出总线，几千个补跑周期都在“未取得总线、不发帧”的状态下空转掉。
   双臂产物路径不匹配 `early_pin.py`，更新线程留在管理核上，与维护线程并行；维护线程 1 ms 内交出总线，剩余补跑周期全部发帧。
   只改 `early_pin.py` 的匹配（钉回 CPU14）后，双臂启动没有任何连发、计数无新增，验证了这一点。

2026-10-19 12:01 事故后半段（JTC 首周期 8.38 ms 之后连续补跑导致 WKC 异常、0x1208）是同一个主循环缺陷。

## 改动

- `rt_control_bringup` 新增可执行文件 `ros2_control_node`：2.53.1 原文，只把等待下一周期的一步换成
  `rt_control_bringup::next_deadline()`（`include/rt_control_bringup/control_loop_timing.hpp`），
  规则与 Jazzy `sleep_for_periodic_cycle` 相同：落后时跳到原节拍网格上第一个未过去的节拍；参数名、默认值同上游（`overruns.manage`，默认 true）。
- 计时改用 steady_clock（2.53.1 用 system_clock，墙钟跳变也会引起补跑或长睡）。
- 实时线程只做原子计数；非实时 1 s 定时器在有新落后时打一条 WARN（上游在实时线程里打日志，这里不打）。
- 5 个启动文件改用 `rt_control_bringup` 的节点；可执行文件名不变，现有按进程名识别的工具不受影响；
  `tools/rt_control_thread_affinity.py` 同时接受两个包路径。
- EtherCAT 驱动、ICube 补丁和控制器都不改。以后迁到 Jazzy 时删掉这个节点即可。

## 验证

- 单测 6 项：准时、恰好到节拍、落后 2.4 个周期、6.6 s 阻塞、关闭开关保持 Humble 行为、250 Hz 网格。
- 本地 `-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror` 编译通过。
- 台架工控机临时目录编译（Release，BUILD_TESTING=ON），`rt_control_bringup` 51 项测试全部通过。
- mock 栈（域 92，`use_mock_hardware:=true`，左臂台架 scope）对 `ros2_control_node` 进程 SIGSTOP 100 ms 后恢复，
  用 Runtime-LOG 记录控制器的 1 kHz 周期统计：

  | 节点 | 恢复后 1 s 内周期数 | 长周期 | 背靠背周期（< 500 µs） |
  | --- | --- | --- | --- |
  | 上游 2.53.1 | 1102 | 100.5 ms ×1 | 100 |
  | 本节点 | 1002 | 101.0 ms ×1 | 0 |

  本节点在非实时定时器打出一条 WARN：“1 overrun(s) skipped 100 missed cycle(s)”。
- 台架复现条件（双臂 scope，不钉核、无 host，更新线程在 CPU8）用本节点启动：
  - WARN “1 overrun(s) skipped 7093 missed cycle(s)”；
  - 主站发送错误、丢帧计数无新增，15 个从站 OP，enable_manager IDLE（无锁存故障）；
  - 同样条件下，上游节点两次启动都把总线冲垮。
- 台架 600 s A/B（双臂 scope，CPU14，失能、无人操作，Runtime-LOG `tools/ab_jitter.sh` / `ab_analyze.py`）：

  | 组 | 配置 | 标准差 µs | 偏差 p99 / p99.9 / 最大 µs | > 50 µs | 丢帧 / 发送错误 |
  | --- | --- | --- | --- | --- | --- |
  | A | 上游节点，`early_pin.py` 钉核，无 host | 0.543 | 4 / 4 / 9 | 0 | 0 / 0 |
  | B | 上游节点，host 官方钉核 | 0.573 | 4 / 5 / 9 | 0 | 0 / 0 |
  | C | 本节点，host 官方钉核 | 0.517 | 4 / 4 / 9 | 0 | 0 / 0 |

  C 组启动时本节点跳过 6603 个周期（激活期间被锁挡住的部分）。
- C 组有 242 次周期中途约 2 µs 的内核线程打断，全部落在同一个 0.5 s 相位上。同一次启动里，关掉操作台和重启操作台后各测 120 s，都是 0 次，尾部指标不变。
  这与操作台每 0.5 s 轮询 controller_manager 服务、偶尔与更新线程争锁的解释一致，与本节点无关；记为后续观察项。

## 结论与冻结事实

- F1: Humble 2.53.1 `ros2_control_node` 无落后处理；激活期间 `resources_lock_` 挡住 read/write，解锁后补跑。（源码 + 台架发送轨迹）
- F2: 左臂 scope 的“正常”依赖更新线程与维护线程同核同优先级，不是设计保证。（台架对照）
- F3: 新节点落后时跳拍，不补跑（mock：上游补跑 100 个周期，本节点 0 个；台架：不钉核双臂启动跳过 7093 个周期，总线无异常）。
- F4: 台架 600 s A/B 尾部指标与基线相同，无超过 50 µs 的周期。
