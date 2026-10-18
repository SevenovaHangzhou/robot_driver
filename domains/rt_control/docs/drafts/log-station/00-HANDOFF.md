# Handoff：RT-Control 日志站需求讨论（交接给下一位 AI）

> 写于 2026-10-18。目的：让另一位 AI 在**不重读整段对话**的情况下接着完善本目录下两份产品文档。
> 本目录是**未入库草稿**，不是 `docs/areas` 正式记录；接手者不得把它们当作已裁决事实。

## 0. 先读什么、按什么规则干活

1. 根 `AGENTS.md`、`domains/rt_control/AGENTS.md`、`collaboration-and-commit-standards.md`。
   硬规则：不猜硬件事实；RT 路径禁止分配/锁/IO/日志；不新增 watchdog、不做自动恢复；
   提交/推送只在用户明确要求时；记录格式用 `docs/areas/TEMPLATE.md`。
2. 本目录三份文件：`00-HANDOFF.md`（本文）→ `02-rt-process-monitor-prd.md`（当前正在完善的）
   → `01-rt-log-station-prd.md`（总体方案，含 1 kHz 飞行记录仪与实验台）。
3. 代码基线：分支 `claude/adoring-edison-l6szip`，HEAD `fd07453`，与 `origin/main` 一致。
   工作树除本目录外**没有任何改动**，也没有提交。

## 1. 用户是谁、要什么（用户原话 + 已确认的回答）

用户做机械臂与底盘控制，在台架/工控机 native 方式反复启停 RT-Control 栈联调。

原始诉求（第一条消息）：搭一个"log 站"，用于启动调试、电机控制性能（跟随误差、运动曲线、
力矩变化）；痛点是电机突然掉线/报故障不知原因、PID/前馈/插补改进无法量化、监控本身不能拖累
控制性能；希望做得成熟、以后作为个人项目复用；要求 AI 以产品经理身份补全需求。

用户随后逐条确认：

| 问题 | 用户回答 | 含义 |
| --- | --- | --- |
| 启动调试关心什么 | 只看失败在哪一步；启动时长问题已解决 | 甘特耗时对比降为可选 |
| 力矩进 PDO 难不难 | 好加 | PDO 扩展（0x6077/0x606C/0x603F）提前为总方案 P0 前置项 |
| 掉线形态 | 通信层 stale 和驱动器 Fault 都遇到过 | 黑匣子触发器两层都做 |
| 要调什么 | 驱动器 SDO 参数和 ros 侧前馈都要调 | 实验元数据启动时读一次 SDO + controller 参数（不是每周期） |
| 实时绘图 | 事后绘图/解码即可 | 实时 Web 曲线移出路线图 |
| 落盘格式 | 接受 MCAP | 定稿 |
| play_launch（PyPI）有用吗 | 用户自己判断"作用有点小"，AI 同意 | 不引入；但用户仍想看进程启动时序和占用 |
| 自己做个轻量的？ | 要，"不用调亲和性"，"先用 Python 做起来" | 产生了 `02-rt-process-monitor-prd.md` |
| 用户中途指令 | "只给我一个产品文档继续" | **先不写代码**，只完善文档 |
| 对 02 文档第 2 节的反馈 | "背景问题没有说清我的需求" | 已重写为 N1–N6 需求表，用户尚未再反馈 |

## 2. 两份文档现在的状态

### 02-rt-process-monitor-prd.md（当前焦点，v0.2）

纯标准库 Python、只读、跑在非隔离核的旁观者：解析 ros2 launch 的 `launch.log` 得启动时间线，
读 `/proc` 得每进程每线程资源，产静态 HTML 报告 + CSV/JSONL。第 2 节已按用户要求重写为
"原话 → 需求 N1–N6 → 验收判断"。**等待用户对 N1–N6 的确认**。

文档末尾（第 12 节）和上一轮对话留下三个待用户拍板的问题：
1. 默认进程集和阶段正则表（第 5、7 节）有没有漏；
2. enable_manager 的 `state` 变化要不要进时间线（需要 rclpy 旁路，AI 建议首版不做）；
3. 输出目录默认当前目录还是 `/var/lib/rt-control/validation/<run>/`。

### 01-rt-log-station-prd.md（总体方案，v0.2）

三件事共用一条采集通道：故障黑匣子（1 kHz，仅故障前后窗口）、控制性能实验台（1 kHz，实验期间）、
启动时间线（事件级，不需要 1 kHz）。架构：ros2_control controller 在 RT 线程做定长 memcpy 进预分配
环形缓冲 → housekeeping 核写盘线程 → MCAP。第 7 节记录了用户回答与调整，第 8 节是 play_launch 定位。
**这份暂不动**，等 02 定稿后再回来拆 Issue。

## 3. 已核实的代码事实（接手者可直接引用，带路径）

- V3 配置 `rt_control_machine_config/machines/Alfa_V3/arms.yaml`：双臂 14 轴全 ZeroErr eRob，
  profile `zeroerr_cia402_csp`（`use_slave_pdo_defaults: false`，PDO 可改）；台架 `bench_left_arm.yaml` 7 轴。
  **Ti5 是二代遗留**（`robot_hw_ethercat/config/slaves/ti5_*.yaml`、`variants/alfa_v1.yaml`），不是现役硬件；
  第一版文档曾误把 Ti5 当现役，已修正，接手者不要再犯。
- ZeroErr TPDO 现只有 0x6064 位置、0x60FD digital_inputs、0x6041 状态字；没有速度/力矩/错误码。
- 运行期读不到 0x603F：`patches/igh/0003-preop-only-coe.patch` 让主站拒绝非 PREOP 的 CoE。
- 现有观测链速率：`rt_internal_state_broadcaster` 50 Hz（`rt_control_composer/config/controller_defaults.yaml`）、
  JTC `state_publish_rate: 50.0`、`rt_diagnostics` 1 Hz（`rt_diagnostics_node.cpp`）。
- 相关裁决：BQ-025（不给 controller_manager 打补丁导出 overrun）、BQ-102（IgH `--disable-rt-syslog`）、
  BQ-116（持久故障史应由独立黑匣子需求实现）、BQ-059（非 Fault 掉使能也整组停）。
- 可复用的 RT 安全模式：`rolling_trajectory_controller/rolling_snapshot.hpp`（三槽无锁）、
  `swerve_driver/chassis_runtime.hpp`（`ChassisMailbox` SPSC）。
- 启动阶段可用日志行（02 文档第 5 节的来源）：ecat_icube 补丁 0009/0002/0010 中的
  "EtherCAT startup realtime scope entered/restored"、"All configured EtherCAT slaves are OP with a complete working counter"、
  "EtherCAT cyclic handoff maintenance started"、"EtherCAT activation failed"；bringup `scope_lifecycle.py` 的
  `RT_SCOPE_RT_CHECK`、`RT_SCOPE_HARDWARE_ACTIVE_DISABLED`、`RT_SCOPE_DISABLE`、`RT_SCOPE_SHUTDOWN`；
  V3 runtime launch 节点 `output="both"`（`rt_control_scope_runtime.launch.py`）。
- 宿主：CPU14 隔离（`hostsetup/grub-rt.md`：`isolcpus=14 nohz_full=14`，housekeeping 0-13,15-27）；
  性能阈值 `domains/rt_control/testing/thresholds.yaml`（BQ-134）。
- 现有工具：`tools/rt_perf_capture.py`（单进程 180 s）、`tools/rt_control_thread_affinity.py`（会**修改**亲和性）、
  `tools/rt_cpu_contamination_check.sh`（门禁）、`tools/igh_startup_trace.py`（kprobe，重）。
- 本地环境：Python 3.13，无 `coverage` 模块，jsonschema 4.x（门禁要求 3.2.x），所以
  `tools/quality_gate.sh` 在本容器跑不通；`ruff`、`pytest` 在 `/root/.local/bin`。

## 4. 接手者下一步（按顺序）

1. 请用户确认 02 文档 N1–N6 和第 12 节三个问题；按反馈改文档，**仍不写代码**，直到用户说"开始做"。
2. 用户说开始后：按 02 文档第 10 节归属（platform 区 `tools/rt_process_monitor.py` + `tools/tests/test_rt_process_monitor.py`），
   第二档测试与实现一起交付；写 `docs/areas/platform/records/<date>-rt-process-monitor.md`（TEMPLATE 格式，
   verified=UNVERIFIED/T0），PROGRESS 时间线加一行，platform README 冻结事实表加行。
3. 验证至少跑：`python3 tools/repository_gate.py`、`python3 -m pytest tools/tests -q`、`ruff check`；
   `quality_gate.sh` 若本地缺依赖，如实写"未执行，原因"。
4. 不提交、不推送，除非用户明确要求。本 `drafts/` 目录在正式立项后应转为记录或删除，不要原样入库。

## 5. 不要做的事

- 不要把 02 工具做成会改亲和性/优先级/respawn 的东西；不要让它替代 `rt_control_start`。
- 不要给 02 工具加 rclpy/psutil/pandas 依赖（用户要轻量、标准库）。
- 不要在 01 方案里把"监控"写成任何形式的停机判定或安全功能。
- 不要再把 Ti5、履带 CANopen、`alfa_v1` 当现役硬件。
- 不要用 mock/静态检查的结果写成"实机验证通过"。
