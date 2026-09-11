---
id: realtime-host-20260908-01
area: realtime-host
title: Native 启动实时配置前置与 IgH hrtimer 前提
date: 2026-09-08
type: fix
trigger: ELECTRI-97 / 用户要求修正启动实时配置顺序并实机复测
commits: [fix/electri-97-startup-realtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PARTIAL
evidence: [/home/kkozia/research_reviews/electri97-gen2-20260908/startup-rt-hrtimer-v2]
supersedes: []
related: [BQ-120, BQ-129, BQ-142, realtime-host-20260904-02]
---

## 背景

二代机原生启动在全部 OP 后才设置 EtherCAT-OP FIFO79，并仅在调用 `/rt/enable`
前绑定控制更新线程。因此不使能启动时，实时配置晚于通信就绪检查。修正顺序后的首次
实机测试又暴露了已有 IgH 构建未启用 hrtimer：忙碌 FSM 路径只有 `schedule()`，
不能在 FIFO 调度下保证给较低优先级线程让出 CPU。

## 改动

- `tools/rt_control_native.sh` 在启动进程后，有界等待 EtherCAT-OP 出现并设置 FIFO79；
  随后等待唯一 FIFO80 更新线程，连同 `rtcan-master` 绑定 CPU14，再验证服务和 OP。
- 进程尚未完成 exec 时，依据已启动 PID 的存活状态等待身份就绪；配置失败仍进入
  原有失败退出流程，不调用使能或故障复位。
- `tools/rt_control_thread_affinity.py` 区分线程尚未创建与匹配不唯一；前者在截止时间内
  等待，后者及亲和性设置错误立即失败。
- `hostsetup/igh-install.sh` 启用 `--enable-hrtimer` 并记录 `IGH_HRTIMER=1`。
  Native/host 门禁拒绝缺少该前提的模块；Native 进一步比较已加载与磁盘模块的 Build ID
  并检查 hrtimer 符号。实测本次编译选项改变不改变 srcversion，不能只据 srcversion 判断。
- 目标机使用相同 IgH commit `2f7f884f1c7d377c02a7d627eb06512126a0e50e` 和已有
  PreservePdoConfig 补丁重建三个模块；控制周期仍为 4 ms，PDO/DC 参数、初始 offset 算法
  和应用时基未改变。

## 验证

离线：先复现启动顺序、等待线程创建、模块构建前提和 exec 窗口的失败，再实现修正。
`tools/quality_gate.sh` 当前通过 223 项测试，规定的工具门禁覆盖率为 83%；相关 72 项
测试通过。本机未安装 ShellCheck；Bash 语法检查通过，CI 的 ShellCheck 尚未执行。

目标机 `6.8.1-1057-realtime` 编译通过，HRTIMER 模块 Build ID 为
`2d3cb97e9603f49c2a7d497da3dfa601f2c79a75`。原模块和脚本备份在
`/home/user/rt-control-backups/electri97-startup-order-20260908`。

三轮有效对照均通过不使能启动验收，后续 15 秒状态采样保持 OP/WKC 完整：

| 轮次 | 首次全部 OP | DC wait 警告 | UNMATCHED | TIMED OUT | lost frames 增量 | 稳定后 8 节点 DC 抽查最大绝对值 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 53.964120197 s | 6 | 0 | 0 | 0 | 2231 ns |
| 2 | 56.244127086 s | 7 | 1 | 0 | 0 | 3545 ns |
| 3 | 42.564133506 s | 5 | 1 | 0 | 0 | 1380 ns |

三轮都仍有两个力传感器短暂同步错误，不能据随后恢复 OP 判定瞬态已解决。模块重载
重置了累计计数，因此只比较各轮增量，不把重载前后的总数下降当作改进。此前仅提前
FIFO、未开启 hrtimer 的中间实验失败；另一次进程 exec 竞态在激活总线前退出，不计入
上表。三轮是小样本暖启动，不是长期可靠性或 p95 验收。

最终 RT-Control stopped、EtherCAT Idle/Inactive、18 个从站全 PREOP；14 个运动轴
0x6041 读回均为 Operation enabled=false。位置 2、3、8、9 的 Ti5 仍为故障态，
停机门禁返回 fault_requires_reset / UNCLEAN_SHUTDOWN；未执行故障复位，不能报告
干净停机。现场旧的应用二进制未改变，只有宿主脚本和 IgH hrtimer 模块更新。

本次实机操作经用户在会话中授权，使用 Native `start`/`stop` 确认入口；未执行使能、
运动或驱动故障复位。该记录不授权后续使能或运动。

## 结论与冻结事实

- F1: Native 通信就绪检查必须先完成启动实时配置，不能依赖后续使能入口才绑定线程。
- F2: FIFO79 EtherCAT-OP 需要经过验证的 hrtimer 阻塞等待构建；源提交和 srcversion
  相同不足以证明编译选项相同，加载身份以 Build ID 和功能符号交叉验证。
- F3: 启动调度修正不等于 DC 初始同步时长问题已经修复；首次 OP、持续状态、同步误差
  和驱动故障必须分别报告。

## 遗留

Native 三轮对照与最终失能读回已完成。初始 offset、混合时基、启动循环向正常
控制循环交接的连续性和 Ti5 锁存故障仍需分别调查；不放宽同步或停机门禁。未重建或
部署 Docker 镜像，未进行运动或长时间稳定性验收。
