---
id: realtime-host-20260904-02
area: realtime-host
title: Native 运行线程门禁修正
date: 2026-09-04
type: fix
trigger: ELECTRI-113
commits: [fix/electri-113-native-runtime-gates]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - ELECTRI-113
supersedes: []
related: [BQ-120, BQ-129, BQ-142, realtime-host-20260904-01, ELECTRI-114, ELECTRI-115]
---

## 背景

新工控机在 Native 受控使能中暴露两个调度门禁差异：PREEMPT_RT 的 `ktimers/14`
`SCHED_FIFO/1` 被 contamination gate 误判；IgH `EtherCAT-OP` 虽已按 BQ-120 固定到 CPU14，
却仍为 `SCHED_OTHER/0`。用户要求把现场验证过的 timer 豁免同步到 main，并提高
`EtherCAT-OP` 优先级。冻结 IgH 源码没有提供既有 priority 数值，因此 BQ-142 根据
operation FSM 与 FIFO80 主循环的因果关系冻结 FIFO79。

## 改动

- `tools/rt_cpu_contamination_check.sh` 只在名称等于所查 CPU 的 `ktimers/<cpu>`、priority
  精确为 FIFO1 且 `/proc/<tid>/exe` 不存在时，将它识别为内核 timer thread；其它同名或
  不同 priority 线程仍是污染。
- `tools/rt_control_native.sh` 将 controller update 的既有 FIFO80 提升为命名常量，新增
  `EtherCAT-OP` FIFO79 常量。达到 EtherCAT Operation 后，launcher 要求恰好一个
  `EtherCAT-OP`、affinity 精确为 CPU14，再用受控 `chrt` 设置 FIFO79并复读验证。
- 同一调度门禁在 READY 前执行，并在每次 `/rt/enable` 服务调用前重新执行；设置或验证失败
  时不调用 enable。
- 没有修改当前工控机上正在运行的 PID 149664，也没有把现场脏 release 作为提交来源。

## 验证

- TDD RED：main 上三个 focused contracts 为 3 failed，分别命中缺少 `ktimers` 豁免、Hub
  状态分离和 `EtherCAT-OP` FIFO 配置；timer 身份收紧另有 1 次精确 RED。
- GREEN：`python3 -m pytest -q tools/tests/test_rt_control_native.py` 为 63 passed；修改的 shell
  脚本通过 `bash -n` 与 `git diff --check`。
- `tools/quality_gate.sh` 为 214 passed、门禁覆盖率 83%；本机缺少 ShellCheck，必须由
  GitHub governance CI 补齐。
- 现场只读线程快照：controller update=`SCHED_FIFO/80`、`rtcan-master=SCHED_OTHER/0`、
  `EtherCAT-OP=SCHED_OTHER/0`、`ktimers/14=SCHED_FIFO/1`，四者 affinity/PSR 均为 CPU14。
- 冻结 IgH `stable-1.6@2f7f884f` 源码、systemd、modprobe 与 kernel cmdline 均未发现
  `EtherCAT-OP` priority 设置；其 operation loop 明确不应快于发送 RT 线程。
- FIFO79 尚未应用到现场：当前 Native 按用户要求持续运行，不允许为本变更 stop/restart。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `ktimers/<rt_cpu>` 只有在 FIFO1 且确认为无 executable 的内核线程时才进入白名单。
- F2: controller update 保持 FIFO80；`EtherCAT-OP` 固定 FIFO79，必须严格低于 update。
- F3: Native launcher 在 READY 前和每次 enable 前都必须找到、设置并复核唯一
  `EtherCAT-OP`；任一步失败均 fail closed。
- F4: `rtcan-master` 当前保持 SCHED_OTHER 并绑定 CPU14；本任务不改变 BQ-129 的该项裁决。

## 遗留

- 后续维护窗口须在新 main 的全新 Native 会话验证 FIFO79、CPU14 duty、RT throttling、WC、
  lost frames 与 CAN heartbeat gap；当前运行会话不停止、不热改 scheduling。
- can0 `bus-errors` 累计增长已转 ELECTRI-114；PLC `timeout/No route to host` 已转
  ELECTRI-115。本记录不把两项现象判定为无害。
- 本轮不运行 Docker；Docker launcher 和容器调度未修改、未验证。
