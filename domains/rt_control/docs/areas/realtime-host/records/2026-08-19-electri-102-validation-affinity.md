---
id: realtime-host-20260819-01
area: realtime-host
title: ELECTRI-102 validation executor 亲和性边界
date: 2026-08-19
type: investigation
trigger: ELECTRI-102 / T-13 / E102-D16
commits: [功能/视觉伺服-ELECTRI-102]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [tools/rt_control_thread_affinity.py, tools/rt_control_native.sh, hostsetup/grub-rt.md, domains/rt_control/docs/native-development-workflow.md]
supersedes: []
related: [ELECTRI-102, BQ-140]
---

## 背景

Rolling batch callback 在普通 controller executor 中执行 prepare/validation；如果该线程与
250 Hz update 共用隔离核，长 suffix validation 可能污染 RT 周期。T-13 要确认现有启动策略，
而不是在没有测量前新增 validation 专用线程。

## 改动

无代码改动。检查当前 native 启动和线程 pin 机制：完整进程树先由 `taskset` 放到
housekeeping CPUs；`rt_control_thread_affinity.py` 要求恰好一个 SCHED_FIFO/80 update TID，
并只把它和明确命名的 `rtcan-master` 放到 CPU14。随后工具逐 TID 将同一 ros2_control_node
的所有其他线程重设为 housekeeping，并逐项复核 affinity。EtherCAT-OP 由独立 host 策略管理。

Rolling 没有创建 validation worker 或设置 callback affinity，因此 subscription/service/DDS
executor 线程属于“其他线程”，按上述机制留在 housekeeping。

## 验证

- 源码静态检查确认 pin 的选择条件是唯一 FIFO80 update 和明确命名的 bus thread；没有按
  controller、callback 或 validation 名称把普通 executor pin 到 CPU14。
- `verify_affinity()` 对所有剩余 ros2_control_node TID 要求 affinity 精确等于 housekeeping，
  不是只打印日志。
- 历史目标机记录证明该工具曾将 FIFO80 update 与 `rtcan-master` 放到 CPU14；本次没有启动
  当前 ELECTRI-102 目标栈，也没有采新的 `ps -To pid,tid,comm,psr`，故只能标 PARTIAL。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前启动机制只把唯一 FIFO80 update 和明确命名的 RT bus thread 放到 CPU14。
- F2: Rolling validation 没有专用 affinity，按机制在普通 executor／housekeeping CPUs 运行。
- F3: 源码机制不能替代当前目标进程的 TID/PSR 动态证据，目标联调仍须采样确认。

## 遗留

在目标机无运动或另行授权窗口，保存 `ps -To pid,tid,comm,cls,rtprio,psr`、各 TID affinity、
代表性 30 Hz batch validation 和 250 Hz update 的同窗证据；若 validation 实际落到 CPU14，
停止 timing 标定并先修启动／executor affinity。
