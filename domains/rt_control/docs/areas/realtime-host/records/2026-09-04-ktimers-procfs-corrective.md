---
id: realtime-host-20260904-03
area: realtime-host
title: ktimers procfs 身份判定纠错
date: 2026-09-04
type: corrective
trigger: ELECTRI-113 target start failure
commits: [fix/electri-113-ktimers-procfs]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - ELECTRI-113
supersedes: [realtime-host-20260904-02#F1]
related: [BQ-142, realtime-host-20260904-02]
---

## 背景

`main@f765900c` 在新工控机完成 exact clean Native build/doctor 后，首次 `start` 在接触总线、
确认口令和创建 runtime 之前 fail closed：真实 `ktimers/14` 为 FIFO1/PSR14，却仍被报告为
污染。原因是前一记录用 `! -L /proc/<tid>/exe` 识别内核线程；本机 procfs 为该内核线程保留
symlink 条目，但跟随目标返回权限错误，所以 `-L` 为真。

## 改动

- 删除 `/proc/<tid>/exe` symlink 判定。
- 新增 `is_kernel_thread()`，读取 `proc_pid_stat(5)` field 9 的十进制 task flags，并要求
  Linux `PF_KTHREAD` 位 `0x00200000` 非零。
- 原三项门禁继续同时成立：名称必须精确为 `ktimers/<rt_cpu>`、policy/priority 必须为
  `SCHED_FIFO/1`、task 必须带 kernel-thread flag。用户态进程仅伪造 comm 仍不能进入白名单。

## 验证

- TDD RED：focused contract 在旧实现上 1 failed，精确命中仍依赖 `/proc/<tid>/exe`。
- GREEN：focused contract 和完整 Native launcher tests 为 63 passed；修改脚本通过 `bash -n`、
  `git diff --check` 和本机 CPU guard smoke。
- `tools/quality_gate.sh` 为 214 passed、门禁覆盖率 83%；本机缺少 ShellCheck，必须由
  GitHub governance CI 补齐。
- Linux v6.8 权威源码 `include/linux/sched.h` 定义 `PF_KTHREAD=0x00200000`。
- 目标机只读复核：`ktimers/14` field 9 为 `69238848`（`0x04208040`），与
  `0x00200000` 按位与结果非零；同次检查 shell 的按位与结果为零。
- 目标机新 release 的失败发生在总线访问前；最终保持 Master0 Idle/Inactive、18 PREOP、
  `/dev/EtherCAT0` 与 CAN socket 无 owner，reset/enable/PLC 写均为 0。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `ktimers/<cpu>` 的内核身份以 `/proc/<tid>/stat` field 9 的 `PF_KTHREAD` 位为权威，
  不以 procfs executable symlink 的存在性或可跟随性判断。
- F2: 白名单仍同时要求 selected CPU 名称匹配和 FIFO1；任一条件不符即 fail closed。

## 遗留

- 该 corrective 合入并取得新 main CI 后，须在同一目标机 exact release 重跑真实 wrapper
  doctor/start；只有门禁通过并实际验证 EtherCAT-OP FIFO79 后，才能关闭 ELECTRI-113。
- 当前切换失败未启动新 runtime。旧 release 自动回退的单次 start 也提前退出且无日志，现场
  保持安全停机；不得将本地测试写成恢复运行或受控使能完成。
