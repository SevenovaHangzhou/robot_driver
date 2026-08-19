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
| 01#F1 | 只有 FIFO80 update 和明确命名 RT bus thread 被放到 CPU14 | [realtime-host-20260819-01](records/2026-08-19-electri-102-validation-affinity.md)#F1 | 有效（T0） |
| 01#F2 | rolling validation 没有专用 affinity，按机制留在 housekeeping | [realtime-host-20260819-01](records/2026-08-19-electri-102-validation-affinity.md)#F2 | 有效（T0） |
| 01#F3 | 目标机当前 TID/PSR 仍需动态复核 | [realtime-host-20260819-01](records/2026-08-19-electri-102-validation-affinity.md)#F3 | 待验证 |

## 记录索引（倒序）

- 2026-08-19 [ELECTRI-102 validation executor 亲和性边界](records/2026-08-19-electri-102-validation-affinity.md) — investigation，PARTIAL（T0）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-009 相关、T-DEV-NATIVE-002 中 CPU14 调度门禁系列
- `docs/host-setup-record.md`（含 30 分钟 cyclictest 记录）、`hostsetup/grub-rt.md`
- 相关 BQ（不完全）：BQ-064、BQ-090、BQ-093、BQ-096、BQ-098（BLOCKED：NVIDIA PCIe
  隔离解除门禁）、BQ-099
