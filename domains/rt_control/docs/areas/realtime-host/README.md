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
| HOST-IDENTITY-1 | 当前真实硬件 launcher 锁定 `user` / `localhost` / `6.8.1-1057-realtime`；旧 `ar-Default-string` 身份仅保留为历史证据。 | [realtime-host-20260904-01](records/2026-09-04-current-ipc-identity.md) | ACTIVE |
| HOST-CAN-1 | `localhost` 的 native CAN 入口按 PCI 身份和 `dev_id` 绑定 ZLG PCIe-9140I：L0=`can0`（CANopen）、L1=`can1`（BMS），L2/L3 保持备用；不以偶然的 `canX` 枚举号或 CANable USB 序列号识别。 | [realtime-host-20260821-01](records/2026-08-21-zlg-pcie-can-native-preflight.md), [realtime-host-20260904-01](records/2026-09-04-current-ipc-identity.md) | ACTIVE（native；Docker 发布入口尚未迁移） |

## 记录索引（倒序）

- [2026-09-04 当前工控机身份锁迁移](records/2026-09-04-current-ipc-identity.md)
- [2026-08-21 ZLG PCIe-9140I native CAN 预检迁移](records/2026-08-21-zlg-pcie-can-native-preflight.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-009 相关、T-DEV-NATIVE-002 中 CPU14 调度门禁系列
- `docs/host-setup-record.md`（含 30 分钟 cyclictest 记录）、`hostsetup/grub-rt.md`
- 相关 BQ（不完全）：BQ-064、BQ-090、BQ-093、BQ-096、BQ-098（BLOCKED：NVIDIA PCIe
  隔离解除门禁）、BQ-099
