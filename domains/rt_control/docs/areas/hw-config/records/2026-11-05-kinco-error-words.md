---
id: hw-config-20261105-01
area: hw-config
title: Kinco 底盘驱动 TxPDO 0x1A00 追加错误状态字 0x2601/0x2602
date: 2026-11-05
type: commissioning
trigger: 用户 2026-11-04 选方案 A；2026-11-05 授权写入全部 8 台驱动
commits: [feature/arch-kinco-error-words]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: [/var/lib/rt-control/errword-pdo-apply-20261105T123636Z]
supersedes: []
related: []
---

## 背景

Kinco FD 的故障原因在错误状态字 0x2601/0x2602（故障表 `hwcfg_machines/devices/faults/kinco_fd.yaml`），原 TxPDO 不含。

## 改动

- 研华 8 台驱动（ring 0–7）保存的 0x1A00：`6041/6064/606C/6061` → 追加 `2601:00/16`、`2602:00/16`（15 字节）；0x1C12/0x1C13/0x1600 不变；`0x2FF0:01=1` 保存。工具 `~/.cache/remap-errword-pdo.py`（由 `remap-mainline-pdo.py` 只改目标映射），写前守卫：主站空闲、8 PREOP、身份一致、未使能、无故障；写后逐台回读一致。恢复：`sudo python3 ~/.cache/remap-errword-pdo.py restore /var/lib/rt-control/errword-pdo-apply-20261105T123636Z`。
- `kinco_fd_cyclic.yaml` TxPDO 增两项；`KincoCyclicModeSlave` 接受并导出状态接口 `error_word_1`/`error_word_2`；渲染器同步。

## 验证

- 写入后 IgH 需 `ethercat rescan` 才重新读取 PDO（未 rescan 时 IgH 用扫描时缓存的旧映射拒绝配置）。
- `chassis_only` 启动：8 OP、`RT_SCOPE_HARDWARE_ACTIVE_DISABLED`；SDO 读 0x2601/0x2602 全为 0。
- hw_ethercat 73 项测试通过（新增错误字导出用例）。未验证：故障时的取值、诊断解码。

本次实机操作经现场授权，操作人：用户授权 Agent，确认短语入口：“直接做台架修改吧，将新的 0x1A00 映射写入全部 8 个 Kinco 驱动器”。

## 结论与冻结事实

- F1: 研华 Alfa_V3-Advantech-01 的 8 台 Kinco FD 保存的 TxPDO 0x1A00 为 `6041/6064/606C/6061/2601/2602`。

## 遗留

- 错误字按位解码进诊断（依赖底盘驱动进入诊断设备清单，#188）。
