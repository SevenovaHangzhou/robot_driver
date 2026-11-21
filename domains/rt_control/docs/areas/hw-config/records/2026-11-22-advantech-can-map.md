---
id: hw-config-20261122-01
area: hw-config
title: 研华 ZLG CAN 口默认分配：can2 头部，开机启用 1 Mbit/s；头部零点重设
date: 2026-11-22
type: commissioning
trigger: 用户 2026-11-22 确认头部改接 ZLG can2、要求加入默认配置并授权头部设零（ELECTRI-234）
commits: [feature/advantech-can-map]
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: [Advantech /var/lib/rt-control/zlg-can-backup-20261121T202321Z（原开机脚本与 unit 备份）]
supersedes: []
related: [hw-config-20261105-01]
---

## 背景

研华 Alfa_V3-Advantech-01 的 ZLG PCIe-9140I 四路 CAN 中，can2 未在单机部署中分配、开机不启用。
用户确认：can0 左电池、can1 右电池、can2 头部（达妙 ID 1/2）、can3 外置转向编码器与底盘 IMU。

## 改动

- `units/Alfa_V3-Advantech-01/deployment.yaml`：`can_interfaces` 增加 `head: can2`、`bms_left: can0`、`bms_right: can1`。
- `units/Alfa_V3-Advantech-01/head_zero.yaml`：新增，`zero_rad: 0.0`（电机编码器零点即机械零点）。
- `hostsetup/advantech-zlg-can.sh` / `.service`：此前只存在于目标机的开机脚本纳入仓库，增加 `can2 1000000`。
  目标机已按此安装到 `/usr/local/sbin/rt-control-zlg-can` 与 `rt-control-zlg-can.service`（daemon-reload，未重启服务）。

## 验证

- 头部（T2，未使能）：can2 1 Mbit/s ERROR-ACTIVE；两台失能反馈 ERR=0，PMAX/VMAX/TMAX 回读 12.5/50/10，
  模式 2、Master/CAN ID 与 `head_gimbal.yaml` 一致。用户人工摆到机械零位后对 101/102 发 FE，失能读回约 −0.0002 rad。
- 离线：新增研华 head_only 真实组合测试（`can_interface=can2`）；`tools/quality_gate.sh` 通过。
- 未验证：开机后 can2 自动启用（需重启目标机）、头部断电后零点保持、使能后方向与 Robot Model 一致。

## 结论与冻结事实

- F1: 研华 Alfa_V3-Advantech-01 CAN 分配：can0 左电池 250k、can1 右电池 250k、can2 头部 1M、can3 外置编码器+IMU 500k。
- F2: 研华头部零点为 2026-11-22 现场机械零位处的电机编码器零点（FE）。

## 遗留

- `periph_bms` 仍只读一个 `can_interface`（两地址同总线）；研华一电池一总线的接入未改。
- 头部断电保持与使能方向复测。
