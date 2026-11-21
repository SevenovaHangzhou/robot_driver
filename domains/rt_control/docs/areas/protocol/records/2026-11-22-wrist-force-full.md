---
id: protocol-20261122-02
area: protocol
title: 腕部六维力传感器接入整机主站：共用 CiA402 拓扑，发布原始值与 Robot Model 坐标系
date: 2026-11-22
type: feature
trigger: 用户 2026-11-22 要求研华整机接入两个腕部力传感器并发布原始值（ELECTRI-238）
commits: [feature/wrist-force-full]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [protocol-20261122-01]
---

## 背景

BluePoint P140000100 只有台架用的 `force_bench` 路径：渲染器自己持有主站健康组和 `passive_force_bench` 拓扑，无法与双臂、底盘、升降同在 `ecat_arms` 主站系统里合并。

## 改动

- `force_renderer`：非台架时，主站健康组由第一个同主站模块渲染；传感器向共用 CiA402 拓扑加入身份与保存 PDO 校验；固定按 DC（`assign_activate` 0x0300）；广播器 `frame_id` 用模块里的 Robot Model 坐标系；`calibration_valid` 仍为 false（原始值）。台架路径不变。
- `master_health.responder_count`：同主站模块的传感器也计入从站数。
- `wrist_force_sensors.yaml`：`full_ring_positions` [18, 26]；`frame_id` `left_ft_sensor_link`/`right_ft_sensor_link`。

## 验证

- 新增 `test_wrist_force_full.py`（不渲染第二个主站健康组、拓扑为 CiA402 并含身份与 PDO 校验、坐标系与原始值、DC、主站从站数计入传感器）；台架 force 金样不变；组合器与 hw_ethercat 测试通过（1 项环境相关）。
- 未验证：mock 与实机。

## 结论与冻结事实

- F1: 研华腕部力传感器环位 18 左、26 右；坐标系 `left_ft_sensor_link`/`right_ft_sensor_link`；先发原始值（用户 2026-11-22）。

## 遗留

- 工程单位换算与传感器轴向相对坐标系的核对。
