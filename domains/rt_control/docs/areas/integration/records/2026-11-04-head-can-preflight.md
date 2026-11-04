---
id: integration-20261104-04
area: integration
title: 头部 CAN 启动预检（自检阶段 A）
date: 2026-11-04
type: behavior
trigger: 用户 2026-11-04 允许头部预检发送 CAN 帧
commits: [feature/arch-head-can-preflight]
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [integration-20261104-02]
---

## 改动

- `hw_can_damiao` 预检：接口 up 且 ERROR-ACTIVE（与 CANopen 共用 `infra_socketcan.link.link_problem`），然后新工具 `damiao_preflight` 向每台电机发一次失能帧，读回反馈帧 ERR。不回应 → `offline`，ERR > 1 → `fault_code`（按设备故障表查）。地址 `{can_id}` 与诊断设备清单一致；未登记的节点来源为 `can/node<id>`。
- 失能帧的等待逻辑抽为 `query_status()`，参数写入工具同用。
- 只发失能帧：电机不会被使能；身份/配置寄存器仍由插件 configure 校验，不在预检重复。

## 风险

- 若另一个不经 RT-Control 总线锁的程序正在驱动头部电机，预检的失能帧会使其失能。

## 验证

本机：预检解析与判定 4 组测试、接口判定 3 例、自检定位 2 例；相关 5 个包 333 项测试通过。`damiao_preflight` 未在真实 CAN 上运行。

本记录不授权使能或运动。

## 遗留

- 台架：头部上电后启动一次（总线启动需授权），确认两台电机回应且无阻断项；断开一台电机 CAN 再启动一次，确认报离线并拒绝启动。
