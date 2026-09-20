---
id: ecat-axes-20260919-01
area: ecat-axes
title: ELECTRI-136 零差 CSP 力矩偏置 ESI 与 PDO 草案
date: 2026-09-19
type: feature
trigger: ELECTRI-136；用户要求 0x60B2/6077/606C 离线配置设计
commits: [feature/ELECTRI-136-gravity-ff]
env: docker
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-152, motion-20260919-01]
---

## 背景

零差 ESI 宣告 60B2/6077/606C 可 PDO 映射，但实机单位、CSP/抱闸时序与 PDO assignment
均未验证。本轮只归档来源并建立不被 runtime 选择的候选配置。

## 改动

- 原始 `ZeroErr Driver_V3.2.0.xml` SHA-256 为 `67f7f1...e52372c`；仓库副本只去除尾随
  空白并补末尾换行，SHA-256 为 `b0c110...e27e15`。ESI identity 是
  `0x5A65726F/0x00029252@0x1`。
- ESI 固定 PDO：1618={60B2}、1A11={606C}、1A12={6074}、1A13={6077}；现有可变
  1600/1A00 各 10 bytes。对象类型与权限按 ESI 登记，不从中推导物理单位或固件行为。
- 方案 A 目标 assignment 为 Rx `[1600,1618]` 12 bytes、Tx `[1A00,1A11,1A13]`
  16 bytes，可选 1A12 后 18 bytes。GenericEcSlave 支持一个 SM 下多个 PDO；但当前普通路径
  会重写 mapping，PreservePdoConfig 同时核对 mapping/assignment，不能形成 assignment-only
  保证，因此 A 仅是首选设计合同、不可启动，未新增上游补丁。
- 方案 B 把 60B2 加入 1600、606C/6077 加入 1A00，形成 12/16-byte 草案。60B2 的
  `command_interface=effort`、factor=TBD、default=0；TBD 使 Generic loader fail closed。
- 首次读数草案不加 60B2，仅在 1A00 增加 606C/6077；分别导出
  `velocity_actual_raw`/`torque_actual_permille`，factor=1。它不依赖新增力矩换算，但仍继承
  原 V3 CSP 的 assign_activate、位置换算和 60FE 默认值阻塞，不能据此宣称现在可启动。
- 三份草案均 `verified:false`，未注册 family、variant 或 alfa_v3 machine runtime。

## 验证

- 4 个离线 pytest 校验 ESI 哈希、identity/CoE/PDO、固定对象、候选字节偶数、60B2 fail-closed
  及原始上行命名和 factor=1。源码审查确认 GenericEcSlave 把全部 rpdo/tpdo 数组交给同一 SM。
- 未连接 EtherCAT、未读写 assignment/mapping/60B2/605E，未验证固件、WC/DC、CSP、抱闸、
  单位或实物故障反应。因此 frontmatter 保持 UNVERIFIED/T0。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: ESI 足以支持离线接口草案，不足以证明 60B2/6077/606C 的实机单位和时序。
- F2: 当前栈支持多 PDO，但不支持 assignment-only 保证；方案 A 保持不可启动，不擅自打上游补丁。
- F3: 原始反馈使用独立 raw/permille 名称和 factor=1，不标成 SI；60B2 factor 继续 TBD/fail-closed。
- F4: 新配置未进入任何 runtime variant，现有生产/CSP/PP profile 未修改。

## 遗留

按 BQ-152 关闭既有 CSP 阻塞后，依次验证只上行读回、shadow、逐轴换算和 active。方案 A
还需受支持的 assignment-only 实现路径及实机 mapping/assignment 回读。
