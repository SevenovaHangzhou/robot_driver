---
id: canopen-chassis-20260916-02
area: canopen-chassis
title: 四舵轮外置编码器 108/27 齿传动确认
date: 2026-09-16
type: corrective
trigger: 用户确认回转齿圈 108 齿、外置编码器小齿轮 27 齿
commits: [work/electri-117-swerve-module-binding]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [ELECTRI-117, ELECTRI-132, BQ-145, canopen-chassis-20260916-01, motion-20260915-01]
---

## 背景

外置舵角编码器 draft 原来把齿圈/小齿轮齿数保留为 `TBD`。厂家图纸中出现过 108/27，
但此前不能确认它属于电机传动还是外置反馈传动。用户现明确确认：回转齿圈 108 齿，
外置编码器传动小齿轮 27 齿。

## 改动

- 四个 FL/FR/RL/RR encoder 节点统一写入 `ring_gear_teeth: 108`、
  `pinion_gear_teeth: 27`；validator 将这两个值作为已确认合同，错误值直接拒绝。
- 轴角换算保持 `encoder_angle × 27/108`，即编码器转 4 圈对应舵轴转 1 圈。
- 机械参数 draft 将 108/27 明确归属外置编码器，并记录用户确认来源。
- 撤回此前由 `35 × 108/27` 推导的电机到舵轴 `140:1`；电机总传动比恢复为 `TBD`，
  需要厂家另行给出。外置编码器齿比不得进入 EtherCAT 电机位置换算。
- profile 整体仍为 `verified: false`。编码器型号、每圈计数、多圈范围、Node ID、EDS、
  方向和安装零偏继续强制 `TBD`。

## 验证

- 新增公式回归：`counts_per_revolution=10000`、raw=10000、108/27 齿时，轴角为 `π/2`；
  6 个纯核心 gtest 通过。
- draft/validator/机械参数定向测试 5 passed；validator 命令直接运行通过。
- 纯核心 ASan+UBSan 6 tests 通过；CANopen/EtherCAT/machine/governance 受影响配置
  回归 151 passed；仓库 quality gate 282 passed、13 skipped、策略覆盖率 83%。
  本机没有 ShellCheck，留待 CI 执行。
- 未连接真实编码器，未读 0x6004/0x6501/0x6502，未核验啮合方向、背隙或安装偏移。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 外置编码器齿圈为 108 齿，小齿轮为 27 齿，几何传动比 `Zr/Zp=4`。
- F2: 代码中的轴角每计数倍率为 `2π × 27 / (counts_per_revolution × 108)`。
- F3: 108/27 属于外置编码器传动，不能用于推导 Kinco 转向电机总减速比。
- F4: 齿数确认只关闭一个机械事实；profile 仍因 EDS、Node ID、分辨率、方向和零偏缺失
  而保持 fail-closed。

## 遗留

提供编码器准确型号/EDS，并只读回 0x6501 每圈计数、0x6502 可分辨圈数和 0x6004 总位置；
逐轮确认 Node ID、计数正方向、安装零位及正反逼近背隙。另向舵轮/电机厂家索取独立的
电机输出到舵轴总传动比，不能复用本次外置编码器 4:1。
