---
id: release-deploy-20260919-04
area: release-deploy
title: V3 达妙头部非 JTC 位置速度运行链
date: 2026-09-19
type: feature
trigger: 用户要求将误提交到 main 的头部 CAN 方案重新按 v3 分支和三代机电气件推进
commits: [feature/v3-head-can-integration]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - "robot_hw_can + damiao_head_controller：32 tests PASS"
  - "rt_control_bringup head/profile 回归：92 tests PASS"
  - "V3 head/profile 组合与 machine validator：55 tests PASS"
  - "head-only Mock：manager ACTIVE、head_position_controller INACTIVE、硬件接口未 claim 位置命令"
  - "V3 contract、repository_gate、Python/shell syntax、git diff --check PASS"
supersedes: []
related: [ELECTRI-122, ELECTRI-126, BQ-150]
---

## 背景

PR #41 的 base 错用了 main。V3 已冻结 `head_joint` 与 `head_pitch_joint`，且已有
`head_only/full` scope；头部不应重新进入 V2 的旧总装或 JTC 轨迹链。

## 改动

- 迁移 `robot_hw_can` 原生 CAN 协议和 SocketCAN 层到 V3；owner 为 `robot_hw_can`，逻辑轴使用 `head_joint/head_pitch_joint`。
- 增加 `damiao_head_controller`。位置命令使用普通 ForwardCommandController，manager 只占用 enable/reset GPIO 接口，服务为 `/rt/head/enable`、`/rt/head/disable`、`/rt/head/reset_fault`。
- 位置速度模式强制校验 `CTRL_MODE=2`；普通位置帧限制为配置的 `command_rate_hz`（设计默认 250 Hz），电机内部 ACC/DEC/MAX_SPD 负责 T 型加减速。
- 增加 `damiao_parameter_tool`。通过 CAN 读取模式、失能确认、写寄存器 `0x04/0x05/0x06` 并回读校验；只有 `--save` 才发送 `0xAA/0x01` flash 保存，启动过程只读不写。
- 增加 head-only V3 launch、正式模型展开、配置校验和有序停机入口。

## 验证

已验证：`robot_hw_can` 与 `damiao_head_controller` 隔离构建/测试 32 项通过；`rt_control_bringup` 相关构建测试 92 项通过；V3 head/profile 测试 55 项通过；Mock head-only 运行加载 V3 模型、manager active、position controller inactive，位置命令接口未 claim；V3 branch contract、machine validator、repository gate、Python/shell syntax 和 `git diff --check` 通过。

未验证：EtherCAT 完整闭包因本机缺少 `/usr/local/etherlab`/`ETHERCAT_LIB` 未构建；未连接 CAN2，未写参数、保存 flash、使能或运动；机械零位/方向/限位、CAN 看门狗、总线负载与真实 ACC/DEC/MAX_SPD 尚待厂家和现场确认。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: V3 头部使用 `head_joint/head_pitch_joint`，不进入 JTC；目标位置由 ForwardCommandController 转发，头部 manager 独占生命周期和故障服务。
- F2: 达妙位置速度模式通过电机内部 ACC/DEC/MAX_SPD 执行 T 型加减速；普通 CAN 位置帧采用可配置发送频率，默认设计目标 250 Hz。
- F3: ACC/DEC/MAX_SPD 可通过 CAN 写入；默认仅 volatile 写入，`--save` 才允许显式 flash 保存。
- F4: 头部服务固定在 `/rt/head/*`，为未来 full_robot 的整机 `/rt/*` enable_manager 留出唯一命名空间。

## 遗留

完整 V3 runtime 仍受 EtherCAT 开发依赖、头部机械标定、CAN2 物理链路、共享总线负载和 full_robot enable_manager 编排阻塞；实机前必须完成单轴/双轴低速、断线、故障锁存、人工清错和急停验证。
