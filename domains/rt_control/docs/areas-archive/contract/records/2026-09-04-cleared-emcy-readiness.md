---
id: contract-20260904-01
area: contract
title: CANopen 零值 EMCY 归一化与 readiness 收口
date: 2026-09-04
type: corrective
trigger: ELECTRI-113 target readiness failure
commits: [fix/electri-113-ktimers-procfs]
env: native
risk: T3
writes: { reset: no, enable: yes, motion: no, plc: no }
verified: PASS
evidence:
  - ELECTRI-113
  - /home/user/rt-control-main/releases/f765900c/log/native/rt-control-20260904-192123.log
supersedes: []
related: [BQ-006, BQ-022, realtime-host-20260904-03, ELECTRI-114]
---

## 背景

目标 Native 启动后，Node 2/3 当前 CiA402 状态均为 `Operation enabled`，但 pinned
`ros2_canopen` 会在 `emcy_state` 中保留最后收到的 error-reset 文本。旧归一化逻辑把任意
非空文本都判为活动 EMCY，导致 CANopen summary 错误并产生 `canopen_unavailable` blocker。

## 改动

- `rt_diagnostics` 只把精确的 pinned 零值文本
  `Emergency message: eec: 0 er: 0 msef: 0 0 0 0 0 ` 视为已清除状态。
- 空 EMCY 继续为正常；任何非零值、字段缺失、字节数量变化、前导零或尾随空格变化仍
  fail closed 为 ERROR。未新增 raw CAN 解析、状态源、接口或自动恢复行为。

## 验证

- TDD RED 在旧实现精确复现 1 个失败；GREEN 的 `rt_diagnostics` 构建通过，5 个 CTest
  入口、83 个实际 gtest case 全部通过，其中 CANopen diagnostics 为 44/44。
- 本地及目标机 `tools/quality_gate.sh` 均为 214 passed、83% policy coverage；目标构建环境
  未继承 `/home/user/ros2_ws`，安装二进制含精确零值 literal。
- 目标启动后 Node 2/3 与 CANopen summary 均为 level 0；使能前 readiness 唯一 blocker 为
  `control_disabled`。唯一一次 `/rt/enable` 成功后，两轮均为 `ready=true`、`status=HEALTHY`、
  `operational_state=READY`、`blockers=[]`，14 轴均 OperationEnabled。

本次实机操作经现场授权，操作人：用户，确认短语入口：`rt_control_native.sh start/enable`。

## 结论与冻结事实

- F1: pinned `ros2_canopen` 的上述完整零值 error-reset 文本表示没有活动 EMCY；归一化层可
  保留原始字段，但不得因此制造 `canopen_unavailable`。
- F2: 除空值和该完整固定文本外，所有非空 EMCY 仍为 ERROR；格式漂移不允许模糊放行。

## 遗留

- PR CI 与 Docker/main 封装门禁尚待完成；本轮按用户要求只运行 Native。
- can0 arbitration-lost 从 2147 增至 2164，但真实 bus-error、TEC/REC、warning/passive/
  bus-off 与 TX drop 均为 0；该物理层优化继续由 ELECTRI-114 跟踪。
