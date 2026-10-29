---
id: ecat-axes-20260904-01
area: ecat-axes
title: Hub 0/13 运行态门禁修正
date: 2026-09-04
type: fix
trigger: ELECTRI-113
commits: [fix/electri-113-native-runtime-gates]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - ELECTRI-113
supersedes: []
related: [ecat-axes-20260902-01, release-deploy-20260903-01]
---

## 背景

18 位环在真实 Native Operation 中稳定显示 Hub position 0 为 OP、Hub position 13 为 PREOP。
main 的 READY 门禁此前用一个循环要求两个 Hub 都为 PREOP，导致健康运行态被误判失败；现场
脏 release 已用最小差异验证分离状态可以到达 READY/enable，但该临时修改不能代替主线变更。

## 改动

`tools/rt_control_native.sh::verify_operational_ethercat()` 删除两个 Hub 共用 PREOP 假设，改为
逐位、逐 identity 校验：position 0 必须为 `OP + SG-ECAT-HUB_6`，position 13 必须为
`PREOP + SG-ECAT-HUB_6`。14 个 motion axes 和 position 14/15 两台 X503 的 OP 规则不变。

## 验证

- TDD RED/GREEN 已包含在 ELECTRI-113 Native launcher contracts：旧 main 因缺少分离规则失败，
  实现后 focused test 通过。
- 现场只读证据：单一 Master0、Operation/Active、18 responders、WC 48/48；position 0=OP、
  position 13=PREOP，14 轴与两台 X503 均为 OP。
- 主线新代码尚未在目标机重启验证，因为用户要求当前 PID 149664 保持运行，禁止 stop/restart。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前 18 位运行拓扑的 Hub 状态合同为 position 0=OP、position 13=PREOP；两者不能再由
  同一个预期状态循环校验。
- F2: 本修正不改变 E94 descriptor 中的 responder 顺序、motion/sensor 分类或 X503 fixed-PDO。

## 遗留

新 main 只能在后续获准维护窗口创建新 Native 会话并复验 READY；本轮不停止或替换当前会话，
不运行 Docker。
