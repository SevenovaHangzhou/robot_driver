---
id: motion-20260819-03
area: motion
title: ELECTRI-102 rolling 前缀与 horizon 正确性闭环
date: 2026-08-19
type: fix
trigger: ELECTRI-102 / D-05 / D-10 / D-11 / D-12 / D-17
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rolling_trajectory_controller/test/test_rolling_correctness.cpp, src/rt_control/rolling_trajectory_controller/test/test_protocol_vectors.cpp]
supersedes: []
related: [ELECTRI-102, RISK-E102-001]
---

## 背景

移植原型存在四个会阻塞真实 rolling 使用的机制缺陷：Prime 可以从 activation hold 跳到任意
合法点；非零 splice 容差会反向改写 R 前 Hermite 曲线；每代复制 t=0 起全部历史会耗尽
capacity；open 虽回报 max horizon，但 buffer 从未执行上限准入。

## 改动

- `beginSession` 锁存 open hold；Prime t=0 positions 必须逐值相等、velocities 必须全零，
  不使用普通 splice 容差。
- 内部点增加 trivially-copyable role，R 同时保存旧轨迹 left state 与新批 right state；
  sampler 在 `t=R` 选择 right、在 `t<R` 沿旧段采样，不对 left→right 风险跳变做伪插值。
- repeated replace 同一 R 始终与原 left-limit 比较，不能逐代累计容差。
- admission 使用同一个 `AdmissionContext{execution,replaceable,minimum_horizon}`；保留可采样 E
  的最后一个旧节点，删除更早历史，capacity 统计 active internal nodes。
- Prime 可用 future 限定为闭区间 500～600 ms；Running max/min horizon 都相对 coherent E，
  commit 在非 RT 校验后再次使用最新 context 复核。
- buffer capacity 合法下界改为 2；新增 `HorizonExceeded=20`，保持 0～19 wire 值不变。
- open 后 Prime 无需等待一次 RT tick，严格限定的 Priming context 采用 E=0，见 E102-D33。

## 验证

- 新正确性组覆盖：1e-12 Prime 偏差拒绝、interior R 前逐采样等价、同 R 容差不可累积、
  300 代/33 ms 更新历史有界、Prime/Running max horizon 和 running minimum horizon。
- rolling package build 通过；完整 CTest 12/12 通过。
- 原协议向量拒绝顺序、session/close、controller callback、STRICT switch、state publisher、
  stop envelope 和 RT zero-allocation 全部回归通过。

以上只在本机 deterministic mock/unit 环境运行，没有启动现场总线、reset、enable 或运动。

## 结论与冻结事实

- F1: Prime generation 不可改变 open 返回的 t=0 hold。
- F2: 每个 accepted candidate 在 `[E,R)` 与其 authoritative head 采样等价。
- F3: capacity 只统计仍可执行/采样的 active internal nodes，不累计 session 历史。
- F4: `max_horizon_ns` 是实际准入，不是只读 capability 字段。

## 遗留

当前仍执行两遍完整 segment 校验、复制 256 点静态图像并线性查段；这些是下一原子性能任务，
不改变本记录已冻结的可观测拒绝语义。
