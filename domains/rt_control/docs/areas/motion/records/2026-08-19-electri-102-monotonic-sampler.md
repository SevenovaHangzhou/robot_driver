---
id: motion-20260819-07
area: motion
title: ELECTRI-102 RT 轨迹采样改单调游标
date: 2026-08-19
type: perf
trigger: ELECTRI-102 / T-12 / D-15
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rolling_trajectory_controller/src/rolling_buffer.cpp, src/rt_control/rolling_trajectory_controller/test/test_rolling_correctness.cpp, src/rt_control/rolling_trajectory_controller/test/test_rt_update.cpp]
supersedes: []
related: [ELECTRI-102]
---

## 改动

保留纯 `sampleTrajectoryImage` 作为非 RT reference。250 Hz update 改用
`MonotonicTrajectoryCursor`：记录 generation、last time 和当前 point index；同一 generation
且时间单调时只向前推进，不再每拍先做 exact O(N) 扫描再做 interval O(N) 扫描。generation
变化、测试中的时间回退或 index 失效时从当前 compact image 起点重置。

同时间 splice pair 通过 `next.time <= sample.time` 一次跨到最右的 right state；t<R 仍以 left
为旧段终点，t>R 从 right 开始。新 generation 先用局部 candidate cursor 采样成功，再复制
image 并原子替换 RT cursor，失败不会破坏当前 cursor。

## 验证

- 64 点、100 ms knot、4 ms 连续采样全程与 reference 逐轴 q/v bit-exact；任一拍最多前移
  1 个 point index，与 point_count 无关。
- 显式覆盖 splice 前/等于/后、时间回退、history-compacted 新 generation，均与 reference
  一致；t=R 明确选 right state。
- `UpdatePathDoesNotAllocate` 保持零命中；rolling 完整 CTest 12/12 通过。

没有访问现场总线、reset、enable 或运动。

## 冻结事实

- F1: 250 Hz Running 采样使用 generation-aware 单调 cursor。
- F2: 非 RT reference sampler 保留，用于差分 oracle。
- F3: splice 的 t=R 采 right，cursor 与 reference 语义一致。
