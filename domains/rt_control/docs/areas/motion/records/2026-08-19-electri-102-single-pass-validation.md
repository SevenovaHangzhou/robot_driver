---
id: motion-20260819-04
area: motion
title: ELECTRI-102 删除 rolling 重复 segment 校验
date: 2026-08-19
type: perf
trigger: ELECTRI-102 / T-09 / D-13
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rolling_trajectory_controller/src/rolling_buffer.cpp, src/rt_control/rolling_trajectory_controller/test/test_protocol_vectors.cpp]
supersedes: []
related: [ELECTRI-102]
---

## 背景与等价性

`validateCandidate` 原先有两个同序循环。第一遍只调用
`checkSegment(points[i], points[i+1], extrema)` 并在首个错误返回；第二遍先对完全相同的段
调用同一个函数、按同样规则返回，然后才增加 `checkStoppingViability(extrema)`。第一遍没有
写入跨段状态，也没有产生第二遍拿不到的拒绝信息，因此成功 candidate 的 direct segment
检查被精确执行两次。

## 改动与验证

删除第一遍循环；保留第二遍的 direct check、原拒绝先后和 stopping check。splice left/right
零时长风险边界仍只做 splice tolerance，不被当作动态段。

- 协议向量、插值/限值与 prefix 正确性 focused CTest：3/3 通过。
- rolling 完整 CTest：12/12 通过。
- 无 schema、参数、拒绝枚举、状态或 writer 生命周期变化。

没有访问现场总线、reset、enable 或运动。

## 结论与冻结事实

- F1: 每个实际可执行连续段在当前全量 validator 中只执行一次 direct segment check。
- F2: 首个可观测 RejectCode 与删除前保持一致。

## 遗留

本记录只闭环 T-09；增量后缀校验必须另行证明前缀验证结论不依赖跨段全局量。
