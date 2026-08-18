---
id: motion-20260819-06
area: motion
title: ELECTRI-102 snapshot 只复制有效轨迹节点
date: 2026-08-19
type: perf
trigger: ELECTRI-102 / T-11 / D-14
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rolling_trajectory_controller/include/rolling_trajectory_controller/rolling_types.hpp, src/rt_control/rolling_trajectory_controller/test/test_rolling_snapshot.cpp]
supersedes: []
related: [ELECTRI-102]
---

## 改动

保留协议 ceiling 256、三个固定 snapshot slot 与固定 RT active image；没有引入 vector、堆分配
或共享所有权。splice role 从 `JointPoint` 的 8-byte 对齐膨胀中拆为 image 内独立 1-byte 数组，
`JointPoint` 恢复为 232 bytes。`copyTrajectoryImageEffective` 只复制 `point_count` 个 point/role
及 24-byte 元数据，尾部允许保留旧字节，读路径继续以 point_count 为唯一边界。

publish、buffer handoff 和 RT generation 接管均使用有效复制；生命周期 reset 仍可在非周期路径
清空固定对象。无效 point_count fail-closed，不发布半写 slot。

## 验证

- capacity 64 有效复制：`64 * (232 + 1) + 24 = 14,936 bytes`，严格小于 15 KiB。
- 正常 10 节点有效复制：2,354 bytes。
- 单测用 sentinel 证明 point_count 之后的 point/role 未被覆盖。
- 三槽百万次 publication/lease/ABA stress、allocation trap、controller RT update 全部通过。
- rolling 完整 CTest：12/12 通过。

没有访问现场总线、reset、enable 或运动。

## 冻结事实

- F1: 256 是 wire/storage ceiling，不等于每次跨线程复制量。
- F2: capacity 64 的有效 image copy 上界为 14,936 bytes。
- F3: 未使用尾部内容无语义，任何读路径不得越过 point_count。
