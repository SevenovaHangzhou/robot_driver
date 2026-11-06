---
id: control-20261105-03
area: control
title: 按精简协议重写 Rolling 轨迹存储与实时交接
date: 2026-11-05
type: behavior
trigger: 用户 2026-11-05 选择 A2，按精简协议重写核心并简化实时交接
commits: [feature/arch-rolling-core]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261105-01, control-20261105-02]
---

## 改动

- `RollingBuffer` 只维护最新接受的轨迹。非实时回调校验、拼接、检查新段的限值与停车可行性，再回读执行边界后提交。删除独立 pending 轨迹、generation 确认协议与缓冲内的第二套会话状态机。
- 删除 `required_initial_horizon_ns`、协议版本、boot/session/client 三组 ID、重复的 sequence 处理、旧关闭/预热停止原因、只写不读的诊断字段，以及仅用于旧测试的配置/提交旁路。公开 `epoch` 与 `sequence` 仍在消息入口校验。
- 连续拼接只有一个轨迹点：首点只检查位置与速度容差，实际从已有轨迹的精确状态接续；删除同时间左右两个拼接点及角色数组。容量计算随之不再为重复点预留位置。
- `RollingSnapshotExchange` 改为单生产者、单消费者的三个预分配槽。生产者写空闲槽后交换中间槽；实时消费者每次接收只做一次原子交换，持有的槽直到下次接收前不可被生产者覆盖。实时采样直接引用槽内轨迹，不复制整条轨迹。
- 保留内部单调会话 epoch 与发布序号边界，用于隔离停机后重启、重新激活时的旧轨迹；不恢复公共 UUID 或 open/close 协议。
- 保留准入时与提交时的时间检查、实时交接的超时边界、连续三次多项式插值、位置/速度/加速度限制、停车可行性、断流与缓冲耗尽制动、生命周期与接管检查。

## 验证

- 隔离构建 `ctrl_rolling_trajectory` 与下游 `integ_bringup` 通过。
- `colcon test --packages-select ctrl_rolling_trajectory integ_bringup` 和 `tools/run_scoped_tests.sh --base HEAD`：PASS；`colcon test-result --verbose` 共 99 项，无错误、失败、跳过，其中 Rolling 包 62 项。
- 新核心行为用例覆盖首点误差不变成命令跳变、拼接前逐毫秒采样不变、重复拼接不累计容差、开始前多个批次接续、长期流的历史压缩、容量/时间/限值拒绝与提交时重新检查边界。
- 百万次发布/消费压力测试：每个快照的会话、代数、时间与全部点一致，消费者持有的槽不被覆盖。控制器更新路径零分配测试继续通过。
- 核心 32 项在 `g++ -fsanitize=address,undefined -fno-omit-frame-pointer` 构建下通过，启用泄漏检查，无 sanitizer 报告。
- `tools/quality_gate.sh`：PASS（工具测试 191 通过、11 按环境跳过）。
- ThreadSanitizer 编译成功，但运行在本机报 `FATAL: ThreadSanitizer: unexpected memory mapping`，没有取得 TSAN 竞态检查结果。未为此修改宿主配置。

本记录不授权使能或运动。

## 遗留

需要实机复测 A2 的连续批次、断流制动及 B2 的双臂＋头部共同启停和模式切换。两个 PR 在取得对应证据前保持 Draft。当前台架仍为 TF 部署，不包含本改动。
