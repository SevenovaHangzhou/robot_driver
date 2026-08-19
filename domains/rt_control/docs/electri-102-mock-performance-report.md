# ELECTRI-102 mock 长稳与桌面性能报告

> 日期：2026-08-19
> 层级：无硬件 T1/T2；不是目标机实时性或硬件运动验收。

## 1. 结论

14 项 deterministic mock 矩阵全部通过；其中连续更新场景按真实墙钟运行 600 秒，在 fake
250 Hz loop 中完成 150,000 个周期。10 Hz knot、30 Hz suffix batch 共发布并接受 18,001 批，
没有 reject、LateReplace、RT allocation、invariant failure 或 late cycle。

该结果足以关闭 ELECTRI-102 的软件／mock 长稳门。它不能用于下调 `replace_lead_ms=16`，
因为测试没有包含目标机 DDS、Motion 进程调度、真实 controller-manager 线程交接和代表性负载。

## 2. 可复现命令

构建时使用 ROS 2 Humble、公开 `robot_interfaces` 功能分支 overlay，以及当前 driver 功能分支：

```bash
source /opt/ros/humble/setup.zsh
source <robot_interfaces-feature-overlay>/setup.zsh

python3 tools/electri_102_mock_gate.py \
  --build-root <colcon-build-root> \
  --output-dir <new-empty-evidence-dir> \
  --soak-seconds 600
```

长稳来源 commit 为 `fa786908b943ddee1331a1d2f6a87ae07e638c2a`，执行前 worktree clean。
后续提交只增加 public producer、文档和依赖锁，不改该测试覆盖的 controller 算法。

## 3. 场景矩阵

| 场景 | 结果 | 关键断言 |
| --- | --- | --- |
| frequency jitter | PASS | 有界 control-period jitter 不产生 clock anomaly。 |
| interruption/recovery | PASS | update timeout 后旧 session 不复活，可 finalize 后 fresh open。 |
| out-of-order | PASS | 随机到达序与 sequence reference model 一致。 |
| duplicate sequence | PASS | duplicate 作为 stale，不改变 pending/accepted trajectory。 |
| late replacement | PASS | 消费 sequence，但不改变 authoritative trajectory。 |
| capacity exhaustion | PASS | prefix+suffix 超过 runtime capacity 时 fail-closed。 |
| low water | PASS | 等于阈值时在 buffer 耗尽前进入同步停车。 |
| prime timeout | PASS | 保持 open 的原始 hold，不接受迟到 prime。 |
| clock anomaly | PASS | 非正／过大 period 使用 nominal step 并锁存原因。 |
| exit realtime | PASS | graceful close 连续停车并拒绝后续 update。 |
| session during disable | PASS | deactivate 使 boot/session 身份失效。 |
| mode switch safety | PASS | verified STRICT switch 与 group-fault 抢占。 |
| RT zero allocation | PASS | 250 Hz update 路径 allocation trap 零命中。 |
| continuous update soak | PASS | 600 秒、10/30/250 Hz 长稳与全部零容忍计数。 |

每个场景独立输出 log 和 JUnit；aggregate 为 14 tests、0 failures、0 errors。

## 4. 长稳量化结果

```json
{
  "seed": 57602,
  "realtime": true,
  "cycles_requested": 150000,
  "cycles_completed": 150000,
  "simulated_duration_ns": 600000000000,
  "wall_duration_ms": 600000,
  "published_batches": 18001,
  "accepted_batches": 18001,
  "rejected_batches": 0,
  "late_replace_count": 0,
  "rt_allocation_count": 0,
  "invariant_failure_count": 0,
  "late_cycle_count": 0,
  "max_point_count": 6,
  "knot_interval_ns": 100000000,
  "batch_rate_hz": 30,
  "update_rate_hz": 250
}
```

活动 point 最大值为 6，证明 capacity 64 没有被 session 历史持续吃满；64 是准入上限，不是
正常复制量或 Motion 必须填满的点数。

## 5. 桌面耗时分位数

| 路径 | p50 | p99 | p99.9 |
| --- | ---: | ---: | ---: |
| fake RT update | 11.381 µs | 24.230 µs | 34.380 µs |
| batch validation | 64.923 µs | 109.345 µs | 127.092 µs |
| direct test-peer callback→RT 可见 | — | — | 23.424 µs |

这些数只描述当前桌面、当前 6 点 hold suffix 和进程内 test peer：

- `callback→RT` 没有走真实跨进程 DDS，不能称为 DDS 端到端延迟；
- validation 分布没有覆盖 64 点最坏 candidate、Motion/视觉负载或目标机 housekeeping 争用；
- fake loop 未获得 SCHED_FIFO，不能代替 CPU14/p99.9 实时调度门；
- 模式切换测试验证结果和失败收敛，但没有形成目标机 STRICT switch 分布。

因此当前动作是保持 `replace_lead_ms=16`。目标机标定至少应分 capacity/batch size 记录
validation p50/p99/p99.9，记录 Motion publish→RT accepted generation 延迟、LateReplace 比例、
STRICT switch 分布，并同时保存负载、CPU affinity 和 DDS 配置。

## 6. Public producer 补充验证

`tools/electri_102_public_mock_harness.py` 将 producer 和 protocol peer 放到两个独立进程，使用
正式 public IDL 与命名 QoS 经 DDS 完成：

```text
FJT_READY -> ROLLING_READY -> open -> prime -> 30 Hz updates
-> REQUEST_STOP -> HOLDING -> FINALIZE -> FJT_READY
```

一次验收运行接受 7 个连续 sequence，最终 session 已删除、mode 回到 FJT_READY；未启动
controller manager、hardware 或 enable endpoint。它证明 Motion 端接口使用正确，但 peer
不是 trajectory controller，因此轨迹语义仍由上述 C++ fake-250 Hz 矩阵负责，两层证据不能
互相替代。

## 7. 未完成证据

- 目标工控机 `ps -To pid,tid,comm,psr` 和 validation callback 的动态 PSR；
- 目标机跨进程 DDS 延迟、generation 交接和 STRICT switch 分布；
- standard GenericSystem 的 CiA402 enable 仿真；
- production envelope 台架停车数据；
- 真实相机／手眼 TF／机械臂闭环精度。

本报告没有访问总线、reset、enable 或运动，也不授权后续硬件动作。
