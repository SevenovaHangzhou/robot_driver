# ELECTRI-102 rolling 增量后缀校验等价性论证

> 结论：在本文列出的前置条件成立时，普通 replacement 只校验新 right state 开始的 suffix
> 连续段，与对整个 candidate 做全量 dynamic-envelope 校验得到相同首个 RejectCode。Prime
> 仍从第 0 段全量校验。本文只论证 segment limits/stopping；identity、shape、splice、capacity、
> horizon 与 commit race 仍按完整准入顺序执行。

## 1. 被优化的计算

对每个实际可执行连续段 `(p[i], p[i+1])`，原全量 validator 依次执行：

1. `LimitChecker::checkSegment`：构造 14 轴 cubic Hermite，计算该段 q/v/a extrema，并按轴序
   返回 position、velocity、acceleration 中的首个拒绝；
2. `LimitChecker::checkStoppingViability`：只使用上一步该段的 `SegmentExtrema` 和冻结 envelope，
   计算该段任意状态停车所需的 position envelope。

显式 splice left→right 是风险边界而非时间正长的动态段；它只在 candidate 构造前执行
position/velocity tolerance，不进入以上两步。

## 2. 前缀结论为什么可以复用

### P1 — segment checker 没有跨段输入

`checkSegment(start,end,extrema)` 的全部输入是两个端点、const `DynamicEnvelope` 和输出对象。
函数没有读取前一段 extrema、candidate 总时长、段号、generation 或可变全局状态。

### P2 — stopping checker 没有跨段输入

`checkStoppingViability(extrema,output)` 只读取同一段刚生成的 extrema 与同一冻结 envelope。
duration 是 14 轴在该段内的最大停车时间，不聚合其他段，也不修改 checker 配置。

### P3 — session 中 envelope 不可变

`configureLimits` 只在 `SessionState::kNone` 接受。Priming/Running 中无法更换 limits source、
version 或任一轴数值，因此旧段的验证前提不变。

### P4 — authoritative prefix 的执行函数不变

candidate 从当前 validation head 复制 `[E,R)`；历史裁剪只删除 E 之前不再会执行的节点，并
保留能够采样 E 的最后节点。R 落在旧段内部时，left state 从旧 Hermite 在 R 采样，把旧段
限制到 `[start,R]`。Cubic Hermite 的该限制与原多项式相同；实现测试对 R 前多个非 knot
时刻逐轴比较 q/v，误差门为 `1e-12`。该数值误差远小于 provisional position margin，且
test-only 全量 oracle 会继续比较优化路径与完整 validator。

### P5 — 待提交 head 与 base generation 不会暗换

prepare 记录 `validation_base_generation`；commit 要求当前 validation head generation 未变、
candidate generation 正好为 base+1，并重新检查最新 replaceable boundary 和 minimum horizon。
因此校验完成后不能把结论套到另一个 prefix。

由 P1～P5，旧连续段的输入函数与 envelope 均不变，其 direct/stopping 结果也不变。普通更新
只可能引入：splice right state、right→下一 knot 以及后续 suffix 段，所以从 right state 的
index 开始按原顺序检查即可。

## 3. 实现边界与 fail-closed 条件

- `PreparedSubmission.first_validated_segment_index` 由内部 candidate 构造确定，不能来自 wire。
- `validated_segment_count` 仅用于测试/证据，不参与接受逻辑。
- internal structure 仍在每次校验前检查；非法 role、孤立 left/right、非标记重复时间都会拒绝。
- suffix 至少两个传输点，因此 right state 总有一条实际新连续段可校验。
- Prime、session fresh start 或无法证明 base generation 的路径从 index 0 全量校验。
- 若未来 checker 引入跨段 jerk、energy、累计时间或全局 extrema，本论证立即失效；修改者必须
  恢复全量校验或给出新证明和差分 oracle。

## 4. 测试证据

`test_rolling_correctness.cpp` 的 `IncrementalValidationChecksOnlyTheChangedSuffix` 构造长前缀与
两点短 suffix：生产路径验证段数严格为 `suffix_count-1=1`，test peer 对同一 candidate 从
index 0 全量校验并得到同一 `None`。随后只破坏新 suffix 末点，增量与全量 oracle 都返回
`PositionLimit`，而增量检查段数更少。

此外，完整 protocol vectors、Hermite/extrema/stop、history/splice、controller/RT 生命周期和
状态发布套件保持 12/12 通过。全量 oracle 只通过 friend test peer 可达，不存在 production
运行时“增量后再全量”的双倍开销。
