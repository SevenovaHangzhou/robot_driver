# ELECTRI-102 需求澄清与风险裁决日志

> 状态：软件设计裁决已闭环；功能代码、目标机证据和硬件运动授权尚未完成。
> 日期：2026-08-19
> 范围：视觉伺服使用的 14 轴 rolling trajectory 控制路径。

本文是本期唯一的详细裁决登记。它同时记录用户明确裁决和用户授权 Codex 自主完成的
剩余裁决，并为每个问题保留候选方案、推荐项、最终采用项、因果影响和验证门。它用于
约束后续执行计划、代码、测试和 Motion 联调文档；不能替代 mock、性能标定、台架实测
或正式跨域契约发布。

## 1. 控制边界

- Rolling 五个端点是跨域公共接口，权威 schema 落在 `robot_interfaces`，并按 endpoint
  提供方归档：Motion 发布的 update batch/point 属 Motion 包，RT-Control 提供的三个
  service 与 state 属 RT-Control 包。在 `robot_driver` 功能可供 Motion 测试前，不提交
  接口仓库 PR。
- Rolling 始终接收固定顺序的完整 14 轴：
  `right_joint1..6,left_joint1..6,turn,updown`。
- RT-Control 不增加 `controlled_arm` 或 Turn/Updown 业务使能字段；当前实际移动哪些轴由
  Motion 决定。一次一臂是本期 Motion 使用策略，不是 RT 协议能力限制。
- Rolling 与 FJT/JTC 互斥持有 command writer。本文的 provisional 速度、加速度、停车和
  splice 参数只约束 Rolling，不修改正常 FJT/JTC 的既有配置与行为。
- FJT 切换前的 cancel 及 Action 终态等待由 Motion 负责；`set_mode` 不创建 Action client。
  RT-Control 负责源稳定性/接管校验和一次有界 STRICT controller switch。

## 2. Motion 生产节奏与缓冲参数

```yaml
motion:
  batch_publish_rate_hz: 30
  knot_period_ms: 100
  planned_horizon_ms: 500

rolling_trajectory_controller:
  buffer_capacity: 64
  required_initial_horizon_ms: 500
  max_horizon_ms: 600
  update_timeout_ms: 200
  replace_lead_ms: 16
  state_publish_period_ms: 20
  prime_timeout_ms: 100
  stopping_guard:
    one_cycle_detection_ms: 4
    stop_time_growth_ms: 4
    non_rt_to_rt_visibility_ms: 4
    period_quantization_ms: 4
```

Motion 每 100 ms 生成一个轨迹 knot，但以 30 Hz 重算并发布 suffix。首批典型时间点为
`0/100/200/300/400/500 ms`。四项 stopping guard 必须分别配置和报告，不得与同为
16 ms 初值的 `replace_lead_ms` 混为一项。所有参数在 controller configure 时冻结；修改
YAML 后通过重新 configure/restart 生效，不支持活动 session 热改。

`max_horizon_ms` 的准入语义是候选最后一点的 session time 减去本次验证使用的 coherent
`execution_time` 快照，不是单个 batch/suffix 自身的跨度。Prime 的 execution time 为零，
最后一点必须处于 `[required_initial_horizon_ms,max_horizon_ms]`；运行中若候选可用未来超过
600 ms，则以独立 `HorizonExceeded` RejectCode 拒绝。原型只在 open response 回报
`max_horizon_ns` 而不执行校验的行为必须修复。Motion 正常从 `replace_from=R` 生成约
500 ms suffix；R 领先执行游标约 16 ms 时，接受后的 available horizon 约为 516 ms。

原型将 `replace_from` 之前的完整 session 历史复制进每一代 candidate，导致 capacity 随
session 时长而非当前未来窗口增长。该缺陷必须在增量验证和 snapshot 性能优化前修复：构造
candidate 时使用同一 coherent `execution_time=E`，旧轨迹仅保留 E 之前最近的一个 knot 以及
`[E,R)` 仍需执行的点，E 更早的历史全部裁剪；从 R 开始接入新 suffix。Capacity 的语义是
“仍可能执行的活跃点”，不是 session 历史点数。裁剪前后 `[E,R]` 采样必须等价；capacity 64、
30 Hz update、100 ms knot 的 10 分钟 fake 长稳不得出现历史增长或 `CapacityExceeded`。

`TrajectoryImage` 的协议传输上限仍为 256 点，snapshot exchange 仍采用三个固定预分配槽位，
不引入运行时容器、共享所有权或 RT 动态分配。`buffer_capacity: 64` 作为当前 controller 的
活动内部节点准入上限；snapshot 发布与 RT generation 接管改为复制元数据和 `point_count` 个
有效节点，不再无条件复制完整 256 点数组。内部为保持大 splice 容差下的前缀不可变，会为
拼接边界保留左右两侧状态标记，因此正常活动节点预计约 9--10 个，而不是只计算传输 knot
得到的 7--8 个。capacity 64 的最坏有效复制量仍须控制在 15 KiB 左右，正常复制量低于
2.5 KiB。必须保持 trivially-copyable、三槽租约状态机和零 RT allocation，并用测试记录实际
复制字节数；64 以上配置若要作为正式性能配置，需重新执行性能标定。

## 3. Provisional Rolling 动态包络

13 个旋转轴（双臂 12 轴和 Turn）统一使用：

```yaml
rotary:
  v_limit_positive: 0.2617993878  # 15 deg/s
  v_limit_negative: 0.2617993878
  a_limit_positive: 0.75          # rad/s^2
  a_limit_negative: 0.75
  a_stop_positive: 0.75           # rad/s^2
  a_stop_negative: 0.75
  position_margin_lower: 0.00872664626  # 0.5 deg
  position_margin_upper: 0.00872664626
```

Updown 使用：

```yaml
updown:
  v_limit_positive: 0.09          # m/s
  v_limit_negative: 0.09
  a_limit_positive: 0.5           # m/s^2
  a_limit_negative: 0.5
  a_stop_positive: 0.5            # m/s^2
  a_stop_negative: 0.5
  position_margin_lower: 0.005    # m
  position_margin_upper: 0.005
```

旋转轴满速停车时间约 349 ms、停车距离约 2.62 度；Updown 分别约 180 ms 和
8.1 mm。位置 margin 不代替 stopping-viability 的额外停车距离校验。除已有明确来源的
Updown 数值外，上述 Rolling 数值均按 `provisional / ESTIMATED_NOT_MEASURED` 暴露，
不得描述成生产实测能力。

## 4. Splice 容差与已接受风险

用户明确裁决使用以下较大容差：

```yaml
splice_tolerance:
  rotary:
    position: 0.01      # rad，约 0.573 deg
    velocity: 0.03      # rad/s，约 1.72 deg/s
  updown:
    position: 0.003     # m，3 mm
    velocity: 0.02      # m/s，20 mm/s
```

### RISK-E102-001 — 大 splice 容差允许跨代命令不连续（已接受）

- **状态**：`ACCEPTED_BY_USER`（2026-08-19）。
- **因果机制**：splicing 在同一个 `replace_from_ns` 比较旧轨迹 left state 与新 suffix
  right state 的 `q/qdot`。RT 不在两者之间生成过渡时间；内部图像保持 R 前旧前缀，
  execution cursor 跨过 R 时将容差内差值直接作为新期望值。
- **已知影响**：最大允许的旋转位置差折算到一个 4 ms 周期约为 2.5 rad/s（约
  143 deg/s），旋转速度差折算加速度约为 7.5 rad/s^2；Updown 也可能在拼接边界超过
  本文普通 Rolling 速度/加速度包络。旋转位置容差还大于 0.5 度 position margin。
- **责任边界**：D-09 保持不变，Motion 仍负责输出平滑的首点；RT 只按上述容差接受或
  拒绝，不自动 blend、clamp、插点或改写首点。容差是准入上界，不是建议 Motion 用满。
- **强制验证**：mock 与 Motion 联调必须逐批记录实际 `delta_q/delta_qdot`，覆盖接近容差
  边界的正反向用例，并报告 generation 切换处命令步阶、跟踪误差和拒绝率。批次被接受
  不能作为平滑验收结论。真实硬件运动仍需另行授权。

## 5. 模式切换 source-command 证据方案

> 原型仓库将本问题登记为 BQ-131；当前 robot_driver main 的 BQ-131 已被其他契约裁决占用，
> 因此这里只保留方案语义，不沿用该编号作为当前权威记录。

模式切换采用 BQ-131 Option A，不新增 hardware last-command mirror，也不扩大 JTC 补丁：

1. `enable_manager` 使用带接收年龄的 JTC controller-state `output.positions` 或 Rolling
   public-state `desired_positions` 做切换前 source-command admission。
2. 同时使用 250 Hz position state 做连续 N 周期的有限差分稳态检查，并校验 source command
   与 actual 的 takeover residual。
3. FJT→Rolling 的 STRICT switch 开始后，Rolling 在 `on_activate()` 读取 command interface
   中精确的持久最后命令，再做一次边界 residual fail-closed 复核。
4. Rolling→JTC 保留既有 `set_last_command_interface_value_as_state_on_activation: false`，由
   stock JTC 从 actual seed hold；该方向只承诺命令步长不超过 takeover tolerance，不宣称
   严格 C0。

上游 controller-manager 会先 deactivate source，再 assign/activate target，target activation
失败时不会自动恢复 source。因此成功路径的连续性与失败路径的终态必须分别验证；不得把
STRICT 描述成带事务回滚的 controller switch。

模式切换失败按以下规则收敛：

- 任何事前 admission 失败均不得调用 controller-manager，source 保持 ACTIVE。
- controller-manager 明确失败且复核为 source ACTIVE、target INACTIVE 时，保持 source 模式，
  返回 `SwitchRejected`，不自动重试。
- 复核为 source/target 均 INACTIVE 时，运行既有普通整组失能路径直到确认 `0x0040`；状态全部
  明确时，后续恢复入口是 fresh enable，而不是自动重激活任一 controller。
- service timeout、部分切换或 controller 最终状态无法确认时，尽力完成硬件失能，锁存
  `RestartRequired`，本进程拒绝再次 enable/reset。
- 只有既有 Fault、意外离开 Operation Enabled 或普通失能阶段超时才升级到已有 Quick Stop
  路径；普通 mode-switch 拒绝本身不新建 Quick Stop 触发条件。

方案 A 的初始接管参数为：

```yaml
mode_switch:
  source_state_max_age_ms: 100
  stable_sample_count: 5
  maximum_sample_period_ms: 8
  stable_velocity_threshold:
    rotary: 0.00872664626  # 0.5 deg/s
    updown: 0.001          # 1 mm/s
  takeover_error_tolerance:
    rotary: 0.00872664626  # 0.5 deg
    updown: 0.005          # 5 mm
```

JTC 的 `state_publish_rate` 显式配置为 50 Hz。稳态门至少观察约 20 ms；source-state 快照
超过 100 ms、样本周期无效、任一轴仍在移动或 command/actual residual 超限时，均在调用
controller-manager 前拒绝。Motion 可在源稳定后使用新的 `request_id` 重试，RT-Control
不自动重试。

Rolling 两个 topic 使用 `robot_interfaces_qos` 的独立命名 profile：

```yaml
Q_ROLLING_COMMAND:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 1
  deadline_ms: 100
  lifespan_ms: 100
Q_ROLLING_STATE:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 5
  deadline_ms: 100
  lifespan_ms: 200
```

Command profile 选择最新完整 suffix，不用可靠重传追赶已经变旧的 update；丢包由后续 30 Hz
更新、future horizon、update timeout 和 low-water 状态机处理。State profile 可靠传输接收
序号、拒绝原因和停止状态。DDS deadline miss 只进入通信诊断，不直接触发停车。

## 6. 完整裁决登记

### 6.1 记录规则与完成口径

本节使用以下标记：

- **【推荐／最终采用】**：Codex 推荐且已成为实现约束。
- **【Codex 推荐】** 与 **【用户最终采用】**：用户明确接受了与 Codex 首选不同的风险方案；
  实现必须服从用户最终裁决，同时保留风险说明。
- **【备选】**：讨论过但不采用。
- **证据门**：软件设计已经确定，但数值是否可升级为生产结论仍必须通过测量。证据门不是
  留给编码者自行选择的“开放设计问题”。

截至 2026-08-19，本期影响协议、实时所有权、失败收敛、参数语义和联调边界的软件选择均已
给出最终项。后续实现若发现事实与本节不一致，应停止并修改本记录；不得在代码中静默选择
另一个方案。

### E102-D01 — 扩展 JTC 还是新增 rolling controller

**问题**：视觉伺服的持续 suffix 更新应进入现有 JointTrajectoryController，还是由独立
controller 承担？

- **【备选 A】** 扩大现有 JTC patch，增加 session、buffer、timeout 和 rolling topic。
  优点是少一次 controller 切换；缺点是显著扩大第三方补丁，并把普通 FJT 与持续流模式的
  状态机耦合。
- **【推荐／最终采用 B】** 新增 rolling_trajectory_controller，保持 whole_body_jtc 只负责
  完整 14 轴 FJT；两者声明同一组 position command interfaces，任何时刻至多一个 ACTIVE。

**影响**：不改变 CiA402 CSP mode 8、PDO 或 slave 运动配置；新增工作量集中在独立插件、
enable_manager 模式协调和互斥验证。普通 FJT 的行为与补丁边界必须回归不变。

### E102-D02 — 公共接口落点与联调前发布方式

**问题**：rolling 五个 endpoint 是 RT-Control 域私有接口，还是 Motion/RT-Control 的公共
跨域接口？在功能未闭环时如何让两个仓库共同构建？

- **【备选 A】** 放进 robot_driver 的 rt_control_interfaces。这样单仓构建方便，但 Motion
  会反向依赖 RT 域私有包，违反现有契约边界。
- **【备选 B】** 立即向 robot_interfaces 提 PR 并合并，再写 driver。这样发布顺序简单，
  但未经过可执行 producer/consumer 验证的 schema 会过早成为公共事实。
- **【推荐／最终采用 C】** 类型直接开发在 robot_interfaces，并按 endpoint 提供方拆分：
  `RollingJointTargetBatch`／`RollingJointPoint` 属 `robot_motion_interfaces`；
  `SetJointControlMode`／`OpenRollingJointSession`／`CloseRollingJointSession`／
  `RollingJointControlState` 及其成员类型属 `robot_rt_control_interfaces`；QoS 属
  `robot_interfaces_qos`。先使用基于
  f699f45972ad15bbbbbb3da1a4894faf209144c9 的功能分支和多仓 overlay 构建，不在
  robot_driver 复制公共 IDL。robot_driver 达到 Motion 可测试门后，才提出接口 PR、
  正式确定 0.8.0 发布并回填最终 merge SHA。

**影响**：功能阶段可以推送接口功能分支以获得可复现 SHA，但“推送功能分支”不等于提交
PR 或发布合同。driver 的 source-lock 和 deps.repos 只在公共接口合并后锁最终 main SHA；
正式发布前不允许四域混用不同 schema。该拆分不引入 schema 循环：Motion batch 不引用
RT 类型，RT service/state 也不引用 Motion 类型；两域实现节点可以同时依赖两个公共包。
本裁决在接口落地时依据 `contract/endpoints.yaml` 的 producer 所有权门禁作了上述精化，
不是把 update 改成 RT-Control 发布。

### E102-D03 — 轴集合与“一次一臂”的归属

**问题**：RT 协议是否需要 controlled_arm、axis enable 或部分关节数组？

- **【备选 A】** 在协议里增加左右臂选择并允许省略非伺服轴。消息更短，但 RT 必须理解
  Motion 业务策略，并在切臂时处理隐式保持值。
- **【推荐／最终采用 B】** 每点固定完整 14 轴，顺序为
  right_joint1..6、left_joint1..6、turn、updown；RT 不增加 controlled_arm。

**影响**：一次只伺服一条臂是 Motion 本期策略，不是 RT 能力限制。另一条臂、Turn 和
Updown 由 Motion 填写同一未来时刻的已接受期望值，不能直接拿当前 /joint_states 值填入
所有未来 knot。

### E102-D04 — Motion knot、批次和 RT 采样频率

**问题**：100 ms 轨迹点间隔是否意味着整个命令接口只能 10 Hz 更新？

- **【备选 A】** 每 100 ms 才发送一次批次。实现简单，但视觉帧到新 future 生效的等待大，
  且单包丢失直接损失 100 ms 更新机会。
- **【备选 B】** Motion 以 30 Hz 生成 33 ms knot。响应快，但改变用户已明确的 100 ms
  时间参数化并增加验证、序列化负担。
- **【推荐／最终采用 C】** knot 间隔固定 100 ms；Motion 以 30 Hz 重新计算并发布完整
  suffix；RT 以 250 Hz 对 cubic Hermite 采样并只写 position command。

**影响**：典型 500 ms suffix 含 R、R+100、…、R+500 六个传输点。30 Hz 是“重算批次”
频率，10 Hz 是“相邻 knot”频率，250 Hz 是“驱动采样”频率，三者不能混写。RT 不接收
TwistStamped，也不把 qdot 直接写进硬件速度模式。

### E102-D05 — 初始 horizon、最大 horizon 与容量

**问题**：缓冲参数是沿用原型的 8 ms／400 ms，还是按照本期停车包络重新设置？

- **【备选 A】** required initial horizon 8 ms、max horizon 400 ms。内存更小，但连一次
  最大旋转停车时间加 guard 都缺少足够更新余量。
- **【推荐／最终采用 B】** required_initial_horizon_ms=500、
  planned_horizon_ms=500、max_horizon_ms=600、buffer_capacity=64。

**语义**：

- Prime 最后一点相对 session 起点必须落在闭区间 500～600 ms。
- 运行中 max horizon 等于候选最后一点时间减本次 coherent execution_time，而不是本批
  自身跨度；超过 600 ms 返回独立 HorizonExceeded。
- capacity 64 是活动内部节点上限，不是频率、毫秒或 session 历史长度；合法范围冻结为
  2～256，默认 64。协议传输 ceiling 仍为 256。
- planned_horizon_ms 是 Motion 配置，不是 driver ROS 参数；driver 只验证实际候选。

**影响**：500 ms 初始 future 相对约 349 ms 的旋转满速停车时间和 16 ms guard 留出约
135 ms。64 对正常约 9～10 个内部活动节点有充分余量，但不能用来掩盖历史不裁剪的缺陷。

### E102-D06 — timeout、replace lead、guard 与 controller period

**问题**：这些时间量是否合并成一个保守常数？

- **【备选 A】** 一个编译期 safety_margin。字段少，但无法知道迟到来自检测、验证、交接
  还是周期量化，也无法单项标定。
- **【推荐／最终采用 B】** YAML 中分别配置并在状态／证据中保留：

~~~yaml
update_timeout_ms: 200
replace_lead_ms: 16
state_publish_period_ms: 20
prime_timeout_ms: 100
nominal_controller_period_ms: 4
maximum_controller_period_ms: 8
stopping_guard:
  one_cycle_detection_ms: 4
  stop_time_growth_ms: 4
  non_rt_to_rt_visibility_ms: 4
  period_quantization_ms: 4
~~~

**影响**：replace lead 与四项 guard 的初值都碰巧为 16 ms，但用途不同，禁止共用一个字段。
period 非正或大于 8 ms 触发 ClockAnomaly。update_timeout 的有效值必须由 open response
回报；T19 之后可修改 provisional 数值，但不能改变字段语义。

### E102-D07 — 参数存放和生效时机

**问题**：参数应继续写成 C++ 常量、支持 session 中热改，还是使用 YAML configure-time
快照？

- **【备选 A】** 编译期常量。运行路径最简单，但每次调参需改代码，且测试值容易混入生产。
- **【备选 B】** ROS 参数热更新。调整方便，但活动轨迹可能在一半 session 中换掉安全包络。
- **【推荐／最终采用 C】** controller 标量进入 controllers.yaml；每轴包络进入独立
  rolling_envelope_provisional.yaml。参数描述符只读，在 configure 时一次读取、交叉校验并
  冻结；修改后必须重新 configure/restart。

**硬约束**：越界、缺字段、非有限值、关系不成立均 configure 失败，不做静默钳制或默认
回退。Motion 的 30 Hz、100 ms 和 500 ms 配置由 Motion 自己持有，driver 文档只记录合同。

### E102-D08 — provisional 包络的身份和加载方式

**问题**：当前没有台架停车实测，软件是只允许 test-only，伪装成 production，还是引入第三
种来源？

- **【备选 A】** 只允许 test-only，真实硬件永远不能配置。最安全，但无法形成后续受控低速
联调的软件形态。
- **【备选 B】** 当作 production。使用方便，但把估计值伪装成生产能力。
- **【推荐／最终采用 C】** LimitsSource 增加 PROVISIONAL=3；允许软件在真实硬件配置和
激活，但启动持续 WARN、公共状态持续暴露 source，且任何硬件运动仍需单独授权。

**加载与版本裁决**：

- 14 轴十项字段必须显式出现在 YAML；旋转位置界取 URDF 与 joint_limits.yaml 的交集，
  其他数值使用本文第 3 节用户裁决。
- 不从 URDF、PLC 或代码常量隐式补缺失字段。
- limits_version 使用“已安装 YAML 文件精确字节”的 SHA-256；文件不自包含该 hash。
  注释或 metadata 改动也产生新版本，这比人工 version 字符串漂移更可审计。
- allow_provisional_limits 与 allow_test_only_limits 是两个独立 opt-in；后者不能开启
  provisional，前者也不能开启 test-only。

**影响**：这些限制只约束 Rolling；whole_body_jtc 的普通 FJT 不读取该文件。

### E102-D09 — splice 容差、平滑责任与风险接受

**问题**：RT 是否采用较小连续性容差、用户要求的大容差，或自动 blend？

- **【Codex 推荐 A】** 使用接近数值／跟踪噪声量级的小容差，避免容差本身允许明显命令跳变。
- **【用户最终采用 B】** 使用第 4 节的 0.01 rad、0.03 rad/s、3 mm、20 mm/s，并接受
  RISK-E102-001。
- **【备选 C】** RT 自动 blend、clamp 或插点。可隐藏生产者误差，但改变路径、时序和停车
  证明，且与 Motion 负责整形的边界冲突。

**影响**：最终实现服从 B，不执行 C。协议只能宣称“有界 splice 差值”，不能把非零容差
描述成严格数学 C0/C1。Motion 必须让实际差值尽量接近零；验收必须记录边界命令步阶。

### E102-D10 — Prime 首点如何与 activation hold 对齐

**问题**：原型在没有旧 candidate 的 Prime 中跳过 splice 检查，首点是否可为任意合法状态？

- **【备选 A】** 只检查限值和 horizon。这样 Prime generation 激活时可从 hold 直接跳到
  任意合法点。
- **【备选 B】** 允许使用普通大 splice 容差。可以启动，但把已接受风险扩大到每次 session
  入口。
- **【推荐／最终采用 C】** open response 返回的 hold positions 必须被 Prime t=0 首点原值
  复制；首点 velocities 必须全为零。使用数值精确相等，不用普通 splice 容差，也不由服务端
  插入锚点。

**影响**：PositionDiscontinuity／VelocityDiscontinuity 可复用为 Prime 不匹配拒绝码。
Prime 仍需至少两个点并满足 500～600 ms horizon。该裁决修复原型可在 session 首拍跳变的
缺陷。

### E102-D11 — 大容差下如何真正保持 committed prefix 不可变

**问题**：点式 Hermite 中替换 R 端点会改变它前面的整段曲线。如何既保留用户的大容差，
又不在 generation 提前激活时改写 R 之前的命令？

- **【备选 A】** 沿用原型：复制 R 前的 knots、放入新 R 端点并立即激活。代码简单，但新
  端点会反向改写“上一 knot 到 R”的曲线，违反 authoritative-prefix 合同。
- **【备选 B】** 强制新 R 状态与旧采样精确相等。可保持普通点表示，但实质推翻用户已接受
  的大容差。
- **【推荐／最终采用 C】** 内部 TrajectoryImage 显式表示 splice 的左右两侧：在 R 保留
  旧轨迹采样作为 left state，同时保存新批首点作为 right state；输入批次仍严格递增，只有
  内部图像允许一个有标记的同时间边界。t<R 采旧左侧，t=R 及之后采新右侧，禁止在两侧之间
  插值。

**附加规则**：

- 如果 R 在旧 Hermite 段内部，先按旧曲线采样 q/v，将旧段精确拆到 R；测试必须证明拆分前后
  t<R 的采样等价。
- 如果重复替换同一个 R，连续性与 left-limit 比较，不能相对上一版 right state 逐次累加
  tolerance。
- earliest_changed_ns 仍等于 R，因此 generation 可提前激活而不修改 R 前命令；跨过 R 的
  有界跳变正是 RISK-E102-001，不伪装成普通动态段。
- direct q/v/a 和 stopping viability 只校验实际可执行的连续段；left→right 的跳变只执行
  splice tolerance 与显式风险观测，不拿 4 ms 推导量伪装成包络内运动。

**影响**：这是从代码审计发现的阻塞性正确性修复，优先于历史裁剪、增量校验和 RT 性能优化。
内部边界标记必须 trivially-copyable，且计入 capacity。

### E102-D12 — 历史裁剪锚点与 capacity 语义

**问题**：每一代 candidate 是否保留从 session t=0 开始的所有点？若裁剪，是合成 E 点还是
保留旧节点？

- **【备选 A】** 保留完整 session 历史。实现最直接，但 30 Hz 更新会在数秒内耗尽 64 点。
- **【备选 B】** 每次在 execution_time=E 合成新 anchor。点更少，但引入重复浮点重建和额外
  证明。
- **【推荐／最终采用 C】** 使用同一 coherent E，保留能够采样 E 的最后一个原有有效节点
  及 [E,R) 的未执行 prefix，删除更早历史；若 E 恰在有左右状态的 splice 上，保留右侧状态。

**影响**：capacity 表示“仍可能被执行或用于当前段采样的活动内部节点”，不是 session 历史。
30 Hz／100 ms knot／500 ms suffix／capacity 64 的 10 分钟 fake 长稳必须证明 point_count
有界且不因历史产生 CapacityExceeded。

### E102-D13 — 验证范围、顺序和全量 oracle

**问题**：每批是否重复校验完整 candidate？增量校验从哪里开始？

- **【备选 A】** 每批全量校验所有活动段。最容易证明，但反复检查完全不变的 prefix。
- **【备选 B】** 从 candidate.earliest_changed_ns 开始。pending generation 继承时该字段可能
  早于本批真正变化处，仍会重复校验。
- **【推荐／最终采用 C】** Prime 全量校验；普通替换只校验 splice 的新 right state、新 suffix
  连续段及其 stopping viability。显式 left prefix 和已经验证的连续段不重复校验。

**运行时顺序**：

1. version／identity／session／client 与 stale sequence；
2. shape、非空、传输 count、finite 和输入时间严格递增；
3. LateReplace、旧轨迹覆盖与 splice left/right continuity；
4. 历史裁剪后的内部 capacity；
5. initial／running minimum horizon 与 HorizonExceeded；
6. 新连续段 position、velocity、acceleration extrema；
7. 新连续段 stopping viability；
8. publication 前 base generation 与 replaceable boundary 再检查。

**影响**：删除原型 validateCandidate 中完全重复的第一轮 checkSegment。生产运行时不做
“增量后再全量”的双重校验；全量实现只保留为单元／随机差分 oracle。若等价证明失败，则在
开发阶段取消增量优化，而不是上线时动态回退。

### E102-D14 — TrajectoryImage 存储与跨线程复制

**问题**：按运行时 capacity 动态分配、模板定尺 64，还是保留 transport 256 的静态槽并缩短
复制？

- **【备选 A】** vector/shared_ptr 按 capacity 分配。内存紧凑，但 RT 所有权、析构和分配证明
  明显变复杂。
- **【备选 B】** 编译期只支持 64。复制最小，但 YAML 的 2～256 能力和协议 transport ceiling
  失去意义。
- **【推荐／最终采用 C】** 保留三个 256 节点的固定预分配 snapshot slot 和一个固定 RT active
  image；发布、接管只复制元数据、有效节点和边界标记。

**影响**：capacity 64 的有效复制目标约 15 KiB，正常约 9～10 节点低于 2.5 KiB；未使用尾部
允许残留旧字节，但任何读路径只能访问 point_count。保留三槽租约／publication sequence 的
ABA 防护和零 RT allocation。slot 无法取得属于内部不变量失败，不伪装成普通用户批次拒绝。

### E102-D15 — RT 采样查段方式

**问题**：250 Hz update 是否每拍对点数组做两遍 O(N) 扫描？

- **【备选 A】** 保持原型精确点扫描加区间扫描。易读，但单拍耗时随 execution 位置增长。
- **【备选 B】** 每拍 binary search。上界较好，但没有利用 execution_time 单调性。
- **【推荐／最终采用 C】** RT image 内维护单调 segment cursor；generation 改变时从有界起点
  重置，随后只向前移动。非 RT splice／测试保留纯函数二分或参考采样器。

**影响**：遇到同时间 left/right splice 标记时，t=R 选择最右的新状态，t<R 不跨零时长边界
插值。换代、越界、跨多个段和 period anomaly 都必须与参考采样逐例一致。

### E102-D16 — 输入年龄起点、callback 所有权和 CPU 亲和

**问题**：accepted update age 从验证结束还是从接收开始计算？是否为验证创建专用线程并放到
RT 核？

- **【备选 A】** 验证完成后取时间，并让 callback 可在任意 ros2_control_node 线程执行。
  长验证会把旧输入“刷新成刚到”，且可能污染 RT 核。
- **【备选 B】** 新建专用验证线程并单独 pin。归属最明确，但增加队列、唤醒和线程生命周期。
- **【推荐／最终采用 C】** subscription callback 入口立刻记录 steady receive time；只有批次
  最终接受时才把该原始时间与 generation 原子发布。callback group 使用 MutuallyExclusive，
  维持单一非 RT writer，不创建新线程。

**亲和依据**：现有 native／Docker 策略先把普通进程树放在 housekeeping，仅把唯一 FIFO80
update、EtherCAT-OP 和已冻结的 rtcan-master 放到 CPU14；DDS、service 和 executor 仍留在
housekeeping。因此 T-13 只需验证现有策略，不新增 validation affinity 机制。目标机以
ps -To pid,tid,comm,cls,rtprio,psr 和 affinity mask 保存证据。

### E102-D17 — update timeout、low-water、时钟异常与停止锚点

**问题**：断更时只看 timeout、只看 horizon，还是两者独立？停止从 actual 还是 desired 开始？

- **【备选 A】** 只看 timeout。即使 future 已短到不能停车，也继续执行。
- **【备选 B】** 只看 low-water。生产者失联但旧 future 很长时会继续陈旧运动。
- **【推荐／最终采用 C】** 两者独立，任一先到即锁存 Stopping；停止从上一拍已经写出的有限
  desired q/qdot 开始，生成固定存储同步 C1 stop，随后 hold。

**规则**：

- Running 仅在 H_available > T_stop(current desired)+guard 时成立，等号触发 LowWater。
- 普通更新也必须在 non-RT admission 证明接受后仍有可停车 horizon；否则返回
  InsufficientHorizon 并保留旧 future。
- 非正或大于 8 ms 的 period 锁存 ClockAnomaly，后续 stop 每次只按名义 4 ms 推进。
- runtime process-data age/WC 延续既有 WARN／硬件保护策略，不新建 rolling 自动停车条件；
  mode/open admission 仍要求反馈 age 有限且不超过 500 ms。
- Stopping/Holding 不接受恢复 update；必须 finalize close 后 fresh open。

**影响**：满速时 low-water 可能早于 200 ms update timeout，这是正确行为。视觉丢帧不等于
Motion 失联；Motion 若仍健康，应继续发布自己整形的 hold／减速 future。

### E102-D18 — command/state QoS

**问题**：command 是否可靠重传？deadline 是否直接参与停车？

- **【备选 A】** command reliable keep-last(1)。单包更不易丢，但可靠重传可能在批次已经过期
  后造成 head-of-line 延迟。
- **【推荐／最终采用 B】** command best-effort、volatile、keep-last(1)、deadline 100 ms、
  lifespan 100 ms；state reliable、volatile、keep-last(5)、deadline 100 ms、lifespan
  200 ms。

**影响**：两个 profile 分别命名 Q_ROLLING_COMMAND 和 Q_ROLLING_STATE，并在
robot_interfaces_qos 提供 C++／Python 同源入口。DDS deadline miss 只增加诊断计数；真正的
停止条件仍是 controller 内部 accepted-update age 和 low-water。Service 使用 ROS 2 标准
service QoS。

### E102-D19 — 公共 IDL、版本和错误表达

**问题**：公共结果只给 string、只给通用 ErrorInfo，还是同时提供协议枚举？

- **【备选 A】** bool + string。调试直观，但 Motion 会被迫解析文本。
- **【备选 B】** 只使用 DREE ErrorInfo。统一，但无法稳定表达 LateReplace、SessionBusy 等
  rolling 协议分支。
- **【推荐／最终采用 C】** batch 使用独立 RollingRejectCode，session 使用独立
  RollingStopReason；service response 同时包含细粒度 RollingServiceResult 和
  robot_system_interfaces/ErrorInfo。程序分支读取枚举／retryable，永不解析 message/detail。

**schema 约束**：

- provider-owned 包与 endpoint ID 固定为：R-IN-06 set_mode、R-IN-07 open、R-IN-09 close、
  R-OUT-07 state 使用 `robot_rt_control_interfaces`；M-09 update 使用
  `robot_motion_interfaces`。R-IN-08 暂不分配：update 的 provider 是 Motion，不能为了编号
  连续把它伪装成 RT-Control endpoint；稳定 ID 不要求连续。
- point 的 positions／velocities 均为 float64[14]；batch points 为有界 sequence，ceiling 256。
- UUID 使用 unique_identifier_msgs/UUID；axis hash 固定 32 byte，已核验 digest 为
  25c6e82bf505ca9eb99db1c645ab75d7ecde0153faaf6a7492c6210c4d362526。
- V1 暂时要求 protocol 1.0 精确匹配；ROS schema 本身变化需要原子升级，未建立兼容矩阵前
  不声称 minor 向前兼容。
- sequence 必须非零并严格大于 last_seen；生产者通常从 1 开始，但第一个更高自包含序号也
  可接受，后续允许跳号，永不回绕。
- HorizonExceeded 追加为 RejectCode=20，不重编号原有 0～19；RejectCode 与 StopReason 的
  NONE 常量位于各自成员类型，避免同一枚举空间。
- open response 至少回报 boot/session/client、hold q、axis/limits hash、capacity、
  initial/max horizon、replace lead、update timeout、nominal period和 limits source。
- state 使用明确的 execution／replaceable／buffered／age／generation／sequence 字段，
  不增加语义不清的 progress 百分比。ROS stamp 仅诊断，不参与调度。

### E102-D20 — close 的异步阶段和幂等

**问题**：停止完成后自动销毁 session、以“新 request_id”隐式表示 finalize，还是显式声明
操作阶段？

- **【备选 A】** 一次 close 后自动 Stopping→Holding→None。调用简单，但 Motion 可能看不到
  有 session 身份的 terminal hold，且模式切换与状态丢失更难区分。
- **【备选 B】** 原型规则：Holding 中出现一个新 request_id 就当作 finalize。无需新字段，
  但客户端超时后误换 ID 可能意外销毁 session。
- **【推荐／最终采用 C】** CloseRollingJointSession request 显式携带
  REQUEST_STOP／FINALIZE。REQUEST_STOP 异步锁存 stop，完成通过 state 观察；
  FINALIZE 只在 Holding 接受并无运动地销毁 session。

**影响**：同 request_id + 同 payload 返回原结果；同 ID 不同 payload 为 WrongRequest。
保留固定 8 个 non-finalizing outcome 槽和一个 finalize 槽以避免无界缓存。V1 不提供 public
abrupt abort；disable/fault 是独立高优先级生命周期。

### E102-D21 — FJT↔Rolling 的 source-command 证据

**问题**：没有 hardware last-command state interface 时，切换前如何证明 source command，
是否扩大硬件/JTC patch？

- **【推荐／用户最终采用 A】** enable_manager 使用带本地接收年龄的
  /whole_body_jtc/controller_state output.positions 或 rolling public desired_positions，
  同时直接读取 250 Hz actual position 做稳态／residual gate；FJT→Rolling 再在
  rolling on_activate 读取持久 command interface 做最终 fail-closed 复核。
- **【备选 B】** hardware 导出 14 个 last-command mirror，并补 JTC 从 command seed 的接管
  机制。双向证据更强，但扩大 EtherCAT/JTC patch 和验证范围。

**影响**：Motion 在请求 FJT→Rolling 前先 cancel 自己的 FJT 并等待 Action result；
set_mode 不创建 Action client。Rolling→JTC 保持
set_last_command_interface_value_as_state_on_activation=false，JTC 从 actual seed，因此只
承诺切换步长不超过 takeover tolerance，不宣称严格 C0。

### E102-D22 — 模式切换稳态参数和超时

**问题**：接管门直接复用 FJT 首点容差，还是使用独立可调参数？

- **【备选 A】** 复用 1°／0.05 m FJT admission tolerance。字段少，但该容差用途不同且过宽。
- **【推荐／最终采用 B】** YAML 中使用独立 provisional 参数：

~~~yaml
mode_switch:
  source_state_max_age_ms: 100
  stable_interval_count: 5
  maximum_sample_period_ms: 8
  switch_timeout_ms: 500
  stable_velocity_threshold:
    rotary: 0.00872664626
    updown: 0.001
  takeover_error_tolerance:
    rotary: 0.00872664626
    updown: 0.005
~~~

**语义**：stable_interval_count 计“连续通过的区间”，不是数组样本数；值 5 需要 6 个 position
样本，在名义 4 ms 下至少观察 20 ms。JTC state_publish_rate 显式为 50 Hz；source snapshot
按 steady 接收年龄判 100 ms。反馈 process-data age 还必须有限且不超过 500 ms。

**新增自主裁决**：mode switch 使用独立 500 ms provisional timeout，不改既有 enable/disable
路径的 controller_switch_timeout=4 s。T19 测得分布后再缩放；不得把 4 s 失败等待称为“快切”。

### E102-D23 — STRICT switch 失败如何收敛

**问题**：失败后自动 rollback／retry、总是 restart，还是根据可确认 controller 状态收敛？

- **【备选 A】** 自动重试或自动恢复 source。controller_manager 不提供事务 rollback，自动化
  会扩大竞态。
- **【备选 B】** 任何失败都直接 restart-required。最简单，但可确认 source 未改变的普通拒绝
  也造成不必要停机。
- **【推荐／最终采用 C】** 先做 admission；调用后以 service 结果和 list_controllers 状态分类：
  - admission 失败：不调用 controller_manager，source 保持 ACTIVE；
  - 明确失败且 source ACTIVE／target INACTIVE：保持 source，SwitchRejected，不自动重试；
  - source／target 都 INACTIVE：走既有整组普通失能至 0x0040，状态明确后只能 fresh enable；
  - timeout、两者都 ACTIVE、target-only 意外、部分状态或无法确认：尽力硬件失能，锁存
    RestartRequired，本进程拒绝后续 enable/reset。

**影响**：成功 response 后也必须确认恰好 target ACTIVE、source INACTIVE。普通切换拒绝不新增
Quick Stop 条件；既有 Fault、意外离开 Operation Enabled 或失能超时仍走原路径。

### E102-D24 — JTC topic 旁路、注册表与 launch 顺序

**问题**：如何关闭绕过 Action admission 的 ~/joint_trajectory；rolling 配置失败时是否继续
启动部分系统？

- **【备选 A】** 保留 topic 并依赖使用规范。任何发布者仍可绕过 1°／500 ms admission。
- **【备选 B】** 保留 subscriber，但 callback 每条拒绝并打 ERROR。能够观测误用，但高频误发
  会制造日志和 callback 负担。
- **【推荐／最终采用 C】** JTC patch 根本不创建 joint_trajectory_subscriber；ROS graph 中
  不出现 /whole_body_jtc/joint_trajectory，FJT Action 是唯一轨迹入口。

**集成裁决**：

- enable_manager 参数改为 motion_controller_names=[whole_body_jtc,
  rolling_trajectory_controller] 和 default_motion_controller=whole_body_jtc；空、重复或默认
  不在集合均 configure 失败。
- launch 依次把 whole_body_jtc 配为 INACTIVE、rolling 配为 INACTIVE，再启动 enable_manager。
  任一 mandatory controller 配置失败则 launch fail-fast，不带着半套 registry 继续使能。
- 普通 enable 默认只激活 whole_body_jtc；rolling 不自动 ACTIVE。
- disable 只请求停用实际 ACTIVE 的 registry 成员，并在结果后验证全部成员 INACTIVE。
- joint_state_broadcaster.update_rate:100 不动；实测合同仍为 125 Hz。

### E102-D25 — 手眼外参进入模型与左手镜像

**问题**：外参留在现场临时参数、在 xacro 内计算四元数转 RPY，还是作为版本化安装 artifact？

- **【备选 A】** 现场临时参数，不进入 robot_description。符合一般“现场标定不进基准模型”
  规则，但每次部署可能缺失，无法形成可复现 TF 合同。
- **【备选 B】** YAML 只存 quaternion，在 xacro 写三角公式转换。单一旋转表示，但 xacro
  表达复杂、边界精度和 gimbal 分支难审。
- **【推荐／最终采用 C】** 用户已对本项窄范围覆盖一般规则：固定安装的手眼外参作为版本化
  robot_description config；YAML 同时保存 matrix、normalized quaternion 和由标定导出工具生成
  的 rpy_rad，xacro 只 load_yaml，不手算或硬编码。

**左右臂裁决**：

- 右手使用独立标定 artifact。
- **Codex 推荐**左手独立标定；**用户最终采用**本期明确的镜像估计，并标记
  calibration: mirror_estimate_of_right，左右精度分开验收。
- 左手必须在 YAML 中保存显式 proper rigid transform；xacro 不在运行时猜镜像平面。导出／测试
  校验 rotation 正交、det=+1、quaternion norm、matrix/quaternion/rpy 相互一致及 homogeneous
  最后一行。
- 新增 right/left_hand_camera_link 和对应 color_optical_frame；光学旋转使用精确
  -pi/2、0、-pi/2。

**仓库边界**：robot_description 是权威源，robot_driver 的构建副本同步 exact upstream
改动并安装 config。两个仓库各使用一个中文功能分支；不是每任务建子分支。若实施时仍拿不到
具体 calibration artifact，T-01 以“缺输入数据”停止，禁止填零值或经验值。

### E102-D26 — Motion producer 的完整责任

**问题**：Motion 应向驱动层连续发 TwistStamped、直接发控制律 qdot，还是发整形后的 future？

- **【备选 A】** arm driver 接 TwistStamped。需要驱动层做 Jacobian、奇异性和任务空间语义，
  超出 RT-Control 所有权。
- **【备选 B】** 把 IBVS/PBVS 输出 qdot 直接作为每批新首点 velocity。不能保证 splice、
  100 ms Hermite 段和 0.75 rad/s² 包络。
- **【推荐／最终采用 C】** Motion 把视觉控制律结果经过限速、限加速度和时间积分，生成完整
  14 轴 q/qdot suffix，再发布 RollingJointTargetBatch。

**生产者规则**：

- 只基于 last_accepted_sequence 对应的本地缓存轨迹继续生成；未收到 ack 时可跳过一拍或选择
  更晚 R，不把 rejected batch 当作 validation head。
- 非伺服轴按上一条已接受轨迹在各 future knot 时间采样，不能用接收瞬间 /joint_states 常数
  覆盖整个 suffix。
- R 必须不早于最新 state 的 replaceable_from，并由 Motion 自己增加状态发布、计算和 DDS
  裕量；100 ms 是 knot 间隔，不要求 R 对 session 的全局 100 ms 网格对齐。具体 producer
  margin 由 Motion 和 T19 决定，RT 不替它猜。
- reject 后使用新 sequence 发送完整自包含修正版；不重放已消耗 sequence。
- 视觉丢帧时健康的 Motion 继续发布自己决定的保持／降速 future；只有 Motion 输入流真正停止
  才让 RT timeout/low-water 兜底。

### E102-D27 — 原型文档迁移与 BQ 编号冲突

**问题**：是否把 /home/kkozia/robot 的四份 E102 文档原样复制到当前 robot_driver？

- **【备选 A】** 原样复制。速度快，但旧文档的 dual_arm_jtc、50 Hz 以及 BQ-130／BQ-131
  会与当前 main 的 whole_body_jtc、125 Hz 和已占用 BQ 编号冲突。
- **【推荐／最终采用 B】** 原型文档只作为历史输入，逐份按当前 main 重写／校正；不得把旧
  BQ 编号当当前权威。实施阶段需要新增 BQ 时从当前下一个可用编号 BQ-138 起分配，并建立
  当前 contract record，不回写覆盖现有 BQ-130～137。

**影响**：所有 dual_arm_jtc 改为 whole_body_jtc；E102 文档中的 50 Hz 改为 125 Hz 并引用
现有 2026-08-13 契约记录；旧 R-OUT-03 手工表改成 robot_interfaces 权威 registry／view。
今晚只维护本文这一份裁决文件，避免出现两份同时自称最终决策。

### E102-D28 — “可供 Motion 测试”和接口 PR 的门槛

**问题**：IDL 能 build 就提接口 PR，还是等真实硬件，或在完整 mock checkpoint 提交？

- **【备选 A】** IDL 编译后立即提 PR。太早，不能证明字段足以实现生产者和错误恢复。
- **【备选 B】** 等真实硬件全部验证。太晚，跨域评审和 Motion 软件联调会被不必要阻塞。
- **【推荐／最终采用 C】** 达到完整 mock/fake 软件门后允许接口 PR：
  1. robot_interfaces feature branch 的 IDL、registry、CHANGELOG、C++／Python QoS 全绿；
  2. driver 对精确接口 SHA 的 clean multi-repo build 全绿；
  3. 协议向量、Hermite/extrema/stop、prefix immutability、history compaction、mode/fault
     矩阵和 JTC 回归全绿；
  4. 250 Hz fake 至少 10 分钟，Motion mock 30 Hz／100 ms，point_count 有界，RT allocation
     trap 零命中；
  5. 最小 public-IDL-only producer 可完成 cancel/wait、mode、open、prime、update、
     REQUEST_STOP、FINALIZE、return-to-FJT；
  6. 每场景输出 JSON/JUnit，失败保存 seed、trace 和 report。

**性能证据**：T19 报告 validation p50/p99/p99.9、callback→RT visibility、DDS 延迟、
LateReplace、STRICT switch 分布和复制字节。nominal 长稳要求 LateReplace=0；注入场景按预期
拒绝。16 ms／500 ms 等仍标 provisional，目标机值不能用桌面测量直接升级为 production。

### E102-D29 — 分支、提交和 PR 粒度

**问题**：是否为每个任务创建子分支／PR？

- **【备选 A】** 每任务一个子分支和 PR。隔离强，但大量串行 PR 会让三个仓库的中间态无法
  一起构建。
- **【推荐／用户最终采用 B】** 每个仓库各一个中文功能分支；每个任务一个可独立 review、
  可回滚的原子 commit，不创建任务子分支。达到对应 checkpoint 后按仓库提 PR。

**当前状态**：robot_driver 使用 功能/视觉伺服-ELECTRI-102。robot_interfaces 与
robot_description 实施时也各自从权威 main 创建一个中文功能分支。接口 PR 受 D28 门约束；
本任务不自动创建任何 PR，也不进行目标机部署。

### E102-D30 — 明确延期项与禁止隐式扩域

以下内容不是未完成的软件选择，而是明确不在本期或需要外部证据：

- TF 动态发布频率提升、相机／工控机时钟同步和图像时间戳对齐；
- CSV mode 9 或 velocity command interface；
- Motion 同时伺服双臂的策略；
- 左手独立手眼标定；
- production envelope 台架停车／跟踪实测；
- 目标机 QoS／replace lead／switch latency 最终冻结；
- robot_interfaces 0.8.0 正式发布和四域同 SHA 升级；
- 任何真实总线启动、reset、enable、FJT、rolling 或实机运动。

这些项若将来进入范围，必须新建证据记录／授权，不得借 ELECTRI-102 当前软件分支默认获得
许可。

### E102-D31 — rolling 细粒度结果到公共 DREE 的映射

**问题**：`RollingServiceResult` 已负责 rolling 协议分支，配套的公共 `ErrorInfo` 应按每个
细节另造 DREE 码、全部压成一个通用拒绝码，还是复用现有公共类别？

- **【备选 A】** 为 20 个拒绝原因各增加一个 RT-Control DREE 码。表达最细，但会把
  rolling 私有状态复制进跨域公共错误空间，且扩大本期接口变更。
- **【备选 B】** 全部映射为 `GOAL_REJECTED`。实现最小，但 `retryable`、配置错误、安全拒绝
  和暂态未就绪无法区分。
- **【推荐／最终采用 C】** 程序精确分支继续读取 `RollingServiceResult`；`ErrorInfo` 只映射
  到现有 DREE 类别，并逐项测试锁定：

| RollingServiceResult | ErrorInfo.code | retryable |
| --- | --- | --- |
| NONE | SUCCESS | false |
| WRONG_PROTOCOL | VERSION_MISMATCH | false |
| WRONG_REQUEST / WRONG_CLIENT / AXIS_SET_MISMATCH | INVALID_GOAL | false |
| WRONG_BOOT / WRONG_SESSION | RETRYABLE_INVALID_GOAL | true |
| NOT_ENABLED / SESSION_BUSY / SESSION_EXISTS / SOURCE_STATE_STALE / SOURCE_MOVING / FEEDBACK_STALE / SWITCH_TIMEOUT / NOT_READY | NOT_READY | true |
| WRONG_MODE / SWITCH_REJECTED | GOAL_REJECTED | false |
| TAKEOVER_MISMATCH | RT_TOLERANCE_VIOLATED | false |
| UNSAFE_HOLD | SAFETY_DENIED | false |
| LIMITS_UNAVAILABLE / RESTART_REQUIRED | VERSION_MISMATCH | false |
| 未知枚举值 | INTERNAL_ERROR | false |

`source` 固定为 `rt_control`，`detail` 只供诊断并带原始枚举值；Motion 不解析字符串。这里的
“可重试”表示机器在外部状态变化或重新 open 后可以形成新请求，不表示可原样重放同一
sequence/request。

### E102-D32 — RT 零分配门禁的计数归属

**问题**：同一 gtest 进程中有 ROS/DDS 后台线程时，零分配门禁统计整个进程还是只统计执行
`update()` 的线程？

- **【备选 A】** 全进程全局计数。会把发现节点、日志等非 RT 线程的合法分配误报为 RT
  分配，完整套件中出现时序相关假阳性。
- **【推荐／最终采用 B】** 分配钩子仍覆盖所有 `new` 变体，但 tracking flag 和 counter
  使用 `thread_local`，只在直接调用 250 Hz `update()` 的测试线程内打开。
- **【备选 C】** 测试时停掉所有 executor/DDS 线程。隔离更强，但不能覆盖控制器在真实
  ROS 并发背景下的 update 路径。

采用 B 后，RT 路径真实分配仍必然命中；其他线程不会污染计数。完整 11 项 CTest 与单测
结果一致，避免“单独运行通过、全套运行偶发失败”。

### 6.2 实施顺序裁决

为了避免在错误数据结构上做性能优化，后续原子提交按以下因果顺序推进：

1. 当前决策记录与原型／main 差异门；
2. robot_interfaces 公共 IDL／QoS 功能分支；
3. robot_description 权威外参与 driver 构建副本同步，以及 JTC topic 旁路关闭；
4. rolling 原型移植，但先修 Prime anchor、显式 splice 左右边界、max horizon 和历史裁剪；
5. 删除冗余验证、增量后缀验证、有效点复制和 RT 单调 cursor；
6. YAML 参数、provisional envelope 和 public state；
7. enable_manager registry、source evidence、mode service 与失败收敛；
8. mandatory-inactive bringup、Motion mock、异常矩阵、10 分钟长稳和性能标定；
9. Motion 交接、当前 BQ／contract records；满足 D28 后才准备接口 PR。

在第 4 步正确性不变量全绿之前，不做 T19 数字冻结；否则测到的是会重写前缀或耗尽历史的
错误实现。

## 7. 其他已冻结输入

- `/joint_states` 契约频率采用实测 125 Hz；不修改
  `joint_state_broadcaster.update_rate: 100` 的冻结配置。
- 手眼外参进入 `robot_description` 版本化安装模型；右手独立数据、左手保留
  `calibration: mirror_estimate_of_right`，双臂精度分别记录。
- TF 频率、相机/工控机时钟同步、双臂同时伺服策略、左手独立标定、生产 envelope
  台架实测及 `robot_interfaces 0.8.0` 正式提升仍不属于本期软件闭环。
