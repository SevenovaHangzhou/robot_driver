# V3 机械臂 RT-Control 与 Motion 对接方案

本文定义三代机双七轴机械臂的 RT-Control/Motion 责任边界、目标接口、调用顺序和
准入门禁。远端基线仍是
`robot_driver/v3@1256885ce3b99c794bc9ea68d4b353f880166337`；本轮实现位于尚未提交的
`feature/v3-arm-motion-runtime`，公共接口 pin 已升级为
`robot_interfaces@9aa2693d7d3235958369272b7ce8c48592dd7e83`。

## 1. 先看结论

远端 `v3@1256885` 和旧工控机旁路 release **还不能接收机械臂轨迹命令**。本轮实现分支
已经补齐软件链路，但尚未提交、合并或部署：

- 默认 `rt_control_start` 只执行 machine-profile validation，不创建硬件节点；
- `arms_only` 已冻结 14 个 CSP 臂轴和 2 个 PP 夹爪的拓扑/模式目标；
- `rt_control_enable_only.launch.py` 只用于经单独授权的原位使能，不接收运动目标；
- 新增 `rt_control_arm_runtime.launch.py`，以 1 kHz 加载 14 轴
  `whole_body_jtc`、rolling controller 和 16 轴 `enable_manager`；
- `/rt/enable` 在真实驱动使能成功后默认激活 JTC；JTC 与 rolling 任一时刻最多一个 ACTIVE；
- 两个 PP 夹爪只进入 CiA402 使能/保持，不被 JTC 或 rolling 认领；
- Mock 已验证完整 14 轴 FJT、STRICT JTC->rolling 切换和 rolling state 发布；
- 真实模式仍会因 14 个 CSP 轴方向未验证而 fail-closed，不能据此执行实机运动。

V3 目标方案保留两种互斥的 14 轴控制模式：

```text
DISABLED
   |
   v
FJT_READY  <----严格切换---->  ROLLING_READY
   |                              |
完整批量轨迹                    短时域滚动轨迹
FollowJointTrajectory           session + suffix update
```

两种模式都只控制左右臂 7+7 个 CSP 轴。两个 PP 夹爪独立管理，不进入 14 轴
FJT 或 rolling 数组。

## 2. 状态标签

本文使用以下标签，联调时不得混淆：

| 标签 | 含义 |
| --- | --- |
| `BASELINE` | 远端 `v3@1256885` 和旧工控机旁路 release 的已部署状态 |
| `IMPLEMENTED` | `feature/v3-arm-motion-runtime` 已实现并通过本地构建/Mock 的行为 |
| `TARGET` | 仍需合并、部署或真实硬件验证后才能使用的行为 |
| `BLOCKED` | 缺少硬件、标定、公共契约或 runtime 组合，不允许绕过 |

## 3. 责任边界

| Motion 负责 | RT-Control 负责 |
| --- | --- |
| MoveIt/IK、碰撞检查、路径与时间参数化 | ros2_control controller、实时采样和 CSP/PP 命令写出 |
| 生成完整 14 轴 FJT 或 rolling future | 校验轴集合、时间、连续性、动态包络和反馈新鲜度 |
| 取消自己创建的 goal，并等待终态 | controller 生命周期、唯一 command writer 和严格模式切换 |
| rolling 时维护最后一条确认接受的轨迹 | rolling session、buffer、插值、低水位/超时受控停车 |
| 视觉丢失后的业务降速/退出策略 | 总线故障、驱动故障、失能、停机和诊断 |
| 消费安全状态并禁止不安全的新动作 | 发布关节状态、安全摘要和稳定错误原因 |

Motion 不得直接访问 EtherCAT/PDO、`control_word`、controller_manager 切换服务或
RT-Control 域私有 `/rt/enable`、`/rt/disable`、`/rt/reset_fault`。

## 4. V3 机械臂资源合同

### 4.1 目标控制轴序

`IMPLEMENTED` 的 FJT 与 rolling 固定轴序为：

| Index | Joint | 单位 |
| ---: | --- | --- |
| 0 | `right_joint1` | rad / rad/s |
| 1 | `right_joint2` | rad / rad/s |
| 2 | `right_joint3` | rad / rad/s |
| 3 | `right_joint4` | rad / rad/s |
| 4 | `right_joint5` | rad / rad/s |
| 5 | `right_joint6` | rad / rad/s |
| 6 | `right_joint7` | rad / rad/s |
| 7 | `left_joint1` | rad / rad/s |
| 8 | `left_joint2` | rad / rad/s |
| 9 | `left_joint3` | rad / rad/s |
| 10 | `left_joint4` | rad / rad/s |
| 11 | `left_joint5` | rad / rad/s |
| 12 | `left_joint6` | rad / rad/s |
| 13 | `left_joint7` | rad / rad/s |

该顺序延续“右臂在前、左臂在后”的约定。V3 `axis_set_hash` 的规范输入为每轴一行
`<joint_name>:rad:rad/s\n`，SHA-256 为
`f4c8ff8a32183d1733032494600c9f5d1e7e625673a4ad51556b3ac6c35adee4`。
ELECTRI-102 的二代 hash 对应 `right_joint1..6,left_joint1..6,turn,updown`，禁止复用。

### 4.2 物理拓扑与接口顺序是两件事

`BASELINE` 的 arms-only 物理顺序为：

- 汇川分支器主设备：ring 0；子设备：ring 17；
- 左臂 X2：ring 1..7 为 J1..J7 CSP，ring 8 为 PP 夹爪；
- 右臂 X3：ring 9..15 为 J1..J7 CSP，ring 16 为 PP 夹爪。

公共数组顺序不需要等于 EtherCAT ring 顺序。硬件 owner 负责把 ring position 映射到
Robot Model joint；Motion 只使用冻结的逻辑 joint 名称。

### 4.3 PP 夹爪

两个夹爪逻辑关节是：

- `left_moving_jaw_joint`
- `right_moving_jaw_joint`

PP 控制器的 joint、位置范围、力、超时和 stall 参数仍为 `TBD`。本轮 arm runtime 不加载
gripper controller，也不暴露 V3 PP 夹爪运动命令。

因此：

- PP 夹爪不加入 14 轴 FJT/rolling；
- Motion/Autonomy 不得直接调用私有 draft controller；
- 对外开放前必须在 `robot_interfaces` 明确 endpoint、owner、单位、取消和错误语义；
- 夹爪多圈基准、机械行程、最大力和 Halt 行为通过实机后才能解除门禁。

## 5. 批量轨迹模式 FJT

### 5.1 目标 endpoint

| 状态 | Endpoint | 类型 | Owner |
| --- | --- | --- | --- |
| `IMPLEMENTED` | `/whole_body_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion -> RT-Control |
| `IMPLEMENTED` | `/joint_states` | `sensor_msgs/msg/JointState` | RT-Control -> Motion |
| `TARGET` | `/control/safety_state` | `robot_rt_control_interfaces/msg/SafetyState` | RT-Control -> Motion |

R-IN-02 的 ROS 类型和名称可保持不变，但 V3 必须同步改变其行为语义：14 轴从二代的
12 臂轴+Turn+Updown 变为 14 个臂轴。所有消费者必须锁定同一 `robot_interfaces` SHA
并联合验证；“数量仍为 14”不等于行为兼容。

### 5.2 Goal 准入

V3 JTC 应至少满足：

1. `joint_names` 必须精确包含固定 14 轴，不允许缺轴、重复或额外轴；
2. `allow_partial_joints_goal=false`；
3. position 使用 rad，时间严格递增且所有数值有限；
4. 第一轨迹点必须与新鲜实际反馈一致；每轴容差由 V3 标定配置提供；
5. EtherCAT feedback age 超限时拒绝新 goal；
6. 每个点必须满足 V3 位置、速度和加速度合同；不得复用二代 provisional limits；
7. 只有 `SafetyState.safe_to_start_motion=true` 且消息未过期才允许 Motion 发新 goal；
8. 同时只能存在一个拥有 14 轴 position command interface 的 motion controller。

Mock 已验证完整全零 14 轴 FJT 可接受并正常完成。真实 runtime 的 raw zero 与
524288 counts/output-rev 比例幅值已有配置，但逐轴正方向、动态包络和停车参数仍需实机验证，
因此真实运动仍为 `BLOCKED`。

### 5.3 调用顺序

```text
Motion                         RT-Control
  |                                |
  |-- wait fresh SafetyState ----->|
  |-- send complete 14-axis goal ->| whole_body_jtc
  |<----- accepted/rejected -------|
  |<----- feedback ----------------|
  |<----- final result ------------|
```

- Motion 只取消自己持有的 goal；
- cancel accepted 不是终态，必须等待 Action result；
- Action result 是该轨迹成功/失败的权威结果；`/diagnostics` 不能替代 result；
- RT-Control disable、驱动 fault 或总线 fault 必须终止活动 goal，不允许恢复后自动重放。

## 6. Rolling 视觉伺服模式

### 6.1 来源与实现边界

`TARGET` 方案继承 ELECTRI-102 的控制模型：Motion 不直接发送 `TwistStamped`，而是
以固定频率提交完整 14 轴的短时域 position/velocity future suffix；RT-Control 在实时线程
内插值并继续写 CSP position。

本轮实现分支已迁入 `rolling_trajectory_controller`、FJT/rolling 严格切换状态机、公共
IDL/QoS pin、V3 轴序 hash、临时低速 envelope 和 arm runtime launch。该实现已通过本地
Mock，但尚未进入远端 V3 或旧工控机 release；临时 envelope 标记为
`ESTIMATED_NOT_MEASURED`，不得用于授权真实硬件运动。

### 6.2 目标接口

| 方向 | Endpoint | 目标类型 |
| --- | --- | --- |
| Motion -> RT | `/rt/joint_control/set_mode` | `SetJointControlMode` service |
| Motion -> RT | `/rt/rolling_joint_control/open` | `OpenRollingJointSession` service |
| Motion -> RT | `/rt/rolling_joint_control/update` | `RollingJointTargetBatch` topic |
| Motion -> RT | `/rt/rolling_joint_control/close` | `CloseRollingJointSession` service |
| RT -> Motion | `/rt/rolling_joint_control/state` | `RollingJointControlState` topic |

本轮实现 pin `robot_interfaces@9aa2693`，包含这些 wire schema 和 QoS。该上游提交的消息注释
仍沿用二代 14 轴语义，V3 轴语义暂由本文、runtime 固定顺序和 `axis_set_hash` 共同约束；
正式跨域发布前必须在 `robot_interfaces` 更新注释/契约并让 Motion 与 RT-Control 原子升级。

### 6.3 建议会话流程

1. Motion cancel 自己的 FJT goal，并等待最终 result；
2. `set_mode(expected=FJT_READY, target=ROLLING_READY)`；
3. 校验 `accepted=true`、source controller 已停、rolling controller 已 active 且
   `restart_required=false`；响应中的 boot ID 可能暂时为零；
4. 等待本次切换之后发布的 `/rt/rolling_joint_control/state`：模式为 `ROLLING_READY`、
   轴数为 14、boot ID 非零；超过客户端截止时间仍无状态时不得 `open` 或下发轨迹；
5. `open` 携带 protocol、client/request ID、该状态的 boot ID 和 V3 axis hash；
6. 使用 open 返回的 hold 与 `initial_replaceable_from_ns` 立即发送静止 prime suffix；
7. 只有 state 的 `last_accepted_sequence` 到达本批 sequence 才算接受；
8. 正常阶段持续发布完整 14 轴 future suffix，替换点应留出通信与状态采样余量；
9. 退出时先 `close(REQUEST_STOP)`，等待同一 session 进入 `HOLDING`；
10. 再 `close(FINALIZE)`，等待 `has_session=false`；
11. `set_mode(expected=ROLLING_READY, target=FJT_READY)`。

任何 boot/session/client 不匹配、超时、低水位、fault 或 restart-required 都必须丢弃旧
session 和未确认 batch，禁止自动重放。切换后状态超时应进入明确的停止/恢复流程，
不能把仅有 controller_manager 的 active 结果当作可开会话的证明。

### 6.4 Motion 的 future 生成责任

可用 ELECTRI-102 作为初始工作点：

- batch 发布约 30 Hz；
- knot 间隔约 100 ms；
- 计划 future 约 500 ms；
- 每包始终携带完整 14 轴；
- 当前只视觉伺服一条臂，另一条臂仍从“最后确认接受的 future”按相同未来时刻采样。

禁止用 `/joint_states` 当前值填满非伺服臂的未来点，这会在 splice 位置把仍在执行的轴
拉回当前实测位置。publish 成功也不代表 RT 已接受，必须等 state sequence 证据。

视觉短时丢失属于 Motion 业务策略：继续发布有界减速/hold suffix。RT 的 update timeout
只兜底 producer 或 DDS 真正失联。

### 6.5 V3 必须重新标定的时间参数

本轮实现已把 controller_manager、nominal period 和 one-cycle guard 调整为 1 kHz/1 ms，
maximum period 为 2 ms；这些是 Mock/初始软件值，不是目标机实测结论。下列值仍必须测量：

- nominal/maximum controller period；
- one-cycle detection、non-RT-to-RT visibility 和 period quantization guard；
- replace lead；
- update timeout 与最低可停车 horizon；
- state publish period、prime timeout；
- 每轴 takeover/splice tolerance；
- 每轴速度、加速度、停车加速度和 position margin。

30 Hz/100 ms/500 ms 只作为 Motion 初始方案，最终值由 V3 Mock、目标机调度数据和低速 HIL
共同冻结。

## 7. 状态与错误处理

### 7.1 当前公共状态

| Endpoint | Motion 使用方式 |
| --- | --- |
| `/joint_states` | 目标语义为 14 个 CSP 臂轴 position；消息年龄上限沿公共契约为 200 ms |
| `/control/safety_state` | `safe_to_start_motion=false` 或消息过期时禁止新动作 |
| `/diagnostics` | 运维诊断；不得替代 Action result、rolling state 或硬安全链 |

`SafetyState` 是软件可观测摘要，不代表急停、安全继电器或 STO 的物理状态。Motion 不得
根据单个 `ethercat_ok` 或字符串自行推导 motion admission，应使用明确的
`safe_to_start_motion` 和当前命令接口终态。

### 7.2 Rolling 错误类别

迁移后 Motion 至少要区分：

- 身份类：wrong protocol/boot/session/client/axis set；丢弃本地 session，禁止重放；
- 排序类：stale sequence、late replace、non-monotonic time；读取最新 state 后重新计算；
- 连续性类：position/velocity discontinuity；从最后 accepted future 在替换点重采样；
- 包络类：position/velocity/acceleration/not-stopping-viable；重新规划，不由 RT clamp；
- 生命周期类：not enabled、wrong mode、switch timeout、restart required；停止新命令并人工恢复。

字符串只用于日志。Motion 的状态机应依据枚举结果、`ErrorInfo.code`、`retryable` 和 public
state，而不是匹配错误文本。

## 8. Physical profile 与使能范围

- 双臂台架使用 `physical_profile=arms_only, control_scope=arms_only`；CANopen 和达妙 CAN
  为 `not_required`，缺失不能误报为 fault。
- V3 当前策略规定 `full_robot` 只能选择 `full` scope；完整物理环存在时所有执行器进入
  使能集合，不提供“物理全装但只使能机械臂”的子模式。
- Motion 可以只向机械臂 controller 发命令，但不能改变 enable_manager 的整机使能策略。
- 主动悬挂 fault -> 舵轮 stop-and-inhibit；舵轮 fault -> 机械臂 stop。当前只有静态合同，
  runtime 传播仍未接通。

## 9. 当前旧工控机部署坐标

本节只记录旁路部署，不改变当前生产身份：

| 项目 | 值 |
| --- | --- |
| 机器角色 | 旧工控机 / V3 测试目标 |
| SSH | `ar@192.168.100.40` |
| Host / kernel | `ar-Default-string` / `5.15.0-1032-realtime` |
| V3 release | `/home/ar/rt-control-v3/releases/1256885` |
| Source SHA | `1256885ce3b99c794bc9ea68d4b353f880166337` |
| Source bundle SHA-256 | `064a5ff1db27e4b1afa70f6b6785416463528c6e5283bfe3b35c8a46b5478576` |
| 构建 | 34 packages PASS |
| 测试 | 1697 tests，0 errors/failures，23 skipped |
| 静态入口 | arms-only validation PASS，14 CSP + 2 PP |
| 当前 release link | 未修改，仍指向 `rt-control-operators/51f603.../robot` |

本次没有执行 `doctor`、真实 launch、reset、enable、SDO write 或运动。V3 的 host identity
锁定当前新工控机，旧工控机只能作为旁路构建/测试目标，不得删除身份门禁后强行运行。
本轮 arm runtime 实现尚未部署到上述 release。

## 10. 实施顺序与验收门

### Phase A：公共契约

1. 在 `robot_interfaces` 正式更新并冻结 V3 14 轴名称/顺序/单位；
2. 更新 R-IN-02 和 R-OUT-03 的 V3 行为语义；
3. 保持 rolling schema/QoS，并发布 V3 语义与 axis hash；
4. 明确 PP 夹爪的跨域 owner 和 endpoint；
5. Motion、RT-Control、Perception、Autonomy 原子 pin 同一接口 SHA。

### Phase B：RT-Control runtime

1. 完成 14 CSP + 2 PP 正式 slave profile、缩放、方向、零位和限位；
2. 生成 V3 ros2_control hardware description；
3. `whole_body_jtc`、rolling 和 V3 enable_manager 已接入；PP controller 延后；
4. FJT Mock 和严格模式切换已完成，下一步是无运动真实使能与单轴低速验证；
5. 用目标机数据替换 provisional rolling 参数；
6. 接通故障依赖、readiness 和有序停机。

### Phase C：Motion 联调

1. 只用公共接口和固定 QoS；
2. 验证完整 14 轴 FJT、取消、拒绝、反馈过期和 restart；
3. rolling 依次验证 prime、30 Hz update、splice、reject、低水位、timeout、close/finalize；
4. 先单臂视觉伺服，非伺服臂保持 accepted future；
5. 完成急停、断总线、controller restart 和最终停机验收后才进入生产。

## 11. 明确禁止

- 不向驱动或 RT-Control 直接发送末端 `TwistStamped`；
- 不把机械臂切为 CSV 以绕过 rolling trajectory；
- 不发 partial FJT 或单关节旁路命令；
- 不由 Motion 调 controller_manager 或域私有 enable/reset 服务；
- 不在反馈过期、safety false、mode/session 不明时发送新命令；
- 不复用 ELECTRI-102 的旧 axis hash、4 ms guard 或二代 dynamic envelope；
- 不把旧工控机上的 validation-only 成功描述成三代机真实轨迹控制通过。
