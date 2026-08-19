# ELECTRI-102 Motion 联调与生产端最小实现指南

> 适用范围：视觉伺服的单臂 rolling trajectory producer。
> 当前状态：公共接口与隔离 mock 已验证；真实相机、TF 标定、目标机和硬件运动未授权。

## 1. Motion 最终要实现什么

Motion 不向驱动持续发送 `TwistStamped`，也不切换到 CiA402 CSV。视觉控制律先在 Motion
内转换成关节空间目标，再持续替换一段短的、完整 14 轴 position/velocity future suffix：

- 控制律／图像更新可按实际相机节奏运行；本期 producer 每 100 ms 放一个轨迹 knot；
- Motion 以 30 Hz 重新计算并发布整个 future suffix；
- RT-Control 以 250 Hz 对 cubic Hermite 轨迹采样，只写 CSP position command；
- 同一时刻 Motion 只伺服一条臂，另一条臂、Turn、Updown 仍必须逐点填满；
- FJT 与 rolling 互斥，同一拍只允许一个 controller 持有 14 轴 command writer。

这三个频率分别是 knot 时间参数化、批次更新和驱动采样，不能互相替代。

## 2. 公共接口和固定轴序

Motion 只依赖 `robot_interfaces` 的公开包：

| 方向 | Endpoint | 类型 |
| --- | --- | --- |
| Motion → RT | `/rt/joint_control/set_mode` | `robot_rt_control_interfaces/srv/SetJointControlMode` |
| Motion → RT | `/rt/rolling_joint_control/open` | `robot_rt_control_interfaces/srv/OpenRollingJointSession` |
| Motion → RT | `/rt/rolling_joint_control/update` | `robot_motion_interfaces/msg/RollingJointTargetBatch` |
| Motion → RT | `/rt/rolling_joint_control/close` | `robot_rt_control_interfaces/srv/CloseRollingJointSession` |
| RT → Motion | `/rt/rolling_joint_control/state` | `robot_rt_control_interfaces/msg/RollingJointControlState` |

Topic 必须调用 `robot_interfaces_qos.rolling_command()` 和 `rolling_state()`，禁止在 Motion
自行拼一份“差不多”的 QoS。协议当前为 1.0。

每个 point 固定 14 轴，顺序和单位如下：

| Index | 名称 | position / velocity 单位 |
| ---: | --- | --- |
| 0–5 | `right_joint1..6` | rad / rad/s |
| 6–11 | `left_joint1..6` | rad / rad/s |
| 12 | `turn` | rad / rad/s |
| 13 | `updown` | m / m/s |

轴序 SHA-256 固定为：

```text
25c6e82bf505ca9eb99db1c645ab75d7ecde0153faaf6a7492c6210c4d362526
```

Motion 必须在 open 请求中原样填写该 digest，并校验 response 回报完全一致。

## 3. 一次完整会话

### 3.1 切入 rolling

1. Motion 只取消自己持有的 FJT goal，并等待该 Action goal 的最终 result；只收到 cancel
   accepted 或 status 变化不等于终态。
2. 订阅 rolling state，创建唯一 command publisher，确认恰好一个 update consumer。
3. 调 `set_mode(expected=FJT_READY,target=ROLLING_READY)`。每次逻辑请求生成新的
   `request_id`；只在网络重试同一请求时复用相同 client/request ID 和完全相同 payload。
4. 只有 response 同时满足以下条件才继续：`accepted=true`、`result=NONE`、
   `mode=ROLLING_READY`、source 已停用、target 已激活、`restart_required=false`、boot ID 非零。
5. 若 Motion 重启，生成新的 `client_instance_id`，不要复用上一个进程的 session 身份。

`set_mode` 是 compare-and-set，不会替 Motion cancel FJT，也不会自动重试。失败时先按 response
和公共 state 确认实际 mode；禁止猜测 switch 已成功。

### 3.2 Open 与立即 Prime

Open 请求携带 protocol、client/request ID、刚收到的 controller boot ID 和 axis hash。
接受后必须校验：

- response 是 protocol 1.0、client ID 与本进程一致、session/boot ID 非零；
- `session_state=PRIMING`，limits source、test-only 标志和 limits version 可见；
- capacity、required/max horizon、replace lead、update timeout、controller period 都取 response
  生效值，不能只使用 Motion 本地默认；
- 当前软件期望值来自 `hold_positions/hold_velocities`，不是 open 后再读一次 `/joint_states`。

Open 后 `prime_timeout` 当前只有 100 ms，不要等待下一帧视觉。立即使用 hold 生成首批：

```text
R = initial_replaceable_from_ns
t = R, R+100ms, R+200ms, R+300ms, R+400ms, R+500ms
q(t) = hold_positions
qdot(t) = 0
```

在 100 ms knot 下，point 数按
`ceil(required_initial_horizon / 100ms) + 1` 计算，并同时满足 `buffer_capacity`、
`transport_max_points` 和 `max_horizon`。当前配置得到 6 点、500 ms future。`capacity=64`
表示 controller 最多保存 64 个活动端点，不是要求每批发 64 点。

Prime 只有在 state 的 session ID 一致、`RUNNING` 且 `last_accepted_sequence` 到达本批 sequence
后才算成功。不能把 DDS publish 返回当成 RT 接受。

### 3.3 30 Hz 更新

每次读取同一个 state snapshot：

1. `replace_from_ns` 取不早于 `state.replaceable_from_ns` 的时间；不要从 ROS wall time换算。
2. 用当前已确认接受的本地轨迹在 R 精确采样旧 `q/qdot`，新 suffix 第一点必须与它连续。
3. 以 100 ms knot 生成到约 R+500 ms；候选最后一点相对 coherent execution time 不得超过
   open 回报的 max horizon。
4. `sequence` 每次尝试严格递增。即便上一批被拒绝，该 sequence 也已经消费，不能重放。
5. 只有 public state 确认 `last_accepted_sequence` 后，才把该 candidate 提升为 Motion 的
   “已接受轨迹”；pending 或 publish 成功都不能覆盖它。

非伺服轴的填写规则是本合同最容易出错的地方：对每个未来 knot 时间，采样“上一条已确认
接受的轨迹”在同一时间的对应轴值。不能把一次 `/joint_states` 当前值复制到所有未来点；
那会在 splice 处把原本仍在执行的非伺服轴拉回当前实测位置。

### 3.4 控制律整形

IBVS/PBVS 输出的原始 `qdot_target` 不能直接写成新 suffix 首点速度。首点必须沿用旧轨迹的
`q(R),qdot(R)`，随后才按 Rolling provisional 包络进行速度／加速度整形。参考逻辑：

```python
q[0], v[0] = accepted.sample(R)          # 14 轴精确 splice
for k in range(1, 6):
    dt = 0.100
    target = visual_servo_qdot(k * dt)   # 只覆盖本次伺服臂
    for j in servo_arm_indices:
        target[j] = clamp(target[j], -v_limit[j], v_limit[j])
        dv = clamp(target[j] - v[k-1][j],
                   -a_limit[j] * dt, a_limit[j] * dt)
        v[k][j] = v[k-1][j] + dv
        q[k][j] = q[k-1][j] + 0.5 * (v[k-1][j] + v[k][j]) * dt
    for j in non_servo_indices:
        q[k][j], v[k][j] = accepted.sample_axis(j, R + k * 100_ms)
```

上例只说明责任边界，不替 Motion 选择视觉控制律、IK/null-space、碰撞约束或丢帧降速策略。
Motion 还必须逐点检查 position limit、directional velocity/acceleration 和停车可行性。当前
Rolling provisional 上限为：旋转轴 15 deg/s、0.75 rad/s²；Updown 0.09 m/s、0.5 m/s²。
这些值不影响普通 FJT/JTC。

已接受的大 splice 容差只是 RT 拒绝上界，不是建议误差：旋转 `0.01 rad / 0.03 rad/s`，
Updown `0.003 m / 0.02 m/s`。Motion 应尽量做到数值连续，并在联调日志逐批记录
`delta_q/delta_qdot`。

### 3.5 视觉丢帧不等于 producer 失联

图像短时丢失后，Motion 仍以 30 Hz 发布可执行 suffix：沿已接受轨迹有界减速到 hold，或按
Motion 自己批准的状态机保持。不要停发并依赖 RT 的 200 ms `update_timeout` 代替业务决策；
该 timeout 只兜底 producer/DDS 真正失联。低水位或 timeout 触发后，旧 session 不能靠恢复
发布“复活”，必须按 state 完成 close/finalize 并开 fresh session。

### 3.6 正常退出

1. `close(REQUEST_STOP)` 只代表停止阶段已接受，通常 `completed=false`。
2. 持续订阅 state，等同一 session 进入 `HOLDING`；不要在 `STOPPING` 时直接 finalize。
3. `close(FINALIZE)` 成功且 `completed=true` 后，等待 `has_session=false`。
4. 调 `set_mode(expected=ROLLING_READY,target=FJT_READY)`，校验 source/target 和 restart 证据。
5. 之后 Motion 才可提交新的 FJT goal。

退出、disable、group fault 或 restart-required 期间都不得自动重放旧批次。

## 4. RejectCode 与 Motion 响应

| 值 | 名称 | Motion 正确响应 |
| ---: | --- | --- |
| 0 | `NONE` | 无拒绝；仍用 accepted sequence 判断本批是否生效。 |
| 1 | `WRONG_PROTOCOL` | 停止会话；升级到同一接口版本，禁止降级猜测。 |
| 2 | `WRONG_BOOT` | controller 已重启；丢弃全部本地 session/轨迹身份，重新 set-mode/open。 |
| 3 | `WRONG_SESSION` | 丢弃旧 session，读取 state 后 fresh open；不重放旧批。 |
| 4 | `WRONG_CLIENT` | client identity 冲突；停止当前 producer，排查多 writer。 |
| 5 | `STALE_SEQUENCE` | sequence 已使用或乱序；取 state 的 last seen/accepted，使用更大新 sequence。 |
| 6 | `INVALID_SHAPE` | producer bug；停止发布，检查 point 数、固定 14 轴数组和 ID。 |
| 7 | `NON_FINITE` | producer bug；拦截 NaN/Inf，不允许修成 0 后盲重试。 |
| 8 | `NON_MONOTONIC_TIME` | 重建严格递增 knot；同时间只允许协议定义的内部 splice pair，不由 wire 构造。 |
| 9 | `LATE_REPLACE` | 读取最新 `replaceable_from_ns`，用新 sequence 重算完整 suffix；记录一级指标。 |
| 10 | `TIME_GAP` | 缩小相邻 knot gap 或补点；当前冻结值应保持 100 ms。 |
| 11 | `CAPACITY_EXCEEDED` | 缩短／减少活动点，检查是否错误携带历史；不要提高 capacity 掩盖泄漏。 |
| 12 | `INSUFFICIENT_HORIZON` | 延长 future 到 open 回报的 required horizon；Prime 当前需 500 ms。 |
| 13 | `POSITION_DISCONTINUITY` | 从已接受轨迹在 R 重新采样首点，修正 q splice。 |
| 14 | `VELOCITY_DISCONTINUITY` | 从已接受轨迹在 R 重新采样首点，修正 qdot splice／整形。 |
| 15 | `POSITION_LIMIT` | 重新规划或停止；禁止重复同一越界 candidate。 |
| 16 | `VELOCITY_LIMIT` | 在 Motion 降速并重算整个 suffix；RT 不负责 clamp。 |
| 17 | `ACCELERATION_LIMIT` | 降低速度变化率／加长整形时间后重算。 |
| 18 | `NOT_STOPPING_VIABLE` | 降速、增加可停车 future 或远离 position margin；不能靠放宽 margin。 |
| 19 | `SESSION_NOT_ACCEPTING` | 检查 STOPPING/HOLDING/TERMINATED；完成 finalize 后 fresh open。 |
| 20 | `HORIZON_EXCEEDED` | 以最新 execution/replaceable time 缩短 suffix，保持最后一点不超过 max horizon。 |

任何 reject 都不会让之前 accepted trajectory 自动失效；Motion 应继续以它作为唯一 splice
基线。相同无效 payload 不得用新 sequence 无限重试。

## 5. 可调参数与当前值

Driver 参数在 `src/rt_control/rt_control_bringup/config/controllers.yaml`，包络在
`rolling_envelope_provisional.yaml`。修改 YAML 后需重新 configure/restart，活动 session
不热改。当前关键值：

| 参数 | 当前值 | 影响 |
| --- | ---: | --- |
| `buffer_capacity` | 64 points | 活动轨迹节点准入上限；不是批次目标大小。 |
| `required_initial_horizon_ms` | 500 | Prime 最小 future。 |
| `max_horizon_ms` | 600 | coherent execution time 到候选末点的最大 future。 |
| `update_timeout_ms` | 200 | 最后一次 accepted update 过期后的失联兜底。 |
| `replace_lead_ms` | 16 | 最早允许替换点领先 execution cursor 的时间；目标机待标定。 |
| `state_publish_period_ms` | 20 | rolling public state 约 50 Hz。 |
| `prime_timeout_ms` | 100 | open 后等首个 accepted batch 的时限。 |

Motion 自己持有 30 Hz batch、100 ms knot 和约 500 ms planned horizon；driver 只校验实际消息。

## 6. 最小示例与隔离 mock 验证

默认运行只输出 JSON，不初始化 ROS：

```bash
python3 tools/electri_102_motion_mock_producer.py
```

公开接口 DDS 夹具会启动隔离 peer 和 producer，不启动 controller manager、硬件或 enable：

```bash
source /opt/ros/humble/setup.zsh
source <robot_interfaces-feature-overlay>/setup.zsh
export ROS_DOMAIN_ID=184
export ROS_LOCALHOST_ONLY=1
python3 tools/electri_102_public_mock_harness.py \
  --run-seconds 5 --timeout-seconds 10
```

只有已确认处于隔离 mock Domain、且调用方已经 cancel/await 自己的 FJT goal 时，才可单独执行：

```bash
python3 tools/electri_102_motion_mock_producer.py \
  --allow-command-publication --run-seconds 5 --timeout-seconds 10
```

禁止把最后一条命令用于真实机器人。示例只发送 hold suffix，用于证明公共接口工作流；实际
Motion 必须替换 `build_hold_suffix()` 为上述 14 轴整形逻辑。

## 7. 当前不能据此宣称的内容

- 标准 GenericSystem mock 不模拟 CiA402 enable，公共 enable 会以 `status_word=0x0040` 拒绝；
  这不是 rolling 协议失败，也没有通过绕过安全检查修补。
- 目标机 DDS 端到端延迟、STRICT switch 分布、validation TID/PSR 尚未动态标定；
  `replace_lead_ms=16` 仍是 provisional。
- 当前工作区没有权威 joint5→hand-camera 外参，不能验证真实 IBVS/PBVS 坐标链；左手镜像
  估计和左右精度均未验收。
- provisional 包络未经台架停车实测；本指南不授权 reset、enable 或任何硬件运动。
- TF 发布频率、相机/工控机时钟同步和双臂同时伺服不在本期范围。
