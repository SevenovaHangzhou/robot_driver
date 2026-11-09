# Mink → Rolling 持续会话

默认启动方式为 `script/start.sh`，原生窗口使用 `script/start_sim.sh`；默认网页端口 18131，令牌 `mock`。`--backend mink-rolling` 明确选择本模式；`--backend mink` 和 `--backend mock` 分别保留原直接仿真、原分段协议演示。

## 会话与未来窗口

`MinkRollingSource` 在独立 Mink Configuration 上预测未来状态，不用未来配置覆盖实际机器人 qpos。`RollingStream` 负责协议时间、ACK、追加及替换，复用 `RollingMotion` 的 boot/session/client 身份、幂等 open/close 恢复及 HOLDING 后 FINALIZE。

1. 读取控制器状态并 open，固定轴顺序为 `right_joint1..7,left_joint1..7`。精确复制 open 的 14 轴 hold，首点速度为零。
2. 先发送静止首批，覆盖协商的初始窗口；无需在 100 ms prime 预算内完成整个运动预测。
3. 等待 `last_accepted_sequence` 确认。正常持续输入从已确认末点追加；改速、换向或正常松键，从已确认的未来 knot 替换尾段。接缝之前的轨迹保留。
4. 任何时候至多一个未确认批次。候选窗口仅在 ACK 后提交为轨迹账本；拒包、迟到、状态超时或身份改变均请求停止，不修包续动。

批次时间只使用控制器 session 的整数纳秒，执行时刻不要求恰好落在采样网格上。窗口、采样与更新间隔来自 open 协商；本地模拟默认最大 600 ms、首批至少 500 ms、更新超时 200 ms、lead 16 ms、标称周期 4 ms、容量 64，对应未来窗口约 540 ms、轨迹点间隔 20 ms、约 67 ms 更新一次。它们是本地默认值，不能代替现场协商。每批至多使用半容量，并检查保留前缀、接缝和新点的合并预算；运行时间增长不会增加账本容量。

替换边界预留 lead、生成预算和调度余量；生成结束后重新读取身份及可替换边界，过期则停车。状态传输陈旧或 RT admission/commit 之间继续推进仍可能导致拒包，客户端以实际接受/拒绝状态为准。此 Python 工作线程没有硬实时保证。

## 接缝与动态限制

Mink 建立位姿任务、冗余姿态正则、配置限位与参考模型的避碰 QP。以三次段末端加速度作为求解变量，对速度、加速度及 jerk 添加约束，并保留减速余量。另一臂固定在会话 hold，速度与加速度为零。速度目标包含短时预测及加速度阻尼，避免 jerk 饱和后反复超调。

对每个间隔 `h`，给定接缝 `q0,v0,a0` 与求得的 `a1`：

```text
j  = (a1-a0)/h
q1 = q0 + v0*h + (a0/3+a1/6)*h²
v1 = v0 + (a0+a1)*h/2
```

控制器由两端 q/v 重建的三次 Hermite 与此常 jerk 段一致。追加及替换均复制旧段的 q/v/a，所以保持 C2；jerk 可在 knot 改变，但绝对值有界。发布前解析求位置/速度极值、检查两端加速度、段内 jerk、C2 接缝和同步制动空间，避免只查 knot 漏掉段内超限。TCP 速度及几何检查仍为采样检查，不是完整扫掠体证明。

客户端沿用既有离线 `Limits` 的保守关节限制：`min(model,0.035) rad/s`、`0.07 rad/s²`、`0.28 rad/s³`；TCP 命令上限 50 mm/s。高档速度不保证达到；现场参数与方向性制动包络应由接入方提供经核验的约束，controller 仍会独立验证。默认值不构成硬件调参授权。

## 正常松键与异常停止

`/halt` 保留控制权、旋转 epoch、禁止新动作，客户端从未来接缝规划有限时长的减速尾段。尾段继续按窗口补充，达到 `v=0,a=0` 后才请求 REQUEST_STOP，观测 HOLDING 并 FINALIZE；停止完成前不能切臂或重新启动。

Esc、失焦、400 ms 保活超时、反馈失效、求解/几何检查失败、拒包与身份变化直接请求控制器受控制动。控制器故障制动从当前 desired q/v 出发，通常不保持 C2 或 jerk 上限；不得将正常规划尾段的连续性承诺用于故障制动。未知 open/stop/finalize 结果保留原请求及清理责任，不重新发起运动。

## 仿真与 ROS 接入边界

默认链路为本地 `MockRollingTransport`，然后驱动同一进程的 MuJoCo 执行器；网页整机姿态、TCP 轨迹和速度来自实际 qpos/qvel。实际末端与当前命令的误差超过原 3 mm 阈值、或实际几何触发仿真保护时退出接管。正常运动不直接写实际 qpos。

已有 Motion owner 可构造 `MinkRollingSource(follower, chain, side, desired_positions, limits=...)`，再创建 `RollingStream(source, ros_transport, velocity, clock)`；owner 持续调用 `state()`、`set_velocity()`、`request_halt()` 或 `brake()`。`RosRollingTransport` 继续使用权威生成类型，不切模式、不使能。ROS executor 必须独立 spin，公共 QoS 与接口 pin 必须匹配。

本次没有修改 RT-Control 或公共 IDL，没有接通真实电机。协议模拟器不能证明 C++ 控制器的 RT 消费时序、动态低水位或真实方向性 envelope。实机仍需既有 Motion 执行仲裁及现场验证；旧 `manual_sim.py` 没有自动切换成新适配器。

测试结果与浏览器验证记录见 [本地验证](MINK-ROLLING-VALIDATION.md)，controller 工作树指纹见 `integration/mink-rolling-reference.json`。
