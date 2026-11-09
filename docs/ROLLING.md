# Rolling 发布（0.2.0.dev0）

本页记录保留的 5 mm 分段 Rolling 实现，使用 `script/start.sh --backend mock` 显式启动。当前默认入口已经改为 [Mink → Rolling 持续会话](MINK-ROLLING.md)，不会使用本页的逐段规划。

原 Mock 配置为 `publication_mode: rolling`。笛卡尔直线/姿态/限速规划产生路径，Rolling 客户端将它采样为位置和速度窗口，经本地协议模拟器执行；完整模型与 TCP 轨迹使用窗口的 cubic Hermite 插值状态。`publication_mode: step` 可回到旧 Mock 逐段执行方式。

## 接口

原实现参考旧 `robot_driver/src/rt_control/rolling_trajectory_controller`，历史文件身份保留在 `integration/rolling-reference.json`。新 main 的对应包为 `src/rt_control/ctrl_rolling_trajectory`，当前接入基线与生成类型验证见 [main 对齐记录](MAIN-ALIGNMENT.md) 和 `integration/main-reference.json`。公共接口仍以 `robot_interfaces` 生成包为权威，本项目不复制或修改 IDL。

| 端点 | 类型 | 语义 |
| --- | --- | --- |
| `/rt/rolling_joint_control/open` | `robot_rt_control_interfaces/srv/OpenRollingJointSession` | 协商协议、controller boot、V3 轴 hash、窗口、容量和 hold 锚点 |
| `/rt/rolling_joint_control/update` | `robot_motion_interfaces/msg/RollingJointTargetBatch` | 会话时间 ns、位置 rad、速度 rad/s；严格递增 sequence、replace_from_ns |
| `/rt/rolling_joint_control/state` | `robot_rt_control_interfaces/msg/RollingJointControlState` | 批次接受/拒绝序号、执行时间、可替换边界、期望位置/速度与会话状态 |
| `/rt/rolling_joint_control/close` | `robot_rt_control_interfaces/srv/CloseRollingJointSession` | `REQUEST_STOP` 请求受控制动；观测 `HOLDING` 后才 `FINALIZE` |

固定轴序为 `right_joint1..7,left_joint1..7`，轴身份 SHA-256 为 `f4c8ff8a32183d1733032494600c9f5d1e7e625673a4ad51556b3ac6c35adee4`。必须根据关节名映射，不能使用模型显示数组的 left/right 顺序直接发布。没有完整 14 轴 hold 状态时拒绝创建发布任务。未选中的一臂保持 hold 位置，速度为零。

页面速度可选 0.5、1、2、5、10、20、30、50 mm/s，默认 0.5 mm/s，最高 50 mm/s；数值用于 TCP 限速，实际速度仍受加减速和关节动态限制约束。页面内部按固定 5 mm 分段，每段规划使用一个会话，首批从会话时间 0 和 open 返回的精确 hold 锚点开始。后续窗口在前一批末端追加，等待 state 确认后才发送下一批，避免累积未确认更新。窗口、发布间隔和采样间隔由 open 协商值计算，不使用浏览器时间戳；每批至多使用一半缓冲容量，为保留的执行前缀和接缝留空间。构造窗口时另检查控制器实际使用的 cubic Hermite 插值的路径、姿态、速度和关节动态限制；该检查仍是离线采样检查。

本地模拟协商值来自控制器源码默认值：容量 64、最大窗口 600 ms、首批至少 500 ms、更新超时 200 ms、替换提前量 16 ms、标称周期 4 ms。它们仅用于模拟，不构成任何台套的已测限制或实机参数授权。当前客户端对协商参数有支持范围，无法满足容量/窗口预算时会关闭已开的会话并拒绝运动。

松键、Esc、输入保活失效和资源占用均请求 REQUEST_STOP。Rolling 按控制器关节制动语义停止，制动段不保证 TCP 仍沿原直线；不能将旧的路径时间制动结论用于 Rolling。停稳并完成 FINALIZE 前保持 busy，不能切臂或开始另一会话。拒包、确认超时、状态超时、boot/session 变化均退出键盘接管，不自动重试运动。

## 可选 ROS 适配

`RosRollingTransport(node, command_qos=..., state_qos=...)` 使用权威生成消息和服务，服务请求带 UUID，可重用相同请求等待未确认的结果；超时不等于服务未执行。应用必须使用与 driver pin 一致的公共接口构建环境和冻结 QoS，并在独立执行器线程 spin 节点。传输层不切控制模式、不使能、不重置、不自行创建硬件运动。`RollingMotion(plan, transport)` 是已有 Motion 运动仲裁可接入的客户端入口。

网页 HTTP 服务仍只允许 `mode: mock`。此前已经完成原生 ROS 2、权威生成类型、C++ Mock 控制器和 MuJoCo 的联合测试；此前“本机没有 ROS/编译器、未进行 ROS 联测”的记录已过期。历史 `gripper_jog/manual_sim.py` 拥有独立 Cia402MockSystem 控制栈，使用真实 ROS transport；它已从两个默认启动脚本中移除。

手动入口启动后完成 Mock 使能、配置初始姿态 FJT 和 ROLLING_READY 切换，再开放网页；源码入口为 `gripper_jog/manual_sim.py`。它核验生成的硬件插件及 14 轴身份，拒绝连接已有 DDS 控制器，使用正常公共服务和 Action。退出时停止/finalize、失能，再关闭自己启动的联合入口。网页展示控制器命令，MuJoCo 物理反馈仅用于观察与新鲜度检查，不作为 driver 的硬件反馈。实机接入仍须复用唯一执行仲裁，完成现场模型/TCP、碰撞检查、反馈、控制器激活与故障停车验证。

此历史模式长按仍按固定 5 mm 分段，每段末端停稳再开始下一段。新增 `RollingStream` 在同一会话持续追加或替换未来轨迹，复用本页的公开传输协议和 `RollingMotion` 生命周期；没有修改 controller 或公共接口。

## 本机运行

```bash
cd /home/zlz/Desktop/ing_obj/tool/gripper_end_move_handle
source .venv/bin/activate
python -m gripper_jog.server --backend mock --port 18131
```

访问 `http://127.0.0.1:18131/jog`，令牌 `mock`。以源码模块启动可确保运行当前开发版，而不是此前安装的 rc1 wheel。

历史 ROS 手动联调须自行准备 ROS/driver overlay 后运行 `python -m gripper_jog.manual_sim`；它仍包含 FJT 初始姿态准备。当前 `script/start_sim.sh` 已改为本地 Mink + MuJoCo，不再启动这一历史链路。历史日志在项目 `.runtime/manual-sim/`。

冻结包 `releases/v0.1.0-rc1` 保留原样；当前工作源码及本机 Mock 构建已包含联合测试修复，未部署远端或提交代码。
