# 机械臂末端键盘控制

当前组件版本 **v0.2.0**（2026-11-09），版本说明见 [v0.2.0](docs/releases/v0.2.0.md)。本工具以独立源码分支 `release/gripper-jog-v0.2.0` 保存在 `SevenovaHangzhou/robot_driver` 仓库；驱动侧接入保存在 `codex/jog-current-framework`，精确提交由 `integration/release.json` 记录。

真机入口为 [18121 关节操作台](http://192.168.100.40:18121/arm) 和 [18131 末端页](http://192.168.100.40:18131/jog)。两页共享登录，双臂控制权按连接互斥。18121 的动作入口自动选择 JTC，18131 自动选择 Rolling；停稳与控制器互斥反馈确认后才执行，硬件使能由操作员显式发起。

当前现场使用 `bench_dual_arm_head_wrist_force`、ROS Domain 77。Mink 在同一 Rolling 会话内持续更新未来轨迹，包含双臂互碰、自碰撞和段内动态限制检查。v0.2.0 冻结长按/松键/再次起步修复、非实时快照重读、98% 规划预算、连续制动尾段、时延分位统计和具体拒绝原因。初窗 120 ms、替换窗 100 ms，重力补偿关闭。

2026-11-09 部署及只读健康检查完成，用户随后反馈“效果不错”。该反馈记录与离线测试分开保存；完整生产验收、Docker 镜像发布及更短提前量验收尚未完成。默认本机启动入口仍为仿真。详情见 [部署记录](docs/DEPLOYMENT-20261109.md)、[真机键盘链](docs/REAL-KEYBOARD.md) 和 [真机碰撞检查](docs/REAL-COLLISION.md)。此前 `releases/v0.1.0-rc1` 保持冻结。

## 一键启动

```bash
# 网页显示完整机器人及实际末端轨迹，默认 18131。
/home/zlz/Desktop/ing_obj/tool/gripper_end_move_handle/script/start.sh

# 同时打开 MuJoCo 原生窗口，网页仍为 18131。
/home/zlz/Desktop/ing_obj/tool/gripper_end_move_handle/script/start_sim.sh

# 不开原生窗口，或使用其他端口。
script/start_sim.sh --headless --port 18133
```

浏览器访问终端打印的 HTTP 地址，例如 http://127.0.0.1:18131/jog；访问令牌为 `mock`。不要直接打开 `web/jog.html`。获取控制权，选择左臂或右臂，点击“开启键盘接管”。终端 Ctrl+C 或关闭原生窗口可结束本次实例。两个脚本默认使用同一端口，启动 `start_sim.sh` 前须先退出正在运行的 `start.sh`；网页和原生窗口只有在同一进程内才共享状态。显式使用其他端口时，浏览器也须打开该实例打印的地址。

默认加载 `../../sim/real2sim_v3` 的求解实现、场景与模型，可以使用 `--real2sim /absolute/path/real2sim_v3` 指定迁移后的目录。该目录必须同时保留 `real2sim/`、`config/`、`mujoco/`、`urdf/`、`meshes/`、`sources/legacy_joint_limits.yaml`；缺少依赖或模型时启动报错。

项目 `.venv` 已安装依赖。重新安装：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
```

依赖 Python 3.10+、NumPy、SciPy、MuJoCo 3.15.0、Mink 1.3.0、qpsolvers 4.13.0、DAQP 0.10.3、PyYAML。固定求解器版本与参考示例一致，包括碰撞约束的位移单位适配。

## 真机只读观测

```bash
# 自动查找工控机正在运行的 bench_dual_arm_head artifact，读取实际 URDF 和关节反馈。
/home/zlz/Desktop/ing_obj/tool/gripper_end_move_handle/script/start_real_observe.sh

# 仅收集和检查，不启动网页。
script/start_real_observe.sh --prepare-only
```

默认连接 `ar@192.168.100.40`、ROS Domain 78、本机网页 18131，令牌 `mock`。需要已保存的 `ipc-ssh` 技能连接配置、SSH/sshpass、项目 `.venv` 和本地 `sim/robot_description` mesh；本机 ROS 环境不是前置条件。脚本通过 SSH 在目标工作区运行订阅节点，只读取 `/joint_states`、内部状态广播器及 `ListControllers`。它不使能、不切换控制器、不打开 Rolling 会话，也不提供停止其他客户端会话的功能。

网页按真机 URDF 中的原始关节名显示双臂及头部，可直接切换观测左右臂；TCP 轨迹是实测关节角的 FK。未反馈的底盘、升降关节按模型零位显示，并在页面列出。关节反馈失效、运行进程退出或 artifact 模型身份变化时，保留最后有效姿态并标为失效。这里的 0.75 s 是显示新鲜度阈值，不是电机保护参数。当前 JointState 未提供有效速度时，实际速度显示“—”。

每次启动在 `.runtime/real-observe-时间/` 保存模型、配置、校验值和初始状态。`--workspace` 可在多个运行栈之间指定工作区；只能选择正在运行的工作区。更换模型资源使用 `--model-share`，更换观测端口使用 `--port`。默认 Domain 经本轮现场检查为 78，改变部署后须按实际值显式传入 `--domain-id`。此入口与仿真入口使用同一默认端口，启动前先退出已有本机实例。

默认关闭重力补偿（`--gravity-mode off`），不加载、构建或调用外部静态重力库，也不使用 MuJoCo bias 前馈。可选实现保留供显式离线对照：`--gravity-mode static` 调用外部 `sim/Dynamic_Parameter_Identification/static_gravity`（首次需要 CMake 和 C/C++ 编译器），`--gravity-mode mujoco` 使用原 MuJoCo bias。静态库的可选调用与模型校验见 [静态重力补偿](docs/GRAVITY.md)。

## 操作与反馈

- ↑/↓：机器人前/后；←/→：机器人左/右；+/−：机器人上/下。左右含义与左、右臂一致，两臂共用 `base_link` 本体方向，不受相机视角影响。
- 速度可选 0.5、1、2、5、10、20、30、50 mm/s，默认 0.5 mm/s，最高 50 mm/s；后端拒绝 100 档。长按仅发送一次启动，服务端持续续流；系统重复按键不产生额外动作。连续移动时修改速度选项，会在同一会话内替换未来轨迹。
- 松键旋转指令代次，并生成有界 jerk 的减速尾段；达到零速度、零加速度后请求控制器停止并 FINALIZE，完成前保持 busy。Esc、停止按钮、失焦、切后台、断线、保活超时和求解/协议故障直接请求控制器制动并退出接管。
- 切换机械臂前先按 Esc 并等待停止。两臂保留实际仿真姿态；非当前臂的关节命令固定为会话 hold。输入框或速度选项框拥有焦点时，方向键用于编辑；点击方向区空白处再控制机器人。
- 网页显示完整机器人，支持查看整机、当前臂、聚焦末端和分别清空两臂轨迹。模型关节、青色 TCP 轨迹、实际速度及本次按住位移均来自 MuJoCo 实际状态，不用目标点代替实际末端。

所选速度是 TCP 目标速度；实际速度受关节动态限制和物理跟随影响。客户端沿用原离线限制：关节速度不超过模型限制与 0.035 rad/s 的较小者，加速度 0.07 rad/s²，jerk 0.28 rad/s³；这些是仿真候选值，不是实测硬件参数，因此高档位可能无法达到。Mink 在已确认的未来关节状态上预测，保持末端朝向并施加参考场景的碰撞约束；发布前解析检查每个三次段的位置、速度、加速度、jerk 和同步制动空间，TCP 及几何仍为采样检查。实际跟随误差超过 3 mm 或仿真避碰保护触发时退出接管。

模型使用参考示例配套的 `urdf/robot.urdf` 和 `mujoco/scene_mink.xml`，包含标定后的双臂和整机外形。启动检查关节限位与 TCP 变换一致，网页网格只读加载同一份资源；没有混用旧模型。辅助关节固定零位，只有双臂 14 轴参与动力学。避碰距离仅代表该仿真几何，不代表实机安全距离。

## 数据链路

```text
键盘 / 网页方向按钮
  → HTTP velocity + 独立保活 / 控制权 / 指令代次
  → Mink / DAQP 预测未来关节状态（方向、速度、朝向保持）
  → C2 连续三次段 + 动态/制动空间检查
  → 同一 Rolling 会话内追加 / 替换未来尾段 → ACK
  → 本地 Rolling 协议模拟器的三次 Hermite 命令
  → MuJoCo 位置执行器（位置/速度反馈，2 ms；默认关闭重力补偿）
  → 实际 qpos/qvel → 网页整机模型、TCP 轨迹与速度反馈
```

正常控制不直接修改机器人 `qpos`；只有示例初始化会设置弯肘演示姿态。物理循环由单独线程持有，网页轮询不会推进仿真，也不会因浏览器重复按键积攒轨迹。

网页与状态 API 的 `gravity_compensation` 显示补偿关闭、输出前馈为零。仿真仍有物理重力，关闭补偿后位置反馈存在持重偏差。Rolling 仍只发布 q/v；本地补偿选择不修改实机驱动配置。运动算法回归显式使用原 MuJoCo bias 以隔离持重偏差，可选静态库测试显式开启 static。

实现见 `gripper_jog/mink_rolling.py`、`rolling_stream.py`；网页及 HTTP 服务无需前端构建。接口、连续性和验证边界见 [Mink → Rolling 说明](docs/MINK-ROLLING.md)。

## 验证与原 Rolling 实现

```bash
.venv/bin/python -m pip install pytest
.venv/bin/python -B -m pytest -p no:cacheprovider -q
node --test tests/test_input.mjs
```

新增测试使用真实 Mink、MuJoCo 和参考场景，覆盖双臂六方向、各档速度、非当前臂保持、松键、超时、旧代次拒绝和碰撞阻挡取消。模型路径不同时，设置 `JOG_TEST_REAL2SIM=/absolute/real2sim_v3`；无参考资产时这些测试会明确跳过。

显式执行 `script/start.sh --backend mink` 可使用原 Mink 直接仿真；`--backend mock` 是原 5 mm 分段 Rolling 演示，见 [历史 Rolling 说明](docs/ROLLING.md)。历史 ROS 联调代码 `gripper_jog/manual_sim.py` 仍包含 FJT 初始姿态准备，两个默认启动脚本均不进入它。`RollingStream(source, transport, velocity, clock)` 可以复用 `RosRollingTransport`，接入方负责 Motion 仲裁、反馈和 ROS 执行器；当前默认网页没有连接实际控制器。运行中的旧实例须正常退出并重新启动，改源码不会替换已启动进程。

控制响应统计、较短提前量的试验入口和具体超限诊断见 [Rolling 响应耗时与拒绝原因](docs/ROLLING-LATENCY-DIAGNOSTICS.md)。
