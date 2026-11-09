# 真机末端验证与历史准备记录

## 2026-11-07 当前进展

用户已确认 Rolling 测试完成。当前适配使用 `robot_driver cbf4116` 与公共接口 pin `aa82a77`，未来窗口接口为 `/rt/arms/rolling/target_batch` 的 `epoch/sequence/replace_from/points/final`，不使用旧 open/close 协议。

远端 18121 和 18131 已部署到同一控制链：`/home/ar/rt-control-arch-20261106`，作用域 `bench_dual_arm_head`，当前实际 artifact 为 `bench_dual_arm_head_wrist_force`、Domain 77。已按用户确认跟随新控制进程重启两个网页；18121 开启末端发布，18131 运行 Mink 键盘入口。共享双臂控制权的双向互斥、交接、断线归还及浏览器跳转已验证。本次没有使能、模式切换或运动。

客户端与官方 RollingSender 适配已实现，见 [真机键盘链](REAL-KEYBOARD.md)。`script/prepare_real_experiment.sh` 接入双臂互碰、每臂自碰撞和三次插值区间复核，见 [真机碰撞检查](REAL-COLLISION.md)。已重新采集新 artifact 的实测反馈，左右臂六方向在 0.5 mm/s 下生成 21 点窗口并通过离线复核，首窗口耗时约 20–23 ms；原始 URDF SHA-256 为 `51584c4c5309c12139a92e22ca11ba2e3abf1ff4ab6e9847d2a1c33e30cb9c29`。碰撞候选余量、TCP 标定、现场方向与实机短行程验证仍待完成。

以下 2026-11-03 的控制器、接口、进程和包络状态均为历史记录，不能作为当前运行状态。

## 2026-11-03 历史准备

2026-11-03，用户指定首轮左臂，确认另一聊天仍在进行真机 Rolling 测试，要求本聊天同步准备。交接聊天：`codex://threads/01a190bb-de84-7081-bdca-efb563950963`。当前环境没有可读取其他聊天的工具，本记录依据实际只读检查，不代表已取得该聊天的验收结论。

main 更新后的代码基线为 `69d7273`，公共接口为正式 pin `c82f0e4`；生成类型兼容性及客户端相关回归 181 项通过。2026-11-03 22:16（北京时间）只读复查仍为 03e / Domain 78，双臂 JTC 与头部控制器当时 active，未发现 Rolling 服务；证据在 `.runtime/real-observe-20261103-221647/`。主分支更新不能替代现场部署和验收，详见 [main 对齐记录](MAIN-ALIGNMENT.md)。以下较早快照保留作历史。

补充：统一控制权已基于 `689ef34` 完成本地接入并通过双网页验证，补丁与启动说明见 [统一控制权](CONTROL-AUTHORITY.md)。本轮没有修改远端服务；以下现场快照仍为历史，未将本机构建结果视为已部署或验收。

## 已准备的可运行入口

```bash
/home/zlz/Desktop/ing_obj/tool/gripper_end_move_handle/script/start_real_observe.sh
```

本机 18131 提供真机只读整机模型、双臂切换和反馈 FK 轨迹；本机令牌 `mock`。脚本自动查找目标机正在运行的 `bench_dual_arm_head` artifact，校验 `robot_description.xml`、`controllers.yaml` 与 `artifact.yaml` 的 SHA-256，拒绝 mock artifact。按名字绑定两臂各 7 轴反馈，没有符号修正、偏移或演示初始姿态。头部按实际反馈显示；未反馈的底盘及升降轴明确标注为模型零位。

远端仅启动用户态只读 ROS 节点，订阅 `/joint_states`、`/rt_internal_state_broadcaster/dynamic_joint_states`，调用 `controller_manager/ListControllers`，读取服务图。没有启停实时栈、总线、使能或控制器，没有打开或关闭 Rolling 会话，没有发布轨迹或电机目标。网页与后端均禁止获取运动控制权、接管、点动和续流；关闭观测窗口也不会停止另一客户端的动作。

反馈超时、Domain 不一致、关节缺失或非有限数值、模型校验不一致、绑定的实时进程退出时，不呈现为正常实时反馈。保留最后有效姿态并标明失效，运动始终禁止。显示新鲜度阈值 0.75 s 仅供观测，不构成硬件安全参数。

## 本轮只读快照（后续仍须刷新）

初次采样为下表的 03d。重启本机只读服务时自动发现另一聊天已更新至 `/home/ar/rt-control-arch-20261103e`，新报告在 `.runtime/real-observe-20261103-212404/`，模型 hash 为 `15ec7701014dbf7e47f052b1e53e6deed1357522dd800a0580e540d29522dcf7`。该次 JTC/head 查询为 inactive，反馈仍可订阅；不对另一聊天的操作或验收结论作推断。

补充统一控制权状态后，本机最终运行入口的报告与页面截图位于 `.runtime/real-observe-20261103-213923/`，仍绑定 03e。18131 只读服务已启动，状态为 `control_authority.required=true`、`verified=false`。

| 项 | 核对结果 |
| --- | --- |
| 工控机 | `ar@192.168.100.40` |
| 当前实时工作区 | `/home/ar/rt-control-arch-20261103d` |
| scope / Domain / 操作台 | `bench_dual_arm_head` / 78 / 18120 |
| 当前双臂控制器 | `dual_arm_jtc` active；只读图查询未发现 Rolling 服务 |
| 已安装 Rolling 插件 | `ctrl_rolling_trajectory/RollingTrajectoryController` |
| 当前包络来源 | 只找到标注 mock-only、ESTIMATED_NOT_MEASURED 的包络；不能作为真机参数 |
| 18121 | 旧 `rt-control-electri127`，Domain 77，不能用作当前双臂栈状态 |
| 真机模型 | `8f9dd4353b4d2d8fd8b40c2ee421c5a656002d330cd216a3793c1f834efcb46f` |
| 网格资源 | 41 个可视 mesh 均可从本地 `sim/robot_description` 解析 |
| 首轮机械臂 | 左臂；具体现场方向尚未确定 |

快照时 EtherCAT link_up=1、18 个从站响应、wc_error_count=0，双臂误码为零。这些只说明当时可观测状态，不是 Rolling 或运动验收。另一聊天后续改变部署时，这份快照可能失效，入口会重新读取当前运行栈。

本地本轮证据：`.runtime/real-observe-20261103-prepared/`。`report.json` 记录来源、SHA-256、关节绑定与方向映射；`initial-state.json` 保留采样值；`model-comparison.json` 保留模型比较；`observer.png` 为页面验证截图。此前 `/home/ar/rt-control-arch-20261103c` 的检查历史保留在 `.runtime/real-rolling-preflight-20261103/`，不再当作当前工作区。

远端已安装的 `RollingJointTargetBatch`、`RollingJointControlState`、`OpenRollingJointSession`、`CloseRollingJointSession` 生成类型均可导入；字段和 IDL 校验值保存在 `generated-interfaces.json`。这不等于接口版本兼容、服务运行或运动验收通过，仍需以最终交接为准。

## 模型差异与 Mink 接入边界

真机 URDF 与 `real2sim_v3/urdf/robot.urdf` 的双臂关节名一致，但模型 hash、安装变换、轴向、TCP 固定变换和限位不同。左 J3/J6、右 J3/J4 存在轴向差异。用本轮实测 q 在两个模型的 `arm_carriage` 中做 FK，末端位置差约为左 3369 mm、右 3378 mm；这是模型定义不一致，不能解释为实际跟随误差。

真机本体方向 `base_link` 到 `arm_carriage` 的映射为 X 保持、Y/Z 反向，已直接从 URDF 导出；不能沿用仿真中的单位映射。方向键尚未发布，现场方向和 TCP 标定仍待核验。当前 q 接近零位、双臂近伸直，雅可比最小奇异值很小；后续必须评估可行起始姿态，不能把仿真弯肘初值写入真机。

现有 `MinkRollingJog` 仍使用仿真模型与 MuJoCo 反馈。不能仅替换为 `RosRollingTransport` 就作为真机客户端。下一阶段需用验收后的权威 URDF 建立配套 Mink 模型与碰撞几何，以完整实测 q 初始化，冻结非目标臂及辅助轴，并消费实际 Rolling 命令锚点。先做只求解、不发布的方向、TCP、连续性和动态包络核验，再接运动。

## Rolling 验收后所需交接

1. 最终运行工作区、Domain、控制器名、公开服务/话题和已导入的公共接口版本或完整提交 SHA。
2. 权威 artifact/URDF 的路径与 hash；Rolling 14 轴的实际顺序、单位、零位和方向语义。
3. 实机认可的 14 轴速度、加速度、制动加速度和位置余量包络的路径与依据；客户端 jerk 等额外限制的来源。不得将 mock 参数改名当作实机参数。
4. 实际状态中的 mode、boot/session/client 身份、desired q/v、ACK 及 buffer/revision/lead 限制；确认公共接口与本工具当前固定版本是否兼容。
5. 已验证的使能、模式仲裁、松键/主动停止、通信中断、会话归还和故障退出行为，以及验收记录路径。
6. 18121 与 18131 共用双臂运动资源的统一网关及接入方案。当前两个控制栈的租约没有共享；本项目已完成新主操作台与代理的本地接入，但远端未部署。此项完成前 18131 保持只读，详见 [统一控制权](CONTROL-AUTHORITY.md)。

拿到交接后按顺序进行：刷新只读检查 → 真机配套 Mink 模型离线求解 → 完全不发布的影子运行 → 确认现场方向与监护条件 → 左臂 0.5 mm/s 有界短行程 → 停止/松键/断线验证 → 逐步扩展方向和时长。0.5 mm/s 是首轮拟用 TCP 档位，不代替实机动态限制审批。右臂先保持，不自动进行双臂运动。

重力补偿按用户要求继续默认关闭：不加载静态库、不叠加 MuJoCo bias，也不修改真机驱动参数。此前默认关闭路径 2 项测试、相关重力/Viewer/Rolling 49 项回归及显式 MuJoCo bias 对照运动回归 58 项均通过；本轮不重复改动该链路。

## 本轮验证

- 只读反馈绑定、过期/异常反馈、全部运动动作拒绝、HTTP、模型资产、机械臂切换和方向回归：48 项通过，18 项依赖显式外部模型的测试按原规则跳过。
- 浏览器输入状态机：10 项通过；新脚本及前端语法检查通过。
- 实际 SSH 只读观测启动成功。浏览器加载整机 mesh、切换左右臂、TCP 与显示模型对齐、无资源加载错误均通过。所测 TCP 对齐误差显示 0.000000 mm，这只验证软件 FK 与渲染一致。
- 浏览器长按方向键没有发出运动请求；直接调用控制 API 被拒绝；反馈失效页面显示最后有效姿态。
- 新增统一双臂仲裁 7 项测试通过；与只读后端和 HTTP 联合回归共 27 项通过。该结果不代表远端 18121 已接入共享仲裁。

以上没有完成真机 Cartesian 运动验证，也没有提交、推送或部署到实时栈。
