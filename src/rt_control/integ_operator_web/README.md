# integ_operator_web

RT-Control 操作网页：一个进程、一个端口、多个页面（设计稿 `domains/rt_control/docs/operator-web-and-diagnostics-design.md` §4）。页面：`/` 底盘（原 `rt_control_operator_web`，ELECTRI-109/127），`/arm` 手臂（原 `rt_control_arm_console`，ELECTRI-127），`/diag` 诊断（只读）。两页共用登录会话、审计日志与操作权规则：任一页面的控制权被另一个登录会话持有时，本会话不能取得其他页面的控制权。只给台架与调试使用。

## 功能

- 使能、失能底盘；故障（FAILED）时复位。
- 摇杆或 WASD/QE 键盘遥控，休闲档与运动档。指令发到能力声明中的速度话题（`chassis_velocity`）。
- “停止移动”：软件停止，不是硬件急停。
- 转向运动参数：速度、加速度、加加速度（S 曲线）、对齐门限。修改需要令牌确认，只能在未使能时修改，下次使能生效。

## 边界

- 只用公共接口：`/rt/capabilities`、`/control/set_enabled`、`/rt/reset_fault`（仅 FAILED）、
  `/controller_manager/list_hardware_components`、控制器参数服务、`/diagnostics`、能力声明中的速度话题。
- 运行栈必须发布 `/rt/capabilities`，并声明 `enable` 与 `chassis_velocity`；实际硬件组件必须与声明一致。
- 不启动硬件，不写驱动控制字，不自动使能，不自动复位。
- 所有检查在服务端执行；浏览器只发请求。
- 登录：网页用令牌调用 `POST /api/login`，换得 HttpOnly 会话 cookie（`SameSite=Strict`）；WebSocket 握手时校验 cookie 与来源，消息中不再传令牌。会话保存在进程内存，服务重启后需要重新登录。修改转向参数、恢复默认时仍需再次输入令牌确认。
- 操作权：同一时间只有一个连接持有控制权，其他连接只读；持有者断开连接或心跳超时（50 ms 心跳、200 ms 超时）时底盘速度归零，不失能、不复位。
- 审计：登录与所有命令（遥控速度流除外）追加到 `~/.ros/log/integ_operator_web/audit.jsonl`（`--audit-log` 改路径，空字符串关闭）；不写令牌和会话值。

## 转向参数的现场调参

- 默认值来自部署产物（`<generated_dir>/controllers.yaml`）。
- 网页修改写入 `/var/lib/rt-control/tuning/<machine_unit>/swerve_controller.yaml`，并在同目录
  `history.jsonl` 追加记录。`machine_unit` 取自 `<generated_dir>/artifact.yaml`。
- 底盘运行栈启动时由 `integ_bringup` 的 `scope_operator_tuning` 应用该文件；控制器按上限校验。
- “恢复默认”删除现场调参文件，并把参数设回默认值。
- 未给出 `--generated-dir` 时，转向参数只读。

## 启动

1. 加载工作区环境，设置与运行栈相同的 `ROS_DOMAIN_ID`（1–232），并使用 `ROS_LOCALHOST_ONLY=1`
   或只含 127.0.0.1 的 UDP DDS 配置。
   - 如果运行栈本身对局域网可见（例如导航在其他进程用默认 DDS 配置接入），只用回环的网页无法与运行栈通信。
     此时设置 `RT_CONTROL_OPERATOR_WEB_ALLOW_LAN_DDS=1`，并使用与运行栈相同的 DDS 配置。
   - 打开该开关后，同一 Domain 的局域网主机都能看到底盘话题，也能发送 `/cmd_vel`。
     Domain 必须为非默认值，同一时间只允许一个 `/cmd_vel` 发送方。
2. 令牌文件权限 0600，路径写入环境变量 `RT_OPERATOR_WEB_TOKEN_FILE`。
3. 启动：

   ```bash
   ros2 run integ_operator_web integ_operator_web \
     --bind-host <局域网地址> --port 18120 --cpus <非实时 CPU，如 0-6,8-19> \
     --generated-dir <底盘部署产物目录>
   ```

4. 浏览器打开 `http://<地址>:18120/`，输入令牌登录。

## 依赖

`python3-aiohttp`（Ubuntu 22.04 软件源）、`python3-yaml`、ROS 2 Humble 与工作区接口包。

# 手臂页

手臂页通过 `/ws/arm` WebSocket 接收状态并发送命令（每条命令一个请求号，服务端逐条回复）；持有控制权的连接断开时，手臂沿当前路径平滑停车并释放控制权。

单臂关节网页操作台（ELECTRI-127）。用滑块设定 7 个关节目标，确认后以 S 型曲线发送。
页面显示整车 Robot Model：实体为实测位姿（未受控关节在零位），半透明只显示受控手臂的目标预览。

## 边界

- 机械臂控制只调用公共接口：
  - `/rt/capabilities` 中的 `arm_joint_trajectory`（FollowJointTrajectory action）；
  - `/control/set_enabled`；
  - `/controller_manager/list_controllers` 与参数读取；
  - `/joint_states`、`/diagnostics`。
- 操作台不写驱动控制字，不切换控制器，不自动使能，不复位故障。
- 所有检查在服务端执行。浏览器只提交目标。
- “停止”是软件平滑停车，不是急停。

## 临时真空开关（2026-10-24）

页面控制区下方提供真空泵、左臂阀、右臂阀三个独立开关，显示线圈读回状态。
点击已关闭的按钮开启对应输出，点击已开启的按钮关闭；三个开关不自动联动。
开启状态不代表已建立真空或吸附成功；压力读数独立显示，不参与负载判断或开关联动。

后端复用现场用户安装的 `~/.local/bin/vacuum`，可通过 `--vacuum-script` 指定可信脚本。
脚本不存在时显示“未配置”，不访问设备；页面启动及轮询仅读取状态，不写输出。
此入口为本次现场临时 IO 操作，不替代主分支的公共真空接口及完整真空业务联锁。

`GET /api/arm/vacuum` 需要登录，最多每 0.8 秒执行一次读取；3 秒未更新的值不可操作。
写入为 WebSocket 消息 `{type: "vacuum", output: "pump"|"left"|"right", enabled: bool, expected: bool}`。
写入需要原有登录、同源和控制权检查，运动、序列或使能切换期间拒绝；实际线圈值已与
浏览器显示值不同则拒绝本次操作。写入后核对回显及读回；超时或未知结果不自动重试。
写请求使用既有审计日志。脚本的文件锁协调同一用户的 CLI 与 Web 请求。

本次现场映射由 `vacuum` 脚本提供：右阀 00、左阀 01、真空泵 02；绑定 `enp4s0`，
目标 `192.168.1.12:502`，站号 1。网页不接受用户提供脚本路径或 shell 命令。
更换后端需要在无运动、无序列时重启 Web 操作台，刷新后重新登录、获取控制权。

## 真空压力显示（2026-10-27）

右上角显示左、右真空表压，单位 kPa，保留一位小数。读取
`/plc_io/vacuum/left`、`/plc_io/vacuum/right` 的
`periph_plc_io/msg/VacuumSensorState.pressure_kpa`，不在浏览器重复换算或自动归零。
现场已确认 AI0=左、AI1=右，ZSE30A 的大气压为 0 kPa，真空为负值。
传感器有效标志为假、单位错误、非有限值或超过 1 秒未收到消息时清除数字并显示原因；
网页连接断开或停滞 1 秒也清除数字。两路独立判定，未收到压力不阻止机械臂操作。
压力由独立 IO 节点发布；`arm start` 的操作台订阅不会启动或改写模拟量模块。

## 运动规则

- 轨迹为 7 段 S 型曲线（加加速度分段恒定），起止速度和加速度为 0。
- 所有关节沿关节空间直线运动，同时出发、同时到达。移动量最大的关节使用档位上限。
- 档位（`config/console.yaml`）：提供 2～15°/s 的每个整数速度，保留 30°/s 运动档；多档使用速度下拉框。
  新档沿用加速度 = 2 × 速度、加加速度 = 8 × 速度的既有比例。示例：

  | 档位 | 速度 | 加速度 | 加加速度 |
  | --- | --- | --- | --- |
  | 休闲 | 10°/s | 20°/s² | 80°/s³ |
  | 运动 | 30°/s | 60°/s² | 240°/s³ |

- 运动关节的 Robot Model 速度上限低于档位时，使用模型上限。
- 目标必须在 Robot Model 位置限位内侧 2° 以内。
- 发送前检查：持有控制权、已使能、轨迹控制器已激活、关节反馈不超过 0.2 s、无运动在执行。
- 轨迹第一点等于发送时的实测位置，用于通过控制器的起点一致性检查（1°）。
- 停止：从运行状态沿同一直线减速到静止，不越过原目标。控制器拒绝停止轨迹时，取消目标，
  由控制器原地保持。

## 位姿

- “保存当前位姿”保存实测位置，名称 1–32 个字符。默认文件：
  `~/.local/state/rt-control/arm_console_poses.json`。
- 只列出关节集合与当前受控关节一致的位姿。
- “到达”不弹确认框，直接执行；服务端检查与“发送到目标”相同。
- 位姿序列：在位姿上点“+ 序列”按顺序加入（可重复），“依次执行”由服务端逐个执行。
  开始前检查全部位姿存在且在限位内；每一步都重新检查控制权、使能、控制器和反馈。
  “停止”停下当前运动并取消剩余步骤；某一步失败、控制权丢失或失能都会中止序列；
  与当前位置相同的位姿无需发送运动，但仍按该段设置停留。

## 启动

与底盘页同一进程、同一端口（见上文「启动」），浏览器打开 `http://<主机地址>:18120/arm`。运行栈需使用发布 `arm_joint_trajectory` 能力的作用域，例如 `bench_left_arm`。手臂页启动参数：`--arm-config`、`--poses`、`--joint-state-topic`、`--vacuum-script`、`--view-model-share`。

## 故障复位

使能状态为"故障"（FAILED）时显示"故障复位"按钮：需要先取得控制权，确认后调用 `/rt/reset_fault`
（使能管理器整组 CiA402 复位，只把故障轴清回"禁止合闸"，不使能、不运动）。是否接受由使能管理器决定：
`restart_required` 等状态会被拒绝并给出处理建议（例如重启控制栈）。复位前应先排除故障原因。

## 操作审计日志

每个操作请求（登录、取得/释放控制权、预览、执行、停止、去位姿、序列、保存/删除位姿、使能/失能）
追加一行 JSON 到与底盘页共用的 `~/.ros/log/integ_operator_web/audit.jsonl`。
内容：时间、`ROS_DOMAIN_ID`、操作、参数（目标、档位、位姿名、使能）、结果与拒绝原因、会话摘要（8 位哈希）、客户端地址。
不写令牌和会话值；写失败不影响操作。Runtime-LOG 按 ROS 域和时间把它们归入运行时间线。

## 浏览器库

`web/vendor/` 内为离线使用的 three.js 与 urdf-loader，来源和许可证见 `web/vendor/README.md`。

## 位姿序列到位停留（R2S-DWELL-20261021）

位姿序列每段可单独选择速度和到位后停留秒数（非负有限数字，支持小数；0 表示不额外停留）。
新增段继承当前速度和默认停留；“应用到全部段”可批量修改，载入预设后也可逐段调整。
`run_sequence` 消息兼容 `names/gear/dwell_s` 统一设置，`dwell_s` 默认 0，
保持旧客户端行为；字符串、布尔、负数和非有限值拒绝。

停留从该段 JTC 成功完成回调开始；已经到达而无需运动的位置也停留，
最后一个位置停留结束才显示序列完成。以服务端单调时钟计时，系统调度可能延后，
不会承诺精确到毫秒，也不把 Action 成功作为额外的静止物理测量。

停留由非实时工作线程实现，等待期间不持有控制锁。等待中点击停止会取消剩余序列；
控制权丢失、使能/控制器不可用或反馈超时会中止，不自动恢复。每次发送前仍执行现有
准入检查，旧任务不能推进新序列。运行或停留时拒绝单独运动、保存位姿和使能切换。
状态新增 `sequence.dwell_s/dwelling/dwell_remaining_s`，用于网页倒计时。

此选项不修改目标位姿、速度档位、JTC参数、PID或硬件控制环，也不改变历史采集时长。
它不能替代 T0 的手动8秒保持记录。启用新后端需要重启操作台进程，须在无运动、
无序列时按现场授权进行；重新登录并获取控制权后才可由操作员发起下一次序列。


## 指定网页显示模型（R2S-VIEW-MODEL-20261021）

`--view-model-share <robot_description目录>` 选择浏览器整车几何来源，入口为
`urdf/robot_alfav3_mobile.urdf`，网格从同一发行目录读取。未显式指定时，若存在
`~/.local/share/rt-control/view_robot_description`（可为软链接），自动采用该目录；
否则保持原有运行时模型行为。参数不改变 ROS 控制器模型、运动限位或速度准入。

本地当前链接指向用户指定的 `/home/kkozia/桌面/ing_obj/sim/robot_description`。
远端采用同一仓库提交的完整发行目录，不能把 URDF 或网格单独复制到 `web/` 私改。
切换模型目录后需重启网页服务；先检查无运动、无序列，并保留机器人控制栈和 guard。

`/api/arm/info` 的 `view_model` 返回入口、URDF SHA256 和资源版本。资源版本由 URDF 和
`config/mesh_assets.json` 共同计算，网格 URL 使用 `/view-mesh/<版本>/...stl`，
避免浏览器沿用旧缓存。更换网格时应同步更新权威资源清单。
选定模型不兼容受控关节时返回 `view_model_unavailable`，不静默切回旧模型。
控制栈未提供关节数据时也可显示模型，页面提示“模型已加载，等待关节数据”；
模型加载成功不表示设备已经就绪或已使能。

版本、验证边界及回退资料见
[变更记录](../../../domains/rt_control/docs/areas/control/records/2026-10-21-console-latest-view-model.md)。


## 逐段参数（ARM-SEQUENCE-20261024）

`run_sequence` 消息可改用 `steps`（1～50 段，可重复位姿）：

```json
{"steps":[{"name":"A","gear":"speed_3","dwell_s":0.5},{"name":"B","gear":"speed_15","dwell_s":3}]}
```

`gear` 与 `/api/arm/info` 一致；已有 `test_low2/test_low5/leisure/sport` 保留。
速度用于前往本段位姿，停留从本段成功到位后开始；最后一段停留结束才算完成。
执行前验证所有段的位姿、限位、速度和停留；任一段无效整组拒绝。
状态 `sequence.steps` 提供本次参数快照，`sequence.dwell_s` 为当前段停留。
预设文件格式保持兼容，载入时将统一参数展开成独立段，载入不会执行。

# 诊断页

`/diag` 只读，不取控制权、不下发命令，经 `/ws/diag` 每秒推送一次：

- 启动自检报告：`$RT_CONTROL_STATE_DIR/self_check/<作用域>.json`（默认 `/tmp/rt-control/`），每个作用域最新一份，新的在前。阶段 A 由运行栈启动文件在总线启动前写入；运行栈因自检失败没有起来时，这里仍能看到原因。阶段 B 由诊断节点在硬件激活后追加。
- 运行栈就绪状态：R-OUT-09 `/rt_control/readiness`（每个问题的位置、DREE 码、级别、厂家码与说明）。诊断节点未运行时显示"未运行或尚未发布"。
- 原始诊断：`/diagnostics` 每个名称的最新一行及其键值。

本页不替代急停、STO 或安全链。

