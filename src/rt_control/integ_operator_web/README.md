# integ_operator_web

RT-Control 操作网页：一个进程、一个端口、多个页面（设计稿 `domains/rt_control/docs/operator-web-and-diagnostics-design.md` §4）。当前页面：底盘（原 `rt_control_operator_web`，ELECTRI-109/127）。手臂页、诊断页按设计稿后续迁入。只给台架与调试使用。

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
