# rt_control_operator_web

底盘网页操作台（ELECTRI-109 首版，ELECTRI-127 改为按能力声明对接新架构）。

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
- 所有检查在服务端执行；浏览器只发请求。单终端控制权。

## 转向参数的现场调参

- 默认值来自部署产物（`<generated_dir>/controllers.yaml`）。
- 网页修改写入 `/var/lib/rt-control/tuning/<machine_unit>/swerve_controller.yaml`，并在同目录
  `history.jsonl` 追加记录。`machine_unit` 取自 `<generated_dir>/artifact.yaml`。
- 底盘运行栈启动时由 `rt_control_bringup` 的 `scope_operator_tuning` 应用该文件；控制器按上限校验。
- “恢复默认”删除现场调参文件，并把参数设回默认值。
- 未给出 `--generated-dir` 时，转向参数只读。

## 启动

1. 加载工作区环境，设置与运行栈相同的 `ROS_DOMAIN_ID`（1–232），并使用 `ROS_LOCALHOST_ONLY=1`
   或只含 127.0.0.1 的 UDP DDS 配置。
2. 令牌文件权限 0600，路径写入环境变量 `RT_OPERATOR_WEB_TOKEN_FILE`。
3. 启动：

   ```bash
   ros2 run rt_control_operator_web rt_control_operator_web \
     --bind-host <局域网地址> --port 18110 --cpus <非实时 CPU，如 0-6,8-19> \
     --generated-dir <底盘部署产物目录>
   ```

4. 浏览器打开 `http://<地址>:<端口>/`，输入令牌。

## 依赖

`python3-aiohttp`（Ubuntu 22.04 软件源）、`python3-yaml`、ROS 2 Humble 与工作区接口包。
