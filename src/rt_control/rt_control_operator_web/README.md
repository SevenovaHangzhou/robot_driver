# rt_control_operator_web

底盘网页操作台（ELECTRI-109 首版，ELECTRI-127 改为按能力声明对接新架构）。

## 功能

- 使能、失能底盘；故障（FAILED）时复位。
- 摇杆或 WASD/QE 键盘遥控，休闲档与运动档。指令发到能力声明中的速度话题（`chassis_velocity`）。
- “停止移动”：软件停止，不是硬件急停。
- 转向运动参数：速度、加速度、加加速度（S 曲线）、对齐门限。修改需要令牌确认，只能在未使能时修改，下次使能生效。
- 导航/作业模式：接入控制器现有模式服务，等待四轮交接确认后才报告成功。
  作业模式锁定速度摇杆，网页“作业位移”面板通过已有相对位移 Action 执行运动。
- 作业位移：配置前后/左右距离（米）、转角（度）、速度、加速度及超时；上限从控制器回读。
  显式确认后执行，显示阶段、控制器进度及反馈计算的实际位移；支持取消。

## 网页作业操作

1. 登录操作站，使能底盘，点击“作业模式”，等待实际模式确认及作业就绪。
2. 在“作业位移”填写前后、左右位移及转角，方向相对任务开始时的底盘朝向。
   单次平移合量不超过 0.7 m，旋转不超过 15°；正向为前、左、逆时针。
3. 配置速度；展开运动限制可配置加速度、轮侧限制及超时。底盘旋转速度/加速度在网页使用
   °/s、°/s²，轮侧使用 rad/s、rad/s²，提交时转换为控制器单位，服务端再次校验上限。
4. 点击执行并确认，等待实际终态。执行期间不可重复提交或切换模式。
5. “取消作业”或“停止移动”请求取消；释放控制权、页面隐藏/离开或连接断开也请求取消本网页任务。
   取消请求不等于已经停车，须等待任务结束并核对反馈。结果未确认时禁止再次提交。

运行栈启动默认导航模式。网页不自动使能、切换到作业或恢复历史任务。任务完成后保持作业模式，
需点击“导航模式”返回；返回后等待新的导航速度指令。真实 IMU、模式反馈、静止状态或运动限制
不满足要求时执行按钮禁用。控制器进度与反馈计算位移分别显示，进度不代替实际位置反馈。

## 边界

- 基础接口：`/rt/capabilities`、`/control/set_enabled`、`/rt/reset_fault`（仅 FAILED）、
  `/controller_manager/list_hardware_components`、控制器参数服务、`/diagnostics`、能力声明中的速度话题。
- 模式功能在 RT-Control 域内使用配置控制器的私有 `~/set_mode`、`~/state`，不暴露任意 ROS 调用。
  需显式启用 `relative.enabled` 并满足控制器自身的 profile、IMU 和参数门禁。
  未启用时显示“导航（速度入口）”，不伪称收到实际模式确认；参数/状态过期时拒绝相关操作。
  “对齐/连续”另称“导航行走策略”，不等同于导航/作业控制模式。
- 调试建议值见 `swerve_driver/config/relative.commissioning.yaml`。复用导航几何、方向和
  换算，轮面速度/加速度须按半径换成轮侧 rad/s、rad/s²；估计项有注释。
  `Alfa_V3-Advantech-01` 通过 `chassis_operation` 显式选择该模板，由 composer 从导航配置
  推导轮侧限制并接通 IMU 和四驱动切换；重新生成部署后由 `chassis start` 加载。
  启动仍为导航模式，不自动使能。进入作业要求实际 IMU 有效、四轮新反馈确认。
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
