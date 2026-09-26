# rt_control_operator_web

ELECTRI-109 移动端 Web 操作台。V1 实现底盘摇杆驾驶（平移蟹行 + 原地旋转）、档位、
底盘俯视图与 8 路超声波倒车雷达、控制器状态、电量和常驻“刹车”按钮（即软件停止），且**只接入 Mock
底盘控制器**。

## 边界

- 后端为 rclpy + aiohttp 单进程，只开放白名单 WebSocket 请求：`auth`、`acquire`、
  `release`、`drive`、`halt`、`gear`、`soft_stop`。浏览器看不到 ROS graph。
- `command_topic` 不允许为 `/cmd_vel_safe`：N-04 规定 Motion 是其唯一生产者。实机底盘
  路径待 ELECTRI-142 修改公共契约后解除（BQ-153）。
- “刹车”（软件停止，协议消息 `soft_stop`）以更快的减速曲线停车并收回租约，**不是急停**，也不调用 `/rt/disable`。
- 超声波距离与提示音**仅作显示**，不参与减速或停车。提示音按最近距离连续变化：2.5 m 外静音，
  间隔约 1.0 s 渐变到 0.08 s，0.3 m 内长鸣，越近音调越高（`web/beeper.js`）。
- 操作台为独立进程，部署时应放在非实时 CPU；不映射任何设备。

## 驾驶语义

| 项 | 值 |
| --- | --- |
| 坐标 | REP-103 `base_link`：摇杆上 = +X，左 = +Y；逆时针 = +yaw |
| 控制权 | 进入“底盘”页自动获取，离开页面/切后台自动释放；软件停止或超时后暂停，触摸摇杆/旋转按钮即恢复 |
| 摇杆（游戏式） | 从摇杆上按下后全屏跟踪手指，拖出摇杆区域保持满偏；松手后摇杆头跟随实际执行速度回中 |
| 摇杆整形（服务端） | 死区 10%，幅值平方曲线，正前/后/左/右 ±8° 吸附；输入幅值 ≤ 1.01（容忍前端舍入）并截到 1 |
| 平移与旋转 | 互斥；同时输入即拒绝并按正常曲线停车 |
| 档位 | 配置给出（Mock：休闲 0.05 m/s·0.1 rad/s，运动 0.15 m/s·0.3 rad/s）；仅停车时可换；运动档时驾驶相关元素改为橙色主题、摇杆外圈流光、顶栏 SPORT 标识，切换时闪光并有升/降调提示音（安全色不变） |
| 平滑 | 50 Hz；平移按速度向量整体限加速度，yaw 单独限；均带 jerk 限制（S 曲线） |
| 前端发送 | 变化即发，最快 33 ms 一条；静止时 50 ms 心跳 |
| 看门狗 | 有运动目标时 200 ms 无输入：正常曲线减速并收回租约 |
| 松手 / 断连 / 释放 | 正常减速曲线到 0（不直接发 0） |
| 软件停止 | `stop_decel` 与按比例放大的 jerk，快速减速并收回租约 |
| 进程退出 | 直接补发 0，由控制器自身停车兜底 |

`swerve_driver` 收到零速度时会在同一周期把驱动轮速度清零，因此平滑减速必须由命令生产者
完成，这正是本包在服务端做 S 曲线的原因。

## 配置（全部必须显式给出）

| 参数 | 说明 |
| --- | --- |
| `bind_host` / `port` | 字面 IP（禁止通配地址）与端口 |
| `command_topic` | 目标控制器私有 `~/cmd_vel`；禁止 `/cmd_vel_safe` |
| `controller_node` | 控制器节点名；启动后读取其 `module_x/module_y/wheel_radius/*_joints` 画俯视图，不另存几何 |
| `joint_states_topic` | 舵角与轮速来源 |
| `robot_description_topic` | 共享 Robot Model（通常 `/robot_description`，latched）；俯视图从中派生 |
| `chassis_root_link` | 底盘根 link（V3：`chassis_base`） |
| `battery_topic` | 通常为 `/battery_state`（R-OUT-04） |
| `gear_names` / `gear_linear_speeds` / `gear_angular_speeds` | 档位；实机值 TBD |
| `linear_limits` / `angular_limits` | `[accel, decel, stop_decel, jerk]`；实机值 TBD |
| `ultrasonic_topics` / `ultrasonic_layout` | 通道话题与 `<角>:<朝向>` 示意布局 |
| `ultrasonic_layout_confirmed` | 映射未确认时界面显示“通道映射待确认” |
| `static_dir`（可选） | 覆盖已安装的 `share/rt_control_operator_web/web` |

访问令牌由环境变量 `RT_OPERATOR_WEB_TOKEN_FILE` 指向的文件提供：普通文件、权限 `600`、
非空且不含空白字符。按用户 2026-09-25 决定不设最小长度（可为 1 个字符），因此对猜测几乎没有防护，
只应在受信任的局域网中使用。令牌不进仓库、不进日志。V1 为局域网 HTTP，无 TLS。

## 底盘俯视图（来自共享 Robot Model）

后端订阅 `/robot_description`，在零位下沿 URDF 运动学树把 `chassis_root_link` 子树的每个
visual mesh（STL）投影到 `base_link` XY 平面，栅格化后提取带开孔的矢量轮廓（4 mm 栅格、
3 mm 简化），经白名单请求 `get_model` 发给前端。控制器 `steering_joints` 对应的转向子树
（转盘 + 轮子）按转向轴单独输出，前端用 `/joint_states` 的舵角绕真实转向轴旋转。构建在后台
线程进行，不阻塞 50 Hz 指令发布；模型缺失或关节不匹配时退回控制器几何示意图并提示。
控制器 `module_x/module_y` 与 URDF 转向轴偏差超过 1 cm 时界面提示“控制器几何与模型不一致”。

V3 事实（`robot_description` 构建副本）：底板 1.00 × 1.00 m；转向轴在 `base_link` 下为
x = 0.115 / −0.495、y = ±0.305；`base_link`（旋转中心）位于底板中心前方 0.19 m。
URDF 转向关节轴朝下（caster 坐标系 roll = π），即 URDF 关节正方向为俯视顺时针，
与 `swerve_driver` 的逆时针为正相反；舵角符号/零位映射属于硬件适配与标定，仍待确认。
俯视图按控制器约定（逆时针为正）绘制。

## Mock 闭环

`test/mock/` 为仅测试资产，不安装、不属于任何生产 launch：mock_components 模拟
4 转向 + 4 驱动（关节名沿用 V3 `caster0N_joint`/`wheel0N_joint`），外置编码器 mimic
转向电机，`swerve_controller` 使用合成参数（模块位置取自 V3 CAD，非标定值）；
`robot_state_publisher` 发布 V3 `robot.urdf`；
`mock_sensors.py` 发布合成 `/battery_state` 和 8 路 `Range`：障碍物先从前方（ch1、ch3）由 3.0 m
靠近到 0.15 m 再远离，随后在后方（ch5、ch7）重复，40 s 一轮，便于听提示音；第 8 路周期性断流。

```bash
printf '%s' '<至少16位令牌>' > /tmp/op-token && chmod 600 /tmp/op-token
RT_OPERATOR_WEB_TOKEN_FILE=/tmp/op-token ROS_DOMAIN_ID=<隔离域> ROS_LOCALHOST_ONLY=1 \
  ros2 launch src/rt_control/rt_control_operator_web/test/mock/mock_chassis.launch.py
# 浏览器打开 http://127.0.0.1:18109/
```

`test/test_mock_loop.py` 自动验证：首条非零命令延迟 ≤ 100 ms；蟹行四轮 ±90°；松手单调
S 曲线减速且不直接跳 0；逆时针切向转舵；停止发送后 200 ms 看门狗收回租约并平滑停车；
软件停止；运动中断连停车；无 `/cmd_vel_safe` 发布者。Mock 通过不代表实机或 HIL 验收。
