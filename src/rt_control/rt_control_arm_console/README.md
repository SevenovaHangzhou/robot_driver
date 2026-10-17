# rt_control_arm_console

单臂关节网页操作台（ELECTRI-127）。用滑块设定 7 个关节目标，确认后以 S 型曲线发送。
页面显示 Robot Model：实体为实测位姿，半透明为目标预览。

## 边界

- 操作台只调用公共接口：
  - `/rt/capabilities` 中的 `arm_joint_trajectory`（FollowJointTrajectory action）；
  - `/control/set_enabled`；
  - `/controller_manager/list_controllers` 与参数读取；
  - `/joint_states`、`/diagnostics`。
- 操作台不写驱动控制字，不切换控制器，不自动使能，不复位故障。
- 所有检查在服务端执行。浏览器只提交目标。
- “停止”是软件平滑停车，不是急停。

## 运动规则

- 轨迹为 7 段 S 型曲线（加加速度分段恒定），起止速度和加速度为 0。
- 所有关节沿关节空间直线运动，同时出发、同时到达。移动量最大的关节使用档位上限。
- 档位（`config/console.yaml`）：

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
- “到达”使用同一确认和运动流程。

## 启动

1. 加载工作区环境，选择与运行栈相同的 `ROS_DOMAIN_ID`。
2. 写入令牌文件，权限 0600。
3. 启动：

   ```bash
   ros2 run rt_control_arm_console rt_control_arm_console \
     --bind <主机局域网地址> --port 18120 --token-file <令牌文件>
   ```

4. 浏览器打开 `http://<主机地址>:18120/`，输入令牌。

运行栈必须使用发布 `arm_joint_trajectory` 能力的 scope，例如 `bench_left_arm`。

## 浏览器库

`web/vendor/` 内为离线使用的 three.js 与 urdf-loader，来源和许可证见 `web/vendor/README.md`。
