---
id: contract-20260925-01
area: contract
title: Web 操作台 V1 底盘摇杆驾驶与倒车雷达（Mock）
date: 2026-09-25
type: feature
trigger: ELECTRI-109；BQ-153；ELECTRI-142
commits: [feature/electri-109-web-operator]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: []
supersedes: []
related: [BQ-153, ELECTRI-142]
---

## 背景

ELECTRI-109 由 Qt 操作台改为移动端 Web 操作台（用户 2026-09-25 裁决）。V1 范围为总览、
底盘摇杆驾驶、档位、底盘俯视图与超声波倒车雷达、控制器状态、电量和常驻软件停止。
N-04 规定 `/cmd_vel_safe` 由 Motion 唯一生产，故 V1 只接 Mock 控制器私有话题，实机路径
登记为 BQ-153 并由 ELECTRI-142 修改公共契约。旧分支
`feature/rt-control-operator-console@d33e06e` 不合并；诊断/故障模块留 V2。

## 改动

- 新增 `src/rt_control/rt_control_operator_web`（ament_cmake + python）：
  - `teleop`：单租约；摇杆整形（死区 10%、平方曲线、±8° 轴向吸附）；平移/旋转互斥；
    档位仅停车可换；50 Hz jerk 限制 S 曲线（平移按向量限幅，yaw 独立）；松手、断连、
    释放与 200 ms 看门狗均按正常减速曲线停车，软件停止用 `stop_decel` 与按比例放大的 jerk。
  - `protocol`：白名单 JSON（`drive/gear/halt/...`），拒绝非有限数与非法 yaw。
  - `config`：字面非通配 IP、禁止 `/cmd_vel_safe`、令牌文件 600、非空无空白（按用户决定不设最小长度，可为 1 位）、限值与超声波布局校验。
  - `server`：aiohttp，首帧令牌认证、同源校验、CSP/no-store，任何被拒请求即转为停车目标。
  - `node`：发布 `robot_interfaces_qos.control()`；读取控制器几何参数而不另存；订阅
    `/joint_states`、控制器诊断、`/battery_state`、8 路 `Range` 与 `/robot_description`。
  - `robot_model`：零位 URDF 运动学 + STL 投影，提取带开孔的底盘俯视矢量轮廓；转向子树按
    转向轴输出供前端随舵角旋转；后台线程构建；与控制器模块位置偏差 > 1 cm 时告警。
- 纯静态前端 `web/`：三级对比深色层次；顶部手机式状态栏（连接、控制权、电量）；左侧游戏式
  摇杆（拖出区域仍有效、松手后摇杆头跟随实际执行速度回中）+ 摇杆上方一行的逆/顺时针
  圆形旋转按钮；右侧上方
  操作条（档位、控制器状态、静音、软件停止）、下方底盘俯视图（舵角、轮速、四角各两只 90°
  超声波的 5 层倒车雷达、声波动画、每角最近距离）；进入底盘页自动获取控制权；提示音持有
  控制权时响、可静音，节奏随最近距离连续变化（2.5 m 外静音、约 1.0 s → 0.08 s、0.3 m 内长鸣，
  越近音调越高），距离更新只会把下一声提前、不会打断节奏。软件停止按钮在界面上名为“刹车”（仍标注“非急停”）；
  运动档切换为橙色驾驶主题（安全色不变）。
- 测试专用 `test/mock/`（不安装）：8 关节 mock_components（V3 关节名）、编码器 mimic 转向、
  合成 swerve 参数（模块位置取 V3 CAD：x 0.115/−0.495、y ±0.305，轮半径 0.1 m）、
  `robot_state_publisher` 发布 V3 URDF、合成电池与超声波；Mock 值：档位 0.05/0.15 m/s、0.1/0.3 rad/s，平移
  `[0.3, 0.5, 1.0, 2.0]`、旋转 `[0.6, 1.0, 2.0, 4.0]`（accel/decel/stop_decel/jerk）。
- CI 包闭包与 `repository_gate.DOMAIN_PACKAGES` 加入 `rt_control_operator_web`。

## 验证

T1（本机无硬件、`ROS_DOMAIN_ID=109`、`ROS_LOCALHOST_ONLY=1`）：

- 隔离工作区 `colcon build --symlink-install`（robot_interfaces_qos@9aa2693、
  rt_control_semantic_components、swerve_driver、rt_control_operator_web）：PASS。
- `colcon test --packages-select rt_control_operator_web`：136 tests、0 failures（含俯视图提取单测与 Mock 闭环中 V3 模型/控制器几何一致性检查）。
- Mock 闭环：首条非零命令延迟 4–21 ms（≤ 100 ms 门限）；蟹行四轮 ±90° 且驱动有速度；
  松手后命令单调递减、每 20 ms 降幅不超过减速度约束、末值为 0；逆时针切向转舵；停止发送后
  看门狗收回租约并平滑停车；软件停止与运动中断连均停车；无 `/cmd_vel_safe` 发布者；
  全部进程 SIGINT 后 clean exit。
- 单测：S 曲线加速度与 jerk 不超限、无超调；斜向松手方向不变；软件停止快于正常停车。
- 前端：`node --check`；headless Chromium 横屏 740×360/844×390/932×430 一屏无滚动、竖屏
  390×844、桌面 1440×800 截图无控制台错误；拖出摇杆区域保持满偏，松手后摇杆头约 0.6 s 回中。
- 修复：摇杆满偏时前端 3 位小数舍入使幅值略超 1，被服务端判为 `invalid_input` 而停车；前端改为
  向零截断，服务端容忍 ≤ 1.01 并截到 1（新增回归测试）。
- 未验证：手机真机触摸与提示音、Docker、实机。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 操作台单租约；摇杆平移与旋转按钮互斥；前端 33 ms/50 ms、服务端 200 ms 看门狗、50 Hz 发布。
- F2: 停车一律由操作台以 jerk 限制曲线减速到 0；软件停止更快，但不是急停、不调用 `/rt/disable`。
- F3: V1 配置层拒绝 `/cmd_vel_safe`；实机底盘路径由 BQ-153/ELECTRI-142 阻塞。
- F4: 超声波通道布局为示意且未确认；距离与提示音仅作显示，不参与停车。

## 遗留

- ELECTRI-142：N-04 维护生产者契约修改与 Motion 联合评审。
- V3 舵角符号/零位在 URDF 关节与 `swerve_driver` 之间的映射待硬件适配与标定确认。
- 令牌不设最小长度且无失败次数锁定，局域网内可被快速猜中；接实机前需重新评估访问控制。
- 实机档位、加减速与 jerk 保持 TBD；超声波通道映射与安装外参待确认。
- 操作台部署入口（原生/容器、非实时 CPU 绑定）未实现；手机真机触摸与提示音人工验证待做。
- V2 使能管理/故障诊断待按 E94 语义迁移。
