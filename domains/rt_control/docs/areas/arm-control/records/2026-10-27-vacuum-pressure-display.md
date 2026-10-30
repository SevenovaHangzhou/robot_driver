---
id: arm-control-20261027-01
area: arm-control
title: 操作台右上角双路真空表压
date: 2026-10-27
type: feature
trigger: 用户要求分析 arm start 并在实际操作台右上角显示两个真空传感器值
commits: []
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - runs/vacuum-display-20261027-003159/browser-verification.json
  - runs/vacuum-display-20261027-003159/desktop.png
  - runs/vacuum-display-20261027-003159/mobile-header.png
  - runs/vacuum-display-20261027-003159/quality-gate.log
  - runs/vacuum-display-20261027-003159/test-results
supersedes: []
related: [arm-control-20261024-01]
---

## 背景

当前现场 `arm start` 的启动包装器位于部署工作区，依次生成台架产物、启动控制栈、
执行既有启动复位检查、启动运动监护和 `rt_control_arm_console`。网页使用已安装的
Python 包及 symlink web 资源。本次修改实际部署源码，不替换已有现场页面改动。

需求属于 RT-Control 的 `rt_control_arm_console`，直接消费者是已登录浏览器，
阶段为运行时只读遥测；不变更公共接口、控制策略或实时环。
验收条件：右上角独立显示左右压力、单位 kPa；两路对应既有 ROS 话题；无效、
未收到或超时不显示旧值；现有机械臂与 IO 按钮行为保持。

## 改动

ROS bridge 订阅 `/plc_io/vacuum/left` 和 `/plc_io/vacuum/right` 的
`plc_io_modbus/msg/VacuumSensorState`，直接消费已经换算的 `pressure_kpa`。
增加显式消息包运行依赖；接收缓存受操作台锁保护，按各通道单调接收时间独立过期。
事件流增加 `vacuum_pressure`，页面右上角显示一位小数的表压。
超过 1 秒未接收、无效标志、单位错误、非有限数值都清除读数；网页事件流断开或
停滞 1 秒也清除读数。此 1 秒仅是显示时效，不是吸附或运动安全阈值。

## 验证

证据路径相对于现场部署工作区，未将截图或构建产物加入源码。

- `python3 -m pytest src/rt_control/rt_control_arm_console/test -q`：107 passed。
- `colcon build --base-paths robot_driver/src/rt_control/rt_control_arm_console --merge-install --symlink-install --packages-select rt_control_arm_console --cmake-args -DBUILD_TESTING=ON`：PASS。
- `colcon test --base-paths robot_driver/src/rt_control/rt_control_arm_console --merge-install --packages-select rt_control_arm_console`：5 个 CTest 套件全部通过；`colcon test-result` 为 112 tests、0 errors、0 failures（含 CTest 包装结果）。
- `git diff --check`、`git diff --cached --check`、修改的 Python 编译检查、JS module 语法检查：PASS。
- `tools/quality_gate.sh`：PASS，248 passed、13 skipped、27 subtests passed。
  系统 jsonschema 4.26 不符合该部署树要求的 3.2，改用临时 venv 固定 3.2，
  设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 隔离无关 ROS pytest 插件；未改全局环境。
  本机未安装固定版 ruff、clang-format 或 shellcheck，门禁按既有规则提示由 CI 执行。
- 只重启空闲 Web 操作台，保留既有控制栈、运动监护和 IO 节点；未运行 `arm start` 或 `arm restart`。
  新进程实际收到两路有效压力，抽样左 -0.9595 kPa、右 -0.78275 kPa。
- Playwright 访问真实网页并登录：1440px 桌面与 390px 手机宽度下压力区域均在可视范围内；
  桌面完整模型与头部、手机头部截图已检查，未发生页面 JS 异常，未发送控制命令。
  浏览器中验证有限负压、零值、非法数值、无效与超时提示，以及事件流停滞后清除数字。
  为降低软件 GL 的测试开销，首个模型加载后仅在测试浏览器冻结动画；手机完整模型渲染不在本次验证范围。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 左右压力分别来自 `/plc_io/vacuum/left`、`/plc_io/vacuum/right`；现场用户已确认 AI0=左、AI1=右、实物 ZSE30A。
- F2: 页面读取 IO 节点换算后的 kPa 表压；大气压为 0，真空为负，不重复换算或自动归零。
- F3: 显示压力与泵阀命令独立，压力失效不改变控制准入或发出任何输出命令。

## 遗留

压力 IO 节点仍由既有独立启动方式提供，网页订阅不负责启动该节点。
