---
id: arm-control-20261024-01
area: arm-control
title: 台架操作台增加真空泵及左右阀三个手动开关
date: 2026-10-24
type: feature
trigger: 用户口头要求将已安装的临时 vacuum 命令放入机械臂 Web 操作台（既有 ELECTRI-127 台架）
commits: []
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /home/ar/rt-control-electri127/backups/vacuum-web-20261024-182144/verification.json
  - /home/ar/rt-control-electri127/backups/vacuum-web-20261024-182144/manifest.json
  - /home/kkozia/tools/vacuum-arm-console/live-verification.json
  - /home/kkozia/tools/vacuum-arm-console/vacuum-live.png
  - /home/kkozia/tools/vacuum-arm-console/vacuum-mobile.png
supersedes: []
related: [ELECTRI-127]
---

## 背景

用户已确认现场 IO 接线为 00 右臂电磁阀、01 左臂电磁阀、02 共用真空泵，并要求将临时
`vacuum up/down`、`vacuum left/right open/close` 控制放入现有机械臂网页。此变更保留
目标工作树已有的速度档位、序列、显示模型等未提交修改，不修改主分支真空硬件配置。

## 改动

- `rt_control_arm_console` 新增可选 `VacuumControl`，复用 `/home/ar/.local/bin/vacuum`
  的线圈映射、网口绑定、Modbus 读写校验和进程文件锁；未复制另一套 Modbus 协议实现。
- `GET /api/vacuum` 登录后只读并缓存状态；`POST /api/vacuum` 接收指定输出、目标布尔值
  和预期旧值，沿用同源、会话、控制权及审计机制，运动/序列/使能切换中拒绝。
- 页面控制区增加真空泵、左臂阀、右臂阀三个按钮。状态无效、过期、断线、无控制权或
  操作中禁用；未知状态不显示为关闭。发送明确目标，写结果不明时不自动重发。
- 默认为用户目录 `~/.local/bin/vacuum`，可用 `--vacuum-script` 选择可信本地脚本；不存在则
  功能未配置。启动、刷新、轮询、退出不自动写输出。本入口为现场临时 IO 工具，
  没有接入压力反馈、负载判断或完整真空业务联锁，不替代公共真空服务。
- 现场端点为 `192.168.1.12:502`、站号 1，脚本绑定 `enp4s0`。只重启空闲 Web/ROS bridge
  进程，端口保持 18121，令牌、位姿库及运行环境沿用；机械臂控制栈没有被启停。

## 验证

- T1：本地及工控机分别执行 `python3 -m pytest -q <package>/test`，均为 **105 passed**。
  新增用例覆盖鉴权、同源、控制权、无工具、运行中拒绝、映射、重复读取缓存、非法参数、
  旧状态冲突、通信失败及未知写结果不重发。
- T1：真实 Chromium 浏览器连接本地模拟操作台，三个按钮六次切换落到
  `02/02/01/01/00/00`，登录前后轮询不写、获取控制权后可操作、断线禁用均通过；
  390px 窄屏三个按钮均在面板内，页面无脚本错误。六次开关均只在模拟设备发生。
- T0：JavaScript 语法检查和目标包 `git diff --check` 通过。
- T2：部署前两次检查没有活动运动、序列和使能切换，备份原文件并逐文件校验原 SHA 后
  部署，重启 Web 进程。实际页面和鉴权 GET 读取均成功，三路读回为 OFF；真实浏览器
  检查记录的输出 POST 尝试数为 0，没有使能、运动或 PLC 写入。
- 未验证：真实泵/阀切换及机械动作、压力/吸附反馈、完整生产生命周期；未做全仓构建、
  Docker 封装、提交、推送或 PR。此记录为现场补丁证据，不是主分支发布声明。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本次现场用户确认的输出接线为右阀 0、左阀 1、共用泵 2；真实读通信已验证，实际输出动作未测试。
- F2: 网页加载和轮询只读；三按钮需要人工点击，且只操作对应输出，不自动联动其他通道。
- F3: Web 和同一用户 CLI 复用同一临时脚本和文件锁；线圈读回确认不代表真空已建立或物体已吸附。

## 遗留

实机开关由操作员后续人工验证；正式公共真空接口的压力配置及联锁仍待原任务闭环。
现场补丁没有提交到 Git，迁回主分支时需保留现有用户修改并独立评审临时工具的部署方式。
回退原文件、原启动参数、校验和及重启日志位于
`/home/ar/rt-control-electri127/backups/vacuum-web-20261024-182144`。
