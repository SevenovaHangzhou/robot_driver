---
id: motion-20260915-01
area: motion
title: HT-WS-HH270 舵轮厂家机械图纸参数提取
date: 2026-09-15
type: investigation
trigger: 用户提供 HT-WS-HH270-17.68-Q750-Z400(2).pdf 补充机械参数
commits: [work/electri-117-swerve-module-binding]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-117, ELECTRI-127, ELECTRI-132, BQ-145]
---

## 背景

三代机四舵轮缺少厂家机械事实。用户提供一张 SolidWorks 总装尺寸图，本次仅提取
图上可直接核对的参数并登记疑点，不把厂家名义值写成实车标定结果。
源文件不进入仓库；登记文件名、图号和 SHA-256 供后续复核。

## 改动

- 新增 `alfa_v3_swerve_mechanics.draft.yaml`，源图号
  `HT-WS-HH270-17.68-Q750-Z400`，SHA-256
  `8feb1c75ce4258e551ba6fddc9b517d55e842fc32d7e7cffdc49d639ad916c1f`。
- 登记名义轮径 200 mm、轮宽 60 mm、安装高度 270 mm、回转包络直径 367 mm、
  8×M8/直径 190 mm 安装参考、直径 160 mm 定位尺寸及 0/-0.10 mm 公差。
- 登记悬挂预压 10 mm、下浮行程 38 mm、自由状态轮面下浮 20.5 mm。
- 登记转向减速机 35:1、回转支撑 108 齿、主动轮 27 齿；
  由图纸数字可计算总传动比 140:1，但未获厂家确认，保持 `verified: false`。
- 登记选用行走减速比 17.68、额定输出扭矩 340 Nm、名义最大速度
  106.6 m/min，以及 400 W 转向和 750 W 牵引电机/驱动器型号。
- machine manifest 只引用该 draft 记录；`controller.draft.yaml` 的四轮半径和
  舵角上下限继续为 `TBD`，`calibration_verified` 继续为 false。

## 验证

- `pdfinfo` 确认文件为 1 页 A4、SolidWorks 2020 生成。
- `pdftotext -layout` 与 300 dpi 整页/参数表视觉核对用于交叉读取文字和尺寸归属。
- 新增测试核对源文件哈希、名义轮径、17.68 减速比、108/27/35 传动数据，
  并断言控制器仍保留逐轮半径和硬限位 `TBD`。
- machine-profile 直接测试：60 passed；隔离 `rt_control_bringup` 构建通过，
  CTest 汇总 63 tests、0 errors、0 failures、0 skipped。
- 安装后的 validator 完成 5 模块、5 profile、10 组合静态检查；
  `chassis_only --require-runtime-ready` 按预期拒绝未确认机械参数。
- `tools/quality_gate.sh`：277 passed、13 skipped，策略覆盖率 83%；
  本机缺少 ShellCheck，保留 CI 检查。

这是图纸静态提取，不是厂家签字确认、装车测量或实机验收。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 厂家图纸给出的轮径 200 mm 是名义几何初值，运行配置仍必须使用装车负载下
  逐轮标定的有效滚动半径。
- F2: 图纸给出转向减速机 35、108 齿回转支撑和 27 齿主动轮；140:1 是计算结果，
  在厂家确认传动链定义前不得标记为 verified。
- F3: 图纸只给出左右限位接近开关为常开 NPN，没有给出机械舵角上下限或零度基准。
  图中 `-20°～200°` 位于润滑脂行，不是舵角范围；温标也未明确。
- F4: 现场逐轮轮径、零位、硬限位、背隙、安装坐标和阈值标定全部保留，
  厂家图纸不能使 `calibration_verified` 自动变为 true。

## 遗留

需要厂家确认：转向电机电流/扭矩两行是否互换、35×108/27 是否为完整电机到舵轴
传动比、数值舵角硬限位和机械零位、外置反馈齿轮齿数、编码器安装方向、
直径 190 mm/63 mm 尺寸基准、牵引重量单位，以及参数表约 35 kg 与标题栏
11.207 质量值的差异。还需整车厂家提供四个模块相对底盘的安装中心、
转向轴到轮接地点偏置和载荷分配。
