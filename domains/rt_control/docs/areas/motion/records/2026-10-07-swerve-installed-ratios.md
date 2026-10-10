---
id: motion-20261007-01
area: motion
title: 登记安装舵轮减速比与运行换算边界
date: 2026-10-07
type: decision
trigger: ELECTRI-145 用户确认行走27.48与转向140并要求更新换算
commits: []
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-145, motion-20260915-01]
---

## 背景

用户确认安装机型行走电机到轮轴为27.48:1，转向电机到舵轴为140:1。
图纸本身记录17.68选型及35级减速机，作为历史来源保留；不由新速比推定
型号、扭矩、速度、编码器单位或电子齿轮。

## 改动

用户指定软件目标±120°，随后确认机械硬限位±135°。控制器草案四轮
steering_min/max为±2.356194490192345 rad，margin为0.2617993877991494 rad（15°），
目标区间为±120°，导航与Tier A共用。机械参数记录两种端点及用户确认来源；
反馈容差、零位/方向和限位开关触发点仍待现场确认，不打开生产准入。

机械参数文件更新行走选型与转向总速比，并登记用户来源。外置舵角编码器
108/27比例独立保留。安装机型扭矩和最大速度不沿用17.68选型的旧数值，保留TBD。
未修改合成测试fixture或伪造真实Kinco运行profile。

用户选择先按手册计算候选；新增硬件owner候选文件
`src/rt_control/robot_hw_ethercat/config/machines/alfa_v3_swerve_conversion.candidate.yaml`，
`verified=false`、`runtime_enabled=false`，不由运行插件消费。
手册V3-DOC-2026-014（SHA-256 `029f858f7e7a4ec1f1299ae47bb6c9d1dcee8de3ef035517e1479d7166d0125e`）
印刷页107–108给出速度raw = rpm × 512 × N / 1875。以电机侧N计数/转且无另行缩放为前提，
行走position比例为4.373577836165284×N，velocity比例为71.65669926773201×N；
转向分别为22.281692032865347×N和365.06324226646586×N，方向符号另定。
通用对象表实际位置为0x6063而EtherCAT章节/ESI为0x6064，ESI无单位；必须确认安装固件
适用单位，不能仅按候选数值宣称原始计数换算闭合。手册示例N=10000不是本机事实。

## 验证

- `python3 -m pytest src/rt_control/rt_control_bringup/test/test_machine_profile.py -q`：52通过。
- `tools/quality_gate.sh`：通过；本机ShellCheck缺席，CI负责补齐。
- `git diff --check`：通过。
此记录仅登记硬件事实，不宣称实机换算验证通过。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 安装机型总机械比为行走27.48、转向140；不是由外置编码器齿比推导。
- F2: 运行时原始计数换算仍需反馈分辨率、速度单位、电子齿轮、方向与零偏。
  若0x6064/0x607A确为电机侧每转N计数、未另有缩放，则
  position_counts_per_unit = sign * N * ratio / (2*pi)。
  速度若确为电机rpm且每rpm为K原始单位，则
  velocity_counts_per_unit = sign * K * 60 * ratio / (2*pi)。
  这些是条件公式，不能代替实际参数回读。

## 遗留

机械文件目前仅被静态配置/校验读取，真实运行profile未生成。
本次不能仅凭两个减速比使运行时raw/SI比例生效，缺失事实继续由BQ-145跟踪。
