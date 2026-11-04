---
id: integration-20261029-08
area: integration
title: 修复固定CSV监控阻止作业切换并验证零位移
date: 2026-10-29
type: fix
trigger: 用户授权chassis start、导航切作业、使能及零位移验证，遇错修复
commits: []
env: native
risk: T3
writes: { reset: no, enable: yes, motion: yes, plc: no }
verified: PARTIAL
evidence: [validation/chassis-operation-zero-20261029]
supersedes: []
related: [integration-20261029-07, integration-20261029-05]
---

## 问题与改动

19:07:02.420实机NAV→OP切换：轮子6061开始从9变8，仍Operation Enabled、无驱动错误，
enable_manager以固定expected_modes=9判mode_mismatch，立即快停/失能；控制器随后报告drive故障。
问题在使能管理与作业控制器的模式策略不一致，非静止容差问题。

新增冻结的cyclic_mode_switch_joints白名单，仅允许expected_modes=9且由运动控制器管理的
指定轴在已使能/正常控制器停用期间反馈8或9。启动及控制器激活仍须CSV=9；模式0、非法值、
非白名单轴及驱动失能仍立即停机。CSV/CSP交接的静止、预置、发送ACK和新反馈确认继续由
swerve_controller执行。composer只对启用作业的底盘输出四个行走轮白名单；转向保持固定CSP。
未改公共接口、硬件比例、速度限制、控制字所有权或现场静止门限。

首次流程通过后，复测重现第二个问题：正常失能未重置Kinco CSV等待新指令标志，
重新使能时尚无生产者的NaN速度被标为mode_request_error，控制器拒绝激活。
驱动现在在失能反馈分支清除旧速度、重置fresh-command等待；重新使能保持零速到新命令。
新增无设备回归先失败后通过，同时证明旧非零命令不能跨失能恢复；实机始终仅发零指令。

## 验证

- enable_manager构建通过，92测试结果无错误/失败；新增允许轴往返、启动严格CSV、模式0拒绝、
  CSP正常停用及驱动失能不豁免回归。composer 225结果通过，含四轮白名单/转向不包含断言。
- 正式artifact只增加四轮白名单，由composer生成，原产物备份并核对完整性。
- Kinco和bringup重新构建通过；149+49结果通过；四包合计515结果，均无错误/失败。
  质量门禁247 passed、13 skipped；git diff --check通过。
- 最终运行 `runs/chassis-20261029T111355Z/launch.log`，PID40719：同一进程连续两轮
  使能NAV→5秒零速度→切OP→3秒保持→提交dx=dy=dyaw=0→任务成功→返回NAV→5秒零速度→失能。
  两轮均enable/切换/operation_result/disable成功，包括第二轮重新使能。使用操作站真实WS，
  backend_kind=real，非mock；网页作业成功必须Action状态SUCCEEDED=4且结果code=0。
- 只读SHM归档56988帧：全部sent_velocity=0；两段四轮均6061=8并满足CSP状态字
  0x106f/0x1027，各连续3319样本约3.318秒，实际轮速均零，反馈年龄0ms。
  该证据覆盖零位移与保持，不宣称位置精度或非零运动验收。
- 两轮结束均IDLE、enabled=false、fault_code=0；默认启动NAV，运行栈与18110/18200保留运行。
  未发起reset、非零实机运动、PLC操作或手工SDO写入。

## 冻结事实

F1：作业配置的行走轮允许已使能期间CSV/CSP交接，首次启动固定CSV，转向固定CSP；
白名单不是切换成功确认，四轮新反馈与发送确认仍由模式交接器负责。
F2：Kinco失能清除旧CSV速度并重置等待新指令，再使能零速保持不报空指令错误。

## 未验证

非零运动、长期运行和机械运动精度不在此次零位移验证范围内。未提交或推送。

## 自审

变更范围：integration-20261029-08；enable_manager、robot_hw_ethercat、composer及回归/记录。
架构审查：域边界PASS；公共接口/模型兼容PASS（未修改）；实时固定存储与安全链PASS；
生命周期、正常停机与重复使能PASS；普通native部署，无Docker发布影响。
验证证据：四包构建/515测试结果PASS、quality_gate PASS、两轮实机零位移PASS。
Git审查：存在并保留用户无关改动；生成物与实机原始数据在仓库外validation归档；
无新增凭据，未提交/推送（无授权）；未发现本次改动的阻断问题，非零运动及长期验收仍未执行。
