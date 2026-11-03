---
id: arm-control-20261023-06
area: arm-control
title: J6吸盘自重正负重力诊断及固定姿态准备预设
date: 2026-10-23
type: feature
trigger: 用户确认吸盘就是URDF内left_tool_link，要求给18121添加对应测试序列
commits: []
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [arm-control-20261023-05]
---

## 背景

任务DPI-J6-MEASUREMENT-SEQUENCES-20261023；rt_control_arm_console静态数据增量，无跨域接口、后端或实时环变更。
吸盘就是URDF left_tool_link，2.76858828258591kg已计入；未吸附，塑料件与URDF外吸管均已拆，不重复添加工具质量。

## 改动

- ⑦ DPI_J6_TOOL_GRAVITY_SIGN_8S：零位后，M3/j3/M3/零位/M1/j1/M1/零位三轮，25点、每点8秒、leisure。
- ⑧ DPI_J6_KNOWN_TORQUE_PREP_8S：零位/M2/j2，3点、每点8秒，仅准备固定姿态。
- 保留全部58项原预设、保存位姿、档位、音效、50点上限和FJT 2000点上限。

## 验证

当前运行URDF与采集快照一致，left_tool_link已存在。纯preparePreset验证全部60项通过；几何只复用现有保存位姿走廊，位置内侧2度及当前S曲线限点通过，最多527点；两组从零位估算321.14/28.50秒。
预设JSON和直接文档先核对哈希、备份后原子更新；HTTP回读检查，不重启服务，不发送动作。
远端git diff --check与cached检查通过；全仓quality_gate在改动前因既有jsonschema版本断言失败，未记为通过。实际实机运动和工具环境碰撞未验。
完整快照、候选、轨迹/校验和部署证据在用户Dynamic_Parameter_Identification/artifacts/j6_measurement_sequences_20261023。

## 结论与冻结事实

- F1: 吸盘为已建模left_tool_link；两组均不添加额外质量，不改模型/PDO/控制器/使能状态。
- F2: ⑦只是名义重力正负切换诊断，不等于独立已知力矩标定；j3/j1名义J6重力约+5.066/−4.849Nm。
- F3: ⑧只是到位准备，不自动施加±1/±2Nm外力矩，也不自动生成固定姿态全程录制；加载需人工或专用装置，0x2241同步通道尚未准备。

## 遗留

0x2241未采集，独立J6标度/滞后测试仍需准备通道和已知双向加载。相同姿态重复点可能不发送JTC，固定姿态测试必须另开连续手动实验记录，不能只靠自动goal历史尾段。用户需自行选择预设并确认现场条件后执行，本次仅添加预设。
