---
id: integration-20261029-06
area: integration
title: 实机启动及零指令使能检查复现故障
date: 2026-10-29
type: commissioning
trigger: 用户明确要求执行chassis start、使能并检查两模式，允许零速度和零位移
commits: []
env: native
risk: T3
writes: { reset: no, enable: yes, motion: yes, plc: no }
verified: PARTIAL
evidence: [validation/chassis-real-zero-20261029-1806]
supersedes: []
related: [integration-20261029-04, integration-20261029-05]
---

## 背景

用户明确授权真实启动、使能、模式验证及零指令。仅允许零速度和零位移，不提交非零运动。

## 改动

无生产代码或参数改动。执行现有 `chassis start`，使用已安装网页协议提交一次使能；
只有导航就绪后才计划切换作业并提交零位移。异常时停止测试、请求软件停止及失能。

## 验证

- 启动成功，8从站完成预检，操作站及日志站HTTP 200。
- 使能接口返回成功，随即 `swerve_controller` 实时更新返回ERROR；控制器进入motion_fault。
- 日志归档 `20261029-180502-d173-chassis_only`：18:06:34.974控制周期3.647 ms；
  18:06:35.066使能失败阶段转motion_fault。
- 未等到导航模式就绪，未发送作业切换和零位移Action，不宣称任一模式实机验证通过。
- 收尾已请求soft_stop和disable；disable返回motion_fault。
  随后只读确认connected=true、state=FAILED、enabled=false；八轴状态字均不满足
  OperationEnabled判据，四个行走轮反馈速度为零。未复位、未重试使能。
- 启动日志：工作区 `runs/chassis-20261029T100501Z/launch.log`。
  验证脚本/日志归档 `validation/chassis-real-zero-20261029-1806`，不含令牌。

## 结论与冻结事实

F1：本次真实启动成功，但使能后的运行稳定性检查失败；网页使能成功响应不能代表后续模式就绪。

F2：实机两模式及零位移验证仍未完成，前序T1无设备通过不构成实机通过。

## 遗留

底盘运行栈和两网页保持运行，底盘故障且未使能。需定位实时反馈/交接错误后再验证，
本轮未调整保护条件或绕开模式准入。
