---
id: integration-20261029-07
area: integration
title: 恢复导航CSV启动并按范围确认作业静止
date: 2026-10-29
type: fix
trigger: 用户要求修复使能失败、启动导航验证零速度，并要求作业静止按范围判断
commits: []
env: native
risk: T3
writes: { reset: no, enable: yes, motion: yes, plc: no }
verified: PARTIAL
evidence: [validation/chassis-steering-startup-20261029]
supersedes: []
related: [integration-20261029-04, integration-20261029-06]
---

## 背景

18:06:34.977，3号转向速度0.010287 rad/s超过原0.01 rad/s门限，启动CSV零速
确认立即锁存seed_moved。首次修正为等待后，18:48实机仍因等待全部转向静止而超时。
归档1982帧中，3号转向速度峰值0.01855 rad/s、位置跨度0.0000391 rad，最长满足原
门限仅5 ms，无法连续达到50 ms。四轮保持零速度；初始导航并未切换转向模式。
用户进一步明确：导航恢复原行为，导航→作业仍要求行走轮和转向在一定范围内静止。

## 改动

- 初始CSV保持导航原语义：健康实际CSV、行走轮持续零速、全四轮新发送ACK和读回
  通过后准入。单独计时行走轮零速持续时间；反馈中断重计，原switch_timeout不延长。
  不把转向保持的速度波动当作启动位置切换故障，不伪造整车stationary标志。
- 真正CSV/CSP交接仍检查行走轮和转向的完整静止持续时间、健康模式反馈及发送ACK。
  用户授权静止范围：行走轮沿用导航推导±0.1 rad/s，转向设±0.02 rad/s，连续50 ms。
  原始反馈逐样检查，不滤除超限样本；越界重新计时，不恢复旧指令。
- 非实时端点新增首个交接故障原因日志。公共接口、控制字所有者、几何标定、速度上限不变。
- 正式artifact由composer生成并校验，只变controllers.yaml中的转向静止容差；原产物备份。

## 验证

- 新增回归先复现失败，再通过。覆盖初始CSV转向波动、真实零发送确认、未确认拒绝、
  双向非零容差、超限拒绝、完整静止持续时间、反馈异常和CSP返回仍严格检查。
- 两包构建通过，swerve_driver 195、bringup 49测试结果，均零错误/失败；
  含真实控制器/Kinco适配器联合回归。composer配置测试6 passed。
- 中途主机重启后发现安装库和测试程序零字节，已clean-first重建、重新安装并验证ELF。
- 最终实机通过：启动日志 `runs/chassis-20261029T105742Z/launch.log`，日志归档
  `20261029-185743-d173-chassis_only`。使能成功、NAV确认，连续20秒396次零速度请求。
  共享内存只读核对四轮CSV=9、OperationEnabled、反馈有效、sent_velocity始终精确零；
  四轮实际轮速最大绝对值0.000535889 rad/s，发送ACK各前进19959，无控制器更新错误。
- 结束soft_stop、disable均成功，状态IDLE、enabled=false；运行栈和两网页保留。
  未发送作业切换、位移任务或非零速度。实机检查仅覆盖NAV零速度，不构成作业运动验收。
- `tools/quality_gate.sh`：247 passed、13 skipped；`git diff --check`通过。

## 结论与冻结事实

F1：初始CSV导航零速确认与实际CSV/CSP交接分开；导航不要求转向静止，但stationary
标志仍真实检查全部轮系。作业交接采用有限速度容差及持续时间，不要求数学上的精确零。

## 遗留

NAV零速度实机通过；作业模式/非零运动实机验收尚未执行。静止容差为本轮用户要求和现场反馈范围，非通用硬件安全标定。
