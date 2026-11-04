---
id: integration-20261029-04
area: integration
title: 修复作业运行接入后首次使能与重试失败
date: 2026-10-29
type: fix
trigger: 用户报告网页无法使能并要求修复
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: ["v3/validation/chassis-enable-20261029/build.log", "v3/validation/chassis-enable-20261029/tests.log", "v3/validation/chassis-enable-20261029/lifecycle-tests.log", "v3/validation/chassis-enable-20261029/quality.log", "v3/validation/chassis-enable-20261029/archive-replay.log"]
supersedes: []
related: [integration-20261029-03]
---

## 背景

16:36:19.874 运行归档记录周期3695.07us，四轮CSV=9、实际和已发送速度均0，
但PDO反馈年龄为inf。启动零速度确认阶段立即锁存反馈故障；后续激活失败返回ERROR，
控制器退到unconfigured，enable适配报jtc_activate_failed。

验收：只在尚未开放导航、保持零速度的CSV启动确认阶段，在既有switch_timeout内等待
完整反馈；持续故障必须超时拒绝，四轮真实发送ACK与静止持续时间仍必须满足。
正常运行/CSV-CSP交接不改变故障保护。可恢复的激活拒绝保留inactive，允许下次显式使能重试。
范围为swerve_driver及无设备真实adapter联合测试；不写控制字，不修改总线或硬件参数。

## 改动

1. 初次CSV零速度确认阶段（source=CSV，seed_csv/confirm_csv）遇到不完整反馈，继续保持
   零速度，撤销旧发送ACK并重启静止持续时间确认。原有switch_timeout不重置、不加长；
   持续缺反馈超时拒绝。状态、模式、实际运动或非有限位置异常仍拒绝；正常NAV、CSP交接
   和作业中的反馈丢失仍立即锁存。
2. 保留 `sample_time = now - age`。真实硬件的年龄在read阶段采样，当前时刻在update阶段
   采样，二者相减的估计时刻存在微秒抖动。显式标识年龄估计，保留其原值用于时效和静止
   持续时间检查，不钳制、不重盖时间戳；真实PDO序号仍必须单调。绝对来源时间戳仍检查倒退。
3. 实际反馈准入被拒绝时返回生命周期FAILURE而非ERROR，保持inactive；下一次显式激活
   可重试，不会仅因这次拒绝退到unconfigured。配置/绑定或程序异常仍为ERROR。
4. runtime锁存故障接入已有feedback_fault/fault_stop反馈，让enable_manager负责既有
   停机/失能流程，控制器不写控制字、不自动复位或重新使能。

## 验证

- 修改前新增回归：启动丢PDO及激活拒绝重试均失败，确认命中缺陷。
- 两包clean build通过，安装新swerve_driver与bringup测试程序；既有编译警告不影响构建。
- `colcon test`：swerve_driver 191、bringup 48结果，零错误/失败（包含CTest条目）。
  新增7个场景覆盖启动缺反馈恢复/超时/运动拒绝、估计时间抖动及过期拒绝、真实适配器
  连续不完整PDO、真实生命周期activate失败保持inactive再激活成功、运行中故障通知owner。
- 归档只读核对：故障附近0.2s有197帧有限反馈，其中85次年龄反推时间出现倒退，最大约4.53us。
  离线重放故障附近131帧：使用真实归档反馈与离线模拟发送ACK，成功通过零速度/静止/NAV
  准入，未连接总线。该结果不构成真实发送ACK或实机使能验证。
- `tools/quality_gate.sh`：247 passed，13 skipped；`git diff --check`通过。
- 正式chassis_only产物与当前已安装composer校验通过；未改变标定、阈值或产物参数。
- 当前旧运行进程不会自动加载新库，需由用户失能后经chassis stop/start重载，再现场验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1：启动前零速度确认可在原有截止时间内等待完整PDO，期间不开放导航，不复用旧ACK。
- F2：年龄反推时刻保留原始值用于时效/静止检查，PDO序号保持顺序验证；不把估计微抖动
  当作绝对来源时间倒退，也不通过刷新估计时刻伪造反馈。
- F3：可恢复的激活拒绝保持inactive；实际运行锁存故障仍上报既有enable_manager安全收尾。

## 遗留

修复后实机使能待用户执行；不把无设备测试作为现场验收。
