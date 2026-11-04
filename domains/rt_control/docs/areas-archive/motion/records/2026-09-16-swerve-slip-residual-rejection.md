---
id: motion-20260916-01
area: motion
title: 四舵轮最小二乘残差剔除与里程计隔离
date: 2026-09-16
type: feature
trigger: ELECTRI-117，继续迁移四舵轮协议无关的残差校验
commits: [work/electri-117-swerve-module-binding]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [ELECTRI-117, BQ-145, motion-20260909-01]
---

## 背景

main 的 `swerve_driver` 已能用普通最小二乘从四轮实测速度反解底盘速度，但单轮空转、
打滑或异常速度会同时污染 twist 和轮式里程计。旧四舵轮分支已有可参考的固定规模残差
剔除，本次只迁移协议无关的数学和观测语义，不恢复旧电机后端或独立总线生命周期。

## 改动

- `swerve_kinematics` 对四个轮速向量做 3 自由度最小二乘拟合；残差超过
  `slip_residual_threshold` 时逐次剔除最大残差轮，直到得到一致解或剩余两轮仍不一致。
- `ControlCore` 用同一剔除掩码过滤实测 twist 和当周期位置里程计。被剔除轮不参与积分，
  但其 position baseline 仍推进，恢复后不会补入隔离期间的累计位移。
- `ControlOutput` 导出 used count、`slip_detected` 和 FL/FR/RL/RR 掩码；控制器诊断通过
  原子位掩码跨线程发布，不从诊断线程读取实时输出对象。
- 新增 `slip_covariance_scale`，与已有 IMU fallback 和 missing-module scale 相乘；8 个组合
  在 configure 阶段预计算，update 只做固定索引。
- draft 配置保留残差阈值和协方差倍率为 `TBD`。阈值单位为 m/s，必须由健康与诱发打滑
  数据分布确定，不能从厂家名义机械参数猜测。

## 验证

- TDD 首先复现缺失的残差接口、控制核心输出和协方差参数，再完成实现。
- 纯 C++ 小型回归：35 tests 通过，覆盖单轮异常定位、共模盲区、非法阈值、pose 隔离和
  恢复无补跳。
- Native `colcon build/test --packages-select swerve_driver`：6 个测试目标、70 tests，
  0 errors、0 failures、0 skipped。
- 控制器级测试用合成反馈制造 RL 单轮异常，确认 diagnostics 报告 `RL` 且 pose covariance
  按 2（IMU fallback）×4（missing module）×5（slip）放大为 40。
- ASan+UBSan 纯算法/核心回归：35 tests 通过，无 sanitizer 报错。
- CANopen/EtherCAT/machine/governance 受影响配置回归：149 passed；`tools/quality_gate.sh`：
  282 passed、13 skipped，策略覆盖率 83%。本机没有 ShellCheck，留待 CI 执行。
- 未执行 EtherCAT/CANopen 总线、驱动使能、运动或实机打滑试验；阈值、误报率、检出延迟和
  实际轮胎/地面条件均未验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 残差剔除属于 `swerve_driver` 整车观测，不属于 EtherCAT 或 CANopen 后端。
- F2: 单轮被剔除时，异常轮不进入当周期 twist/pose，但 baseline 必须继续推进以避免恢复跳变。
- F3: 四轮共同满足另一组刚体速度的共模误差残差可为零，单靠轮速几何无法识别；仍需 IMU
  或外部定位提供独立观测。
- F4: 当前实现只改变估计质量、诊断和协方差，不把残差告警升级为新的驱动停车/恢复策略。

## 遗留

在封闭场地采集直行、侧移、原地旋转、组合运动和诱发单轮打滑数据，按误报/漏报与连续周期
分布确定 `slip_residual_threshold` 和 `slip_covariance_scale`。如需由残差触发停车或锁存，必须
另行裁决安全策略，不能从本次估计隔离自动推导。
