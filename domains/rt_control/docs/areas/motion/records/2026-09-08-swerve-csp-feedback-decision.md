---
id: motion-20260908-02
area: motion
title: 三代机舵轮保留转向 CSP 与外置舵角观测
date: 2026-09-08
type: decision
trigger: 用户确认保留转向 CSP，关联 ELECTRI-118
commits: []
env: none
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-144, release-deploy-20260907-01]
---

## 背景

用户确认步科驱动器不支持外置编码器输入，并选择不增加实时舵角补偿的 CSP 路线。
同时确认迁移包名为 `swerve_driver`，移除达妙 `rezero`，编码器采用 4 ms SYNC + TPDO。
外置编码器只测转向输出轴，不测驱动轮滚动速度。

## 改动

无运行代码改动。记录 BQ-144，冻结后续舵轮迁移的控制与观测边界；
现有 alfa_v3 manifest 的 steering_csp(4)、drive_csv(4) 和四个 state-only 编码器保持不变。
迁移实现中删除达妙 MIT `kp/kd` 及驱动轮 `ks/kv/ka` 力矩前馈配置、计算和命令输出，
不保留旧参数占位。步科内部环参数按其厂商对象独立整定，不与 MIT 参数作数值映射。

## 验证

本记录为设计裁决，不是实现或实机验收。仅检查现有 manifest 与决定一致；
机械精度、动态跟随、反馈时效和停车/降级行为仍未验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 包名为 `swerve_driver`；转向保持 CSP(8)，驱动轮保持 CSV(9)，不增加主站或独立底盘 EtherCAT 控制环。
- F2: 外置编码器是实际舵角主数据源，用于方向校验、优化/余弦补偿、门控、里程计及异常检测；
  它不是步科内部位置环或上位机转向位置外环的反馈源。
- F3: 转向电机位置和输出轴舵角分别保存。目标映射由批准的齿比、方向和零位确定，
  不在运行中累积角度修正或自动重写零点。
- F4: 不把实际舵角观测描述为已消除背隙/弹性；输出轴误差若超出允许范围，按明确策略门控/告警。
- F5: CANopen 编码器按 4 ms SYNC 触发同步 TPDO；达妙 `rezero`、MIT 增益和驱动轮力矩前馈
  不属于新后端运行时接口。
- F6: 4 ms 为设计周期，不是采样同步精度或反馈时延的实测结论；不保证 CANopen 与 EtherCAT DC 同相。

## 遗留

舵轮 controller 与步科/CANopen 接入另行实现。PDO/SDO、机械标定、反馈时效和停车/降级条件
仍待补齐，三代机实机启动继续由 draft/runtime gate 阻塞。
`swerve_driver` 包本体尚未迁入当前分支，命名与功能删除在迁移时落实；不修改头部达妙协议归属。
