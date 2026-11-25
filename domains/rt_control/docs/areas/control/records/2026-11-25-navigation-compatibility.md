---
id: chassis-control-20261125-01
area: chassis-control
title: 恢复已发布FollowMPPI协议及生产核心预测一致性
date: 2026-11-25
type: corrective
trigger: 用户批准导航评审F1/F2/F3修正；robot_driver#339
commits: [feature/navigation-adaptation]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: [/tmp/navigation-compat-driver-log, /tmp/navigation-compat-driver-build/ctrl_swerve/test_results, /tmp/navigation-compat-interface-log]
supersedes: [chassis-control-20261124-01]
related: []
---

## 背景

导航真实消费者拒绝原PR336的31+4×15布局及固定NaN舵速，预测缺少整体分支/逐轮保持等内部状态。[纠正任务#339](https://github.com/SevenovaHangzhou/robot_driver/issues/339)记录需求与裁决；[接口纠正PR27](https://github.com/SevenovaHangzhou/robot_interfaces/pull/27)替代已合并PR24的未部署格式。

用户明确确认错误格式未被其他运行消费者采用，批准保留现有Topic/version1/91恢复27+4×16；不混发。模型、总线模块身份、真实标定不随导航附件示例覆盖。保留独立Navigation域，不回退接口main上其他变更。Linear授权不可用，沿用明确批准的GitHub-only例外。

## 改动

- ROS时间索引1、序号2、模块起点27/步长16，带真实模块ID；直接导出translation_branch、holding、gate/release/flip_hysteresis。执行Angle采用生产核心的电机实测控制轴角；Error来自实际优化分支；停稳JointState另用外编码器。
- 舵速由至少两个新鲜外编码器观测估计。当前CANopen没有设备时间戳/序号，使用1kHz轮询4ms SYNC时接收年龄严格下降与重建单调接收时间；重复不差分，断流/复位/异常跳变/生命周期重建历史，无效NaN。不能声称设备采样同步或能识别未暴露的隐藏设备复位。
- active采用reliable/volatile/depth1，与导航订阅兼容；两路执行状态同一快照，全91有限才允许ready。
- 移植附件生产核心prepare/finish包络，纯轴向使用完整平移加速度预算。纯旋转接平移检查角速度与角加速度；已经运动的平移不被浮点角速度越界反复清零。
- 用户看到实测离线回放差异后，明确选择采用参考撤权策略：不立即清零正在向外的角加速度，允许整形阶段短暂继续加速，但遵守新的角加速度包络、最终收敛，纯旋转接平移保护和紧急停车保留。不能再引用旧“撤权后角速度立即单调下降”策略。

## 验证

交接包完整SHA256清单通过。外部参考Navigation v3.0.4 `3f09209babca43a716d842f58fe30a5e1fbe48a2`、simulation v.1.0 `e48feba61073c02bec666f052e55f1fc547257cf`；生产核心source-lock验证通过，不修改或永久复制消费者源码。

- ROS Humble构建成功，启用`FOLLOW_MPPI_REFERENCE_DIR=<交接包>/reference_code`。
- 包级9个gtest程序共126个独立用例最终结果全通过，另有外部source-lock检查通过。首轮完整运行后修正3个失败，仅复跑受影响core/feedback/replay组51项，全通过；其他组保留首轮通过证据，不谎称全部在最后一次命令重跑。
- 4个真实消费端集成用例通过：生产发布器DDS消息进入附件未修改decodeExecutionState/stoppedManeuverFeedback，验证有效解析、同快照、模块重排、错误几何拒绝、两帧静止成立、实际转舵/重复/过期拒绝、失效/重激活及可靠QoS。
- 5个逐步回放用例通过：每步从同一内部快照恢复原生产核心并使用同一反馈/指令/dt，比较参考速度/加速度、最终目标、参考舵角/舵速/舵加速度、轮速、分支、Holding、门控；含授权旋转、稳态与加速中撤权、接平移、负vx/vy、限位、一般重定向和可变dt。数值容差1e-5，状态精确一致；这是受测场景的单步恢复回放，不是任意场景数学等价证明。
- 用户裁决后的1ms撤权样本：前ω=1.036578660757236、α=1.5；驱动和参考均输出ω=1.036971359838935、α=0.3922264003120865，后续连续减速收敛验证通过。
- 首轮失败分别为：混合运动被错误地按“纯旋转启动平移”抑制（已修正条件，无放宽断言）；观测0的浮点残差约7e-15（零断言改1e-9）；回放同一抑制错误（同根因修复）。
- 质量门禁通过185 passed/11 skipped；shellcheck本地缺失由CI检查。接口门禁及72工具测试、8接口包构建通过；最终采样来源契约针对7项通过。

未验证：实际导航配置与现场模块映射对齐、目标机硬件新鲜度/舵速估计误差和实时抖动、联合仿真2→3→4→2、实车方向及停止距离。无Nav2联合环境或硬件操作，本次不把离线real配置/软件harness称为实车或整机验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 现有FollowMPPI v1权威兼容布局为27+4×16，不是31+4×15；无效观测不补假0。
- F2: 撤权采用用户批准的参考受控整形，允许短暂继续加速；保护优先级和接平移门禁保留。
- F3: 消费端解析/停稳与受测逐步回放已离线验证；实际映射和目标环境验收是独立发布门禁。

## 遗留

接口纠正PR27尚待评审合并；driver deps.repos/source-lock仍保留原已发布pin，不锁未合并SHA。必须先发布正确接口再同批更新pin并验证消费者配置。不得仅因本次软件测试通过合并部署驱动PR336。既有轮组映射PR326与1124 CAD模型迁移另行协调，不擅自取代现场标定。
