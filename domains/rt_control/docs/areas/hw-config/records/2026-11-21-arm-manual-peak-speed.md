---
id: hw-config-20261121-01
area: hw-config
title: 双臂速度限值同步 eRob 手册输出端峰值
date: 2026-11-21
type: fix
trigger: 用户 2026-11-21 要求按电机手册峰值转速修改并上传；SEV-134 / robot_driver#298
commits: [codex/arm-manual-peak-speed]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261120-01, SEV-134]
---

## 背景

驱动冻结的 robot_description@98d498eb 仍将双臂速度限制为 30°/s。模型主线 eca5b04
已按 eRob V3.44 表 2-2～2-4、12-1 改用输出端峰值转速，但后续模型还包含几何、Home
及悬挂变更。本次在权威模型仓库旧基线上回移速度修正，驱动按完整 SHA 导入同一快照。

## 改动

模型来源：robot_description@7cb6f9bb2897f057cd1018f43feee082e5d3d705，
分支 codex/arm-peak-speed-backport，source_tree a3a5b7f127a2b947c7a991a9884392b398ecae83。
模型主线已有相同速度修正，此回移只用于冻结消费者，不重复合入模型 main。

| 双臂关节 | 输出端峰值 rpm | 旧速度 °/s | 新速度 °/s |
| --- | --- | --- | --- |
| J1–J3 | 12.5 | 30 | 75 |
| J4 | 16.7 | 30 | 100.2 |
| J5–J7 | 30 | 30 | 180 |

换算为 rpm × 2π / 60 rad/s，手册数值已在输出端，不再应用减速比。
JTC 和 Rolling 继续由 composer 从 URDF 注入 joint_velocity_limits。
RT 加速度策略仍为 373.06 / 498.41 / 895.35°/s²；不改变位置、力矩、几何、Home 或域间接口。
重算受影响的组合器 golden，增加生产限值贯通检查；保留原轴序扰动和缺项拒绝测试。
重新生成的 golden 同时反映 composer 已有的逐轴加速度及急停参数；这些运行策略未在本次修改。

## 验证

全部为 T0 离线验证，未启动控制栈或执行硬件操作。

- 权威模型独立 pytest：13 passed、2 skipped；跳过项为缺少 trimesh / yourdfpy 的可选几何检查。
- 三份模型 URDF 除双臂 velocity 外的 XML 语义一致，导入文件逐字节匹配上游提交。
- `validate_machine_profile.py --manifest src/rt_control/hwcfg_machines/machines/Alfa_V3/manifest.yaml --unit Alfa_V3-bench-01 --all`：13 组配置通过。
- `colcon build`：robot_description、hwcfg_machines、integ_composer、integ_bringup 四包通过。
- `colcon test` 和 `colcon test-result --verbose`：295 tests，0 errors，0 failures，2 skipped。
- 组合器和启动包完整源码 pytest：290 passed；包含生产左臂、右臂、双臂速度与加速度贯通检查。
- `tools/quality_gate.sh`（含 arch_check）：通过；使用本机临时依赖路径提供 clang-format、ruff 和 Python 校验依赖，ShellCheck 未安装，未宣称该项通过。

构建和包测试使用 ROS Humble 环境，构建、安装及日志位于临时目录；源码 pytest 的 ament 索引映射到当前源码。
现有测试跳过不作为实机验收证据；本次没有新 PR / CI 运行结果。

## 结论与冻结事实

- F1：URDF 双臂速度上限由统一 30°/s 改为逐轴手册输出端峰值；数据来源为权威模型。
- F2：RT 逐轴加速度策略保持不变，不读取模型旧的统一 60°/s² 作为双臂执行限值。

## 迁移、回退与遗留

消费者是 RT 的 FJT/Rolling、模型规划消费者以及读取同一模型的仿真。
发布顺序：先取得已推送的模型提交，再构建引用该提交的驱动，重新生成 artifact；规划端核对
同一限值口径后再由操作台完成分级验收。旧 artifact 的 controller 参数不会随源码自动变化。
回退时恢复 source-lock 与整个模型快照，并重新生成上一版 artifact；不能混用新旧控制器参数。
本次只提交推送；模型版本联合验收、T2–T4、负载下速度/制动包络及急停遗留 #299 均未完成。
#297 的加速度口径协调及 #298 的 policy/schema 加载链遗留继续追踪，不因本次速度修正关闭。

本记录不授权使能或运动。
