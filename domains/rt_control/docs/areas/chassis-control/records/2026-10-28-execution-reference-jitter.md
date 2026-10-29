---
id: chassis-control-20261028-01
area: chassis-control
title: 连续模式分支保持与只读执行参考状态
date: 2026-10-28
type: fix
trigger: 用户要求先在本机仿真修复导航抖动，必要时同步实机驱动源码
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /home/robot/nav/wheel_artifacts/reports/jitter-fix-20261028/build-driver.log
  - /home/robot/nav/wheel_artifacts/reports/jitter-fix-20261028/test-driver.log
supersedes: []
related: [chassis-control-20261024-01]
---

## 背景

导航抖动录包包含预测拒绝置零，以及连续模式纯平移强制统一分支造成的额外转舵。实际测量导数不能代替驱动内部 S 曲线状态。

## 改动

swerve_driver 连续模式（90°门控）纯平移使用原逐轮可行分支选择，保留迟滞与必要门控；对齐模式不改。ControlCore 提供固定数组只读快照，控制插件预分配、trylock发布100Hz上限的 `/rt_control/chassis/execution_state` 标准 Float64MultiArray。v1包含91个SI单位字段，实际反馈与内部参考分开，附模式、能力、序号、时间；导航侧配套验证。不增加任何设备写接口，不改变机械标定/外参/使能逻辑。该状态是实验性只读候选接口，不代表公共契约冻结。

## 验证

本机单包串行 Release 构建和无设备 gtest；日志见 evidence。导航另验证快照恢复与下一步核心输出一致。实机运行、网络时延与真实IMU复测未执行；本地目录是部署副本而非 Git checkout，无提交 SHA。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 连续模式取消纯平移强制统一分支；必要换支仍受机械可行性和90°门控约束。
- F2: 仿真和导航内嵌核心按本源码生成并更新散列锁，不依赖运行时读取本目录。

## 遗留

公共接口正式发布及实机联调待完成。新导航要求本参考话题，必须成套升级；未把本机改动部署至公共机。

## 2026-10-29 发布与部署补充

用户授权发布到 feature/electri-127-chassis-mode-runtime 并增量部署公共机。本次仅合入本记录所列舵轮补丁，不覆盖公共机其他人的 IMU、电池、日志站、网页及部署配置改动。现场驱动继续默认 align_first，轮式里程计保持 /wheel/odom；分支源码默认值仍由安装配置决定。部署前完整备份导航和驱动的源码、build/install 与配置；增量编译仅选择 swerve_driver，单任务串行。导航消费者为 v3.0.3_test。

发布检查：tools/quality_gate.sh 通过（STYLE_BASE_REF 指向本功能分支远端基线；临时 Python 依赖环境提供 jsonschema 3.2.0）；本机原先无设备 10 个 gtest 组通过。现场重启仅验证状态与接口，未使能、未发送运动，不能称为实机导航验收通过。
