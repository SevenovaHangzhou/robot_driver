---
id: release-deploy-20260913-02
area: release-deploy
title: ELECTRI-117 四舵轮控制器接入三代机模块选择
date: 2026-09-13
type: feature
trigger: 用户要求把四舵轮方案嵌入 main 的三代机模块化底盘配置
commits: [work/electri-117-swerve-module-binding]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-117, ELECTRI-118, BQ-144, BQ-145]
---

## 背景

main 已有 `swerve_driver/SwerveController` 和四舵轮模块资源计数，但 machine manifest
未说明哪个控制器消费 4 路 CSP、4 路 CSV 以及四个必需的外置舵角状态。
独立 `dm_swerve_driver` 节点拥有自己的 IgH master，不可与双臂/Updown
共用主站，因此本次只按现有 ros2_control 架构接入控制计划。

## 改动

- `alfa_v3.yaml` 增加 `swerve_chassis` 的控制器绑定，指向
  `swerve_driver/config/controller.draft.yaml`，列出两类执行器分组与
  `swerve_encoders` state-only 依赖。
- manifest 解析器校验绑定的模块、插件包、分组全集、必需状态和 scope 资源占用。
  `chassis_only` / `full` 返回该控制器，`full_robot + arms_only`
  不返回；v1 的其他机型仍可不声明此字段。
- 模块 launch 的无硬件摘要报告选中控制器；draft 配置单独列为 runtime blocker。
  既有生产启动、双臂/Updown 和头部 CAN 归属均未修改。

## 验证

- `python3 -m pytest -q src/rt_control/rt_control_bringup/test/test_machine_profile.py src/rt_control/rt_control_bringup/test/test_machine_profile_launch.py src/rt_control/rt_control_bringup/test/test_machine_profile_validator.py`：58 passed。
- 更改的 Python 与 launch 文件经 `python3 -m py_compile` 校验通过。
- `colcon build` 在隔离的 `/tmp/rt-control-swerve-*` 路径完成
  `rt_control_bringup` 单包安装；安装后的 validator `--all`
  检查 5 模块、5 profile、10 组合通过；
  `chassis_only --require-runtime-ready` 按预期拒绝并列出草案配置和未确认硬件事实。
- 隔离的 `BUILD_TESTING=ON` 包构建通过；`colcon test --packages-select
  rt_control_bringup --ctest-args -R test_machine_profile` 的
  `colcon test-result --verbose` 汇总 61 tests、0 errors、0 failures、0 skipped。
- `tools/quality_gate.sh`：277 passed、13 skipped，策略覆盖 83%。
  当前本机缺少 ShellCheck，CI 仍需执行该检查。

未执行容器镜像构建/容器内启动、真实 EtherCAT/CANopen 访问、使能或运动；
此变更尚不能作为 main 生产镜像或实机接入验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 底盘控制器是共享 ros2_control manager 中的
  `swerve_driver/SwerveController`，不另启独立 IgH master 或 CAN 电机后端。
- F2: 四台 Kinco 转向 CSP 和四台驱动 CSV 只在选中底盘 scope 时纳入控制计划；
  四个 CANopen 外置编码器是同时选中的必需 state-only 资源，不进入电机使能列表。
- F3: 静态绑定不改变 draft/runtime 门禁；配置文件仍为
  `calibration_verified: false`，真实运行继续拒绝。

## 遗留

BQ-145 的 Kinco EtherCAT 从站 profile、同一主站环位与全局拓扑、
CANopen 编码器状态提供器/反馈年龄、Robot Model 关节名、齿比零位、
机械限位、残差剔除策略和 scope-specific 生命周期编排尚未完成。
旧独立包的最小二乘打滑残差逻辑不在当前 `swerve_driver` 中，需单独迁移和验证。
补齐后应做完整镜像、Mock/隔离启动与经授权的实机验收；本轮不填猜测值。
