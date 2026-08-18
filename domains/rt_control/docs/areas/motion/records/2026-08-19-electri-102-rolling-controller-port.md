---
id: motion-20260819-02
area: motion
title: ELECTRI-102 rolling 控制器移植到公共接口契约
date: 2026-08-19
type: feat
trigger: ELECTRI-102 / D-01 / D-02 / D-03 / D-18 / D-19 / D-20
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rolling_trajectory_controller, tools/tests/test_electri_102_rolling_package_contract.py]
supersedes: []
related: [ELECTRI-102, MOTION-124]
---

## 背景

ELECTRI-102 原型位于旧 `robot` 工作树，使用占位接口、局部 QoS 和旧控制器命名，不能直接
成为当前 `robot_driver@main` 的跨域实现。本次只移植 rolling controller 本体，不提前把
尚未完成的 provisional 包络、模式服务或 bringup 注册伪装为可用。

## 改动

- 移植固定 14 轴 rolling buffer、Hermite 插值、动态包络校验、停车包络、三槽 snapshot、
  session 状态机、controller plugin 及原型 10 组测试。
- Motion command 改用 `robot_motion_interfaces/RollingJointTargetBatch`；open/close/state 改用
  `robot_rt_control_interfaces`；所有外部 endpoint 使用 `robot_interfaces_qos` 的
  `rolling_command()`／`rolling_state()` 命名 profile。
- 适配 public wrapper 枚举、UUID、`ErrorInfo`、显式 REQUEST_STOP/FINALIZE 和 open 生效参数
  回报；close 幂等比较包含 operation，禁止同 request ID 换操作。
- 把原型 `dual_arm_jtc` 测试名对齐为当前 `whole_body_jtc`。
- 对 21 个 `RollingServiceResult` 到 DREE 的映射逐项加测试；未知值 fail-closed 为
  `INTERNAL_ERROR`。
- RT allocation trap 改为只统计调用 `update()` 的线程，排除 ROS/DDS 后台分配造成的
  套件级假阳性，同时保留所有 `new`/aligned/nothrow 变体的钩子。

## 验证

- public-interface overlay 下 package build：1/1 通过。
- rolling CTest：11/11 通过，包括原型 10 组以及新增 DREE 映射组。
- `test_electri_102_rolling_package_contract.py`：3/3 通过。
- pluginlib 可按导出类名发现控制器；STRICT switch 测试证明 JTC 与 rolling 不会同时 claim
  14 个 position command interface。
- `UpdatePathDoesNotAllocate` 在完整套件中零命中。

以上均为本机 mock/单元验证，没有启动真实 bringup、总线、驱动使能或运动。

## 结论与冻结事实

- F1: rolling command 的公共 owner 是 Motion，open/close/state 的 owner 是 RT-Control。
- F2: controller 核心在当前 main 基线和 public feature-interface overlay 上可构建、可加载。
- F3: public service 错误双层语义按 E102-D31 固定，Motion 不解析字符串。
- F4: 这一提交是移植 checkpoint，不等于已经满足“可供 Motion 联调”的 E102-D28 门槛。

## 遗留

Prime anchor、prefix 不可变、历史裁剪、horizon 上限、验证/复制/采样优化、YAML provisional
包络、模式服务、bringup 集成和 10 分钟 fake 长稳由后续原子提交完成。
