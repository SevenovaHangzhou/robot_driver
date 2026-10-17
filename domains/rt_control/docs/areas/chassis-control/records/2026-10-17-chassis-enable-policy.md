---
id: chassis-control-20261017-02
area: chassis-control
title: 底盘使能策略按模块与模式组生成并确认启动模式
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 决定使能分批按模块+模式组、先做底盘 scope
commits: [feature/electri-127-enable-policy]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261017-01]
---

## 背景

研华部署的使能策略是手写文件，关节名、分批和 `expected_modes` 与组合重复维护。
本次让 composer 从模块与模式组生成这些参数，并把现场验证过的模式双确认合入主线 enable_manager。

## 改动

- 使能策略新增 `enable_groups`（module、mode_group、batch_size）与 `mode_confirmation`；
  原 `managed_joints/enable_batches` 写法仍可用（两者其一）。
- composer：`enable_groups` 展开为批次与关节，`managed_joints` 取硬件顺序；
  `mode_confirmation: startup` 时按模式组的 `mode_of_operation` 生成 `expected_modes`。
  scope 可用 `enable_policy` 选择策略引用；新角色 `motion` 的控制器由 enable_manager 使能后激活。
- 新增 `Alfa_V3_chassis_enable_policy.yaml`：转向 CSP 每批 2 轴，然后行走 CSV 每批 2 轴；时序沿用现场值。
- chassis_only scope：active 为 joint_state_broadcaster 与 enable_manager，swerve_controller 为 inactive 运动控制器。
- enable_manager：合入现场 `expected_modes`（每批等待运行使能且 6061 等于预期模式；模式 0 视为待确认，
  其它值立即 `mode_mismatch`，超时 `mode_not_confirmed`），运动控制器只激活一次。

生成结果与研华部署的使能参数一致（managed 顺序、批次、`expected_modes`、`jtc_name`）。

## 验证

- `colcon test --packages-select enable_manager`：81 tests，0 failures。
- composer 测试（源码 PYTHONPATH）：42 passed，含新增底盘使能参数推导用例。

离线测试，不连接总线，不授权使能或运动。

## 遗留

- 多个运动控制器同时激活留待全机 scope；数据源就绪前置条件与运行故障收尾见后续变更。
