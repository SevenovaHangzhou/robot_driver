---
id: chassis-control-20261017-06
area: chassis-control
title: scope 运行时发布能力描述，操作界面不再匹配硬件组件名
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 决定 Web 能力识别与运行入口一起做
commits: [feature/electri-127-capabilities]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261017-05]
---

## 背景

研华 Web 后端靠硬件组件名 `ecat_chassis`、`canopen_steering_encoders` 和 swerve 的
`command_topic` 参数判断后端。新架构的组件名不同，swerve 也没有该参数。

## 改动

- 控制器绑定可声明 `capabilities`（name/topic/type）；swerve_chassis 声明 `chassis_velocity` → `/cmd_vel`。
- composer 在 `lifecycle.yaml` 写入能力（含 enable_manager 对应的 `/control/set_enabled`）与硬件系统列表。
- scope 运行时在生命周期控制器加载成功后启动 `scope_capabilities`，以 `std_msgs/String`（JSON，
  可靠、transient_local）发布 `/rt/capabilities`：schema_version、control_scope、use_mock_hardware、
  capabilities、hardware。
- Web 侧修改另行交付（不在本仓库）：订阅该话题，按声明的硬件列表与能力识别后端，缺失时回退旧判断。

## 验证

- `pytest rt_control_bringup/test/test_scope_lifecycle.py`：6 passed。
- composer 测试：43 passed（底盘能力推导用例）；quality_gate PASS。

## 遗留

- Web 后端补丁待部署授权后在研华更新。
