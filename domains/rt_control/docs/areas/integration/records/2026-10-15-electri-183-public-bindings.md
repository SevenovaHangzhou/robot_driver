---
id: integration-20261015-01
area: integration
title: 公共参数绑定消除底盘、头部与力传感器跨层耦合
date: 2026-10-15
type: fix
trigger: ELECTRI-183；用户授权修复三项架构问题并创建面向 main 的 PR
commits: [bugfix/electri-183-layer-bindings]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [integration-20261014-02, integration-20261014-03, integration-20261014-04]
---

## 背景

当前组合路径保留三处耦合：底盘重复维护硬件事实，头部协议校验固定资源名，
力传感器组合读取协议私有配置。本次统一通过公共结果和显式绑定消除这些耦合。

用语：公共导出指 renderer 的 `parameter_exports`；参数绑定指 scope 的
`controller_parameter_bindings`；命名上下文指 `controller_context`。

## 改动

- 集成层按模块与导出名绑定控制器参数。映射按显式 `instance_order` 排序。
  缺项、实例集合不一致、资源不存在和重复事实在写文件前拒绝。
- 底盘协议从轴配置导出关节名，从单机标定导出轮径与安装坐标。
  `module_x_m/module_y_m` 的单位为米，坐标系为控制器底盘坐标系；实测值仍为 `TBD`。
  CANopen 节点增加显式 `module_position`，不再从编码器名称推断安装位置。
  控制器配置删除对应六组硬件事实，控制策略与 `calibration_verified:false` 保留。
- 头部配置声明 `control_name`。协议只校验资源唯一性和接口结构。
  关节名与 GPIO 名同步绑定到控制器，双电机与单 GPIO 的数量约束保持。
- 力传感器导出 `binding/sensor_name/family`，组合器不再读取其私有 `spec`。
- 公共 ROS 接口、Robot Model、真实零位与运行频率未修改。
  新 schema、协议 owner 和机型数据必须同版本发布。

## 验证

- 五包隔离构建：`robot_description`、`rt_control_machine_config`、
  `rt_control_composer`、`rt_control_semantic_components`、`robot_hw_can`，PASS。
- `pytest` 执行 composer 的 `test_compose.py`、`test_chassis.py`、
  `test_head_force.py`、`test_boundary_bindings.py`：首轮 33 passed / 5 failed。
  失败为新命名上下文遗漏模板字段。补齐后相关 9 项通过。
  更新后的 8 项边界用例独立通过。十份冻结输出保持一致，未更新金样。
- `colcon test --packages-select robot_hw_can --ctest-args -R test_damiao_system`
  与 `colcon test-result --verbose`：6 tests、0 errors、0 failures。
  覆盖改名导出、资源冲突、接口约束与使能前的拒绝行为，不连接真实总线。
- 构建警告来自既有测试 fixture 的 `InterfaceInfo` 部分初始化。
  Xacro 的 Python 兼容警告保留。
- `tools/quality_gate.sh`：PASS；245 passed、13 skipped，架构门禁与固定样式工具通过。
  本地缺少 ShellCheck，CI 继续强制执行；本次未修改 shell。
- `git diff --check` 与 `git diff --cached --check`：PASS。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 底盘关节、编码器、安装坐标和有效轮径通过公共导出绑定，控制器策略不重复保存这些事实。
- F2: 头部新组合路径使用配置中的资源名，控制器参数与硬件声明同步。
- F3: 力传感器私有配置结构可以变化，公共命名上下文保持时无需修改组合器。
- F4: 已知缺项保持拒绝；Mock 不自动替代标定或准入。

## 遗留

完整运行、镜像、实机及部署未验证。底盘其它策略缺项和全机组合准入仍沿用现有边界。
本次按用户要求仅执行相关测试和强制门禁，不重复整仓构建或无关测试。
