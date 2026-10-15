---
id: integration-20261015-02
area: integration
title: 控制器参数声明化、部署时生成运行配置与 EtherCAT 设备族注册表
date: 2026-10-15
type: feature
trigger: ELECTRI-184；用户批准架构审查第 5 项与三项低优先级修改，并选择“部署时离线生成，启动只校验”
commits: [feature/electri-184-declarative-composer]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [integration-20261015-01, integration-20261014-04]
---

## 背景

架构审查发现四处问题：composer 按控制器 `role` 分支生成参数；每次启动在
`/tmp` 生成运行配置且不清理；EtherCAT renderer 内有主站超时常量；EtherCAT
owner 用 `family ==` 分派。用户批准全部修改。

用语：生成物指 `compose_artifact` 写出的目录；参数来源指 `controllers.yaml`
中每个控制器的 `parameters` 列表。

## 改动

- 参数来源：composer 删除全部 `role` 分支。控制器参数由 7 类通用来源声明：
  `yaml`、`policy`、`controllers`、`owner_parameters`、`topics`、`runtime`、
  `policy_file`。原有一致性检查改为 `match`/`partition` 选项。
- 机型数据：控制器增加 `tags` 与 `parameters`，删除 `role`。scope 显式声明
  `policies`、`jtc_only_excluded` 与 `controller_file`（替换
  `preserve_controller_file`）。使能策略视图增加 `enable_only`。
- 生成物：`compose()` 改为纯函数，返回 `Composed.files`。新增
  `compose_artifact` 命令，暂存后改名替换，并写入 `artifact.yaml`（选择项与
  每文件 SHA-256）。拒绝覆盖非生成物目录。
- 启动：四个运行入口删除 `mkdtemp`，改读 `generated_dir`（默认取
  `RT_CONTROL_GENERATED_DIR`）。`load_artifact()` 拒绝缺失、选择项不同、
  文件被改、多文件/少文件与过期生成物。过期判定方法为：内存中重新组合，
  逐文件比较。每种拒绝都给出重新生成命令。启动不写组合配置。
- 每次启动的值：力传感器 `startup_id` 不进入生成物。入口用节点限定的
  `--ros-args -p <控制器>:startup_id:=<值>` 注入；缺值拒绝启动。
- EtherCAT：`ethercat_master.yaml` 保存 `startup_bus_timeout_ms: 70000` 与
  `preload_timeout_ms: 5000`，数值不变。`renderer_families.yaml` 按 spec
  `family` 选择 renderer 与私有 schema，按拓扑 `kind` 选择预检。机臂渲染移入
  `arm_renderer.py`，CiA402 拓扑预检移入 `topology_preflight.py`。设备族表增加
  `position_counts_field`。机臂模块数据增加 `family: zeroerr`。

公共 ROS 接口、Robot Model、标定值与控制周期未修改。

## 验证

- `colcon build`（composer、machine_config、robot_description，隔离 install）：PASS。
- `pytest` composer 全部测试、bringup 入口测试、EtherCAT 两项 profile 测试：
  218 passed。格式化后复跑 composer 与 bringup：211 passed。
  十份冻结输出逐文件规范化一致，金样未修改。
- 新增 `test_artifact_and_sources.py` 21 项（含入口测试）：生成与读取往返，
  读取阶段禁止写盘，四类损坏、过期与选择项不同的拒绝，节点限定运行参数，
  通用来源的拒绝路径，以数据注册新设备族，以及主站超时取自 profile。
- 首轮发现 2 项既有测试需随接口调整：`Composed` 新增两个字段，以及设备族
  校验需要注册的族名。测试意图保持不变。
- `ros2 run rt_control_composer compose_artifact --help` 与缺少 `--mock/--real`
  的拒绝：PASS。`tools/check_v3_branch_contract.py`：PASS。
- `tools/quality_gate.sh`：PASS，246 passed、13 skipped；ShellCheck 本地缺失，由 CI 执行。
- Humble 节点限定参数覆盖 `-p node:param:=value` 已用 rclpy 实测：只作用于指定节点。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: composer 不含控制器类型分支；新增控制器类型只改数据。
- F2: 启动入口只读取并校验生成物；组合配置只由 `compose_artifact` 写出。
- F3: 过期、被改、选择项不同或缺失的生成物在预检和创建 Node 之前拒绝。
- F4: EtherCAT 设备族与预检由注册表数据选择；主站超时取自协议 profile，数值不变。

## 遗留

- 部署流程新增一步：部署或改配置后必须运行 `compose_artifact`；Mock 开发同样需要。
- 目标机部署、镜像、真实 Node 启动与实机未验证。
- 力传感器 renderer 仍用 `hardware_system_id == "bluepoint_raw_bench"` 判断台架，未在本次范围。
