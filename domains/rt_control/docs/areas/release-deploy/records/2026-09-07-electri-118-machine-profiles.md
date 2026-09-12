---
id: release-deploy-20260907-01
area: release-deploy
title: ELECTRI-118 三代机模块化物理 Profile 与控制范围校验骨架
date: 2026-09-07
type: feature
trigger: ELECTRI-118
commits: [kozia/electri-118-module-profiles]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: ["43 manifest/launch/validator tests", "validate_machine_profile.py --all"]
supersedes: []
related: [ELECTRI-94, ELECTRI-117]
---

## 背景

ELECTRI-118 将 ELECTRI-94 的硬件变体分层扩展到三代机。三代机的双臂、Updown、四舵轮、
外置舵角编码器和头部云台将在不同到货阶段分模块测试；机械臂、Updown 和四舵轮最终共用
一个 EtherCAT master，因此物理配置和本次控制范围必须分开表达。

## 改动

- 在 `rt_control_bringup/config/machines/alfa_v3.yaml` 增加 machine manifest，登记 5 个
  physical profile：`arms_only`、`arms_updown`、`chassis_only`、`full_robot`、`head_only`。
- 登记 4 个功能模块（双臂、Updown、四舵轮、头部云台）和 1 个独立 state-only 反馈资源：
  双臂 16 台（14 CSP + 2 PP）、Updown 1 台 CSP、四舵轮暂定 8 台步科 EtherCAT 执行器、
  4 个一一对应 FL/FR/RL/RR 的 CANopen 外置舵角编码器、2 台达妙 CAN 头部云台。步科
  PDO/SDO、编码器 EDS、达妙 CAN ID 和机械参数保持 TBD。
- 新增 `machine_profile.py`，严格校验模块依赖、单 EtherCAT master、物理布局、scope
  矩阵、执行器 mode 分组和 state-only 编码器；静态选择不把 TBD 转换成默认值。
- 新增 `rt_control_module.launch.py` 和 `validate_machine_profile.py`。当前只允许无硬件
  静态校验；任何 draft manifest 的真实运行请求在创建 ROS 节点前 fail closed。
- 将 manifest 校验目标和 43 个测试接入 `rt_control_bringup` 构建/测试；原有
  `rt_control.launch.py`、`alfa_v1` 配置和生产默认行为未改变。

## 验证

已执行：

- `python3 -m pytest -q src/rt_control/rt_control_bringup/test/test_machine_profile*.py`：43/43 PASS。
- `python3 src/rt_control/rt_control_bringup/scripts/validate_machine_profile.py --manifest src/rt_control/rt_control_bringup/config/machines/alfa_v3.yaml --all`：PASS，5 modules、5 physical profiles、10 profile/scope selections。
- `python3 -m py_compile`：新增 Python loader、launch、validator 和测试文件 PASS。
- 临时 `BUILD_TESTING=ON` CMake/CTest：新增 3 个 ament 测试目标 3/3 PASS；其余既有
  cross-domain Mock 目标因本机未安装 `robot_rt_control_interfaces`，未能执行。
- 临时安装空间直接执行 validator 与 `rt_control_module.launch.py`：PASS，未创建硬件节点。
- `git diff --check`：PASS。

未执行：ROS 完整 workspace/Docker 构建、真实 EtherCAT/CANopen 访问、配置读回、reset、
enable、FJT、底盘命令、头部云台命令和任何实机运动。当前工作站缺少已安装的下游包闭包，
无法仅选择 `rt_control_bringup` 完成 colcon 构建；这不影响上述纯 Python 静态证据。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: machine manifest 只拥有 4 个功能模块及其 state-only 反馈资源的组合、physical profile、
  control scope 和 owner 引用；
  PDO/SDO、机械参数、Robot Model 和安全数值仍由各自 owner 管理。
- F2: 三代机双臂固定登记为 16 个执行器，其中 14 个 CSP mode 8、2 个 PP mode 1；
  `arms_only` 的 JTC 视图只包含 14 个 CSP 组，PP 夹爪不进入流式 JTC。
- F3: 四个 CANopen 外置编码器分别绑定 FL/FR/RL/RR 舵轮并作为舵角 state-only 权威源；
  它们不进入 EtherCAT enable_manager。
- F4: 机械臂、Updown 和四舵轮共享 EtherCAT master 0；任何选中的 EtherCAT 模块组合只
  允许一个 master，scope 不能在线热切换。
- F5: `arms_only`、`arms_updown`、`chassis_only`、`full`、`head_only` 是控制范围；
  在 `full_robot + arms_only` 中只允许双臂资源被 claim/enable，Updown 和底盘保持非活动。

## 遗留

- 三代机所有 profile 当前为 draft；取得环位、设备 identity、PDO/SDO、EDS、CAN ID、
  零位、比例、限位和机械参数后，才可提升为 runtime-ready。
- 还需要将选中的 manifest 接入真实的单 EtherCAT system、scope-specific controllers、
  diagnostics/readiness 和有序停机脚本；本记录只交付安全的静态选择骨架。
- ELECTRI-117 负责步科 EtherCAT 四舵轮和 CANopen 外置编码器后端；后续不得把大妙
  SocketCAN/MIT 执行层直接当作生产协议。
