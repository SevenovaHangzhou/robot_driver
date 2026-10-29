---
id: contract-20260814-01
area: contract
title: 公共接口改为固定 SHA vendoring 并采用命名 QoS
date: 2026-08-14
type: decision
trigger: BQ-136 / ELECTRI-77 后续
commits: [feature/robot-interfaces-vendoring]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [quality_gate 187/187, changed ROS packages 73/73, Domain 142 mock 6/6, Docker image sha256:ffd0eba25ccd431d5a6da2b23d334bc6748e7021681630e536998035ade83b54]
supersedes: [contract-20260813-01]
related: [BQ-135, BQ-136, BQ-137, TC-ST-04, TC-IF-01]
---

## 背景

BQ-135 已裁决 `/joint_states` 采用实测 125 Hz，但本仓库仍复制公共 IDL 并维护第二份
RT-Control 契约视图。BQ-136 进一步裁决改为完整仓库 vendoring，并明确采用上游命名
QoS 包。

## 改动

`robot_interfaces` 权威契约由 0.6.0 升到 0.6.1，R-OUT-03 固定为 125 Hz；本仓库
`deps.repos` 与 `src/interfaces/source-lock.yaml` 同时固定上游提交
`eb010e1e33b31ae8ae4ebe0843a2d5c5ca2fabd1`，构建时导入
`src/vendor/robot_interfaces`。删除树内 `robot_rt_control_interfaces`、
`robot_system_interfaces`，仅保留域内私有 `rt_control_interfaces`。

自有跨域 Topic 分别接入 `control()`、`fast_state()`、`state()`、`latched()`、
`diagnostic()`；冻结的 `ros2_controllers` 通过窄补丁接入控制与快速状态 profile，
`/tf_static` 以 mock 运行时等价性断言覆盖。

完整 vendoring 同时暴露了上游 `main` 已合入、旧本地镜像未同步的
`ErrorInfo`/`DomainReadiness` 破坏性字段变化。适配器现按同一 vendor SHA 的生成类型填充，
`ErrorInfo.code` 暂以 DREE 十进制字符串兼容；其跨域语义冲突登记为 BQ-137，不在本记录内
自行裁决。

## 验证

RED 阶段的 vendoring/source-lock/QoS 合同测试先按预期失败；当前实现完成后：

- `tools/quality_gate.sh`：PASS，187 个工具测试全绿，repository gate 覆盖率 85%。
- 隔离 vendor 工作区中，10 个直接受影响包构建通过；其中 4 个自有运行包以
  `BUILD_TESTING=ON` 重建并执行 `colcon test-result --verbose`：73 tests，0 errors，
  0 failures，0 skipped。
- `ROS_DOMAIN_ID=142`、`use_mock_hardware:=true` 的契约套件：6 tests 全绿；运行时验证
  5 个命名 QoS profile、125 Hz `/joint_states`、endpoint 图和删除清单。未访问真实总线。
- `python3 tools/release_test_runner.py validate`：`OK: 33 cases valid`。
- 直接执行无代理 `docker build`：28 个包完整闭包构建通过，测试镜像
  `sha256:ffd0eba25ccd431d5a6da2b23d334bc6748e7021681630e536998035ade83b54`。
  仅有既有 Lely 构建警告与 CANopen DCF `DynamicChannels`/`[607E]` 警告。

主机全闭包构建仍受本机缺少 `/usr/local/etherlab` 开发文件限制；等价闭包已由 Docker
干净环境构建覆盖。目标机与跨域联合运行未执行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `/joint_states` 的权威 RT-Control 契约视图来自固定 SHA 的
  `src/vendor/robot_interfaces/contract/views/rt_control.md`，频率为 125 Hz；本事实取代
  `contract-20260813-01#F1` 的本地文档来源指针，125 Hz 内容继续有效。
- F2: `src/interfaces` 只允许 RT-Control 域内私有接口；公共 schema 必须从固定 SHA 的
  `robot_interfaces` vendor 构建。
- F3: RT-Control 跨域 Topic 使用 `robot_interfaces_qos` 命名 profile，不逐端点私设策略。
- F4: `/cmd_vel_safe` 的 `control()` 必须由 Motion 发布端与 RT-Control 订阅端原子采用，
  禁止新旧 QoS 混合部署。

## 遗留

当前 pin 是上游 PR 分支提交；下游合并前必须更新为 `robot_interfaces` PR 的最终 merge SHA。
BQ-137 的共享错误/readiness schema 语义尚待接口所有者裁决；在此之前本分支只允许构建、
mock、草稿 PR 和评审，不得合并或发布。Motion 端 QoS 同批升级和目标机验证均未执行；
本地测试镜像已构建，但不是批准的 release 镜像。
