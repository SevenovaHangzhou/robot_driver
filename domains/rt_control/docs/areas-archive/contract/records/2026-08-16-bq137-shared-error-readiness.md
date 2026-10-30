---
id: contract-20260816-01
area: contract
title: BQ-137 跨域错误与 readiness 语义裁决落地
date: 2026-08-16
type: decision
trigger: BQ-137 / 用户 2026-08-16 裁决
commits: [feature/robot-interfaces-vendoring]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [upstream robot_interfaces PR 6 head d8236bda7e087a54a8ee7585bc7a2d6a94251af4, quality_gate 187/187, affected ROS packages 22/22, Domain 142 in-process readiness smoke, Docker image sha256:59a476fbb5023a0b36aba47db8323c2b4c73a951cd7a05e42295abeaed5c82f1]
supersedes: []
related: [BQ-133, BQ-136, BQ-137, contract-20260814-01, TC-IF-01]
---

## 背景

完整 vendoring 暴露出共享 `ErrorInfo` 同时承担跨域 DREE 和导航内部字符串两套互斥语义，
`DomainReadiness` 的新共享字段集也缺少一致性规则。用户于 2026-08-16 完成 BQ-137 裁决，
要求把该决定落到上游 schema、下游生成类型和 RT-Control 生产代码，而不是只保留文档结论。

## 改动

上游 `robot_interfaces` PR #6 将契约版本 0.6.1 → 0.7.0，把
`ErrorInfo.code` 从 `string` 改为 `uint32` DREE，声明导航内部诊断字符串必须留在域内类型，
并冻结 `DomainReadiness` 的 `ready/status/blockers/errors/map_version/producer_instance_id`
一致性规则。上游 PR head 是
`d8236bda7e087a54a8ee7585bc7a2d6a94251af4`。

本仓库的 `deps.repos` 与 `src/interfaces/source-lock.yaml` 同步锁定该 SHA；
`control_api_adapter` 删除 DREE 十进制字符串兼容层，向生成的 `ErrorInfo.code` 直接写入
`uint32`。RT-Control readiness 保持 `HEALTHY`/`UNAVAILABLE` 两种实际输出：健康时
`ready=true` 且 blockers/errors 为空；不可用时 `ready=false`，blockers 使用
`control_disabled`、`ethercat_unavailable`、`canopen_unavailable` 等稳定 token，详细人类可读
诊断继续由 `SafetyState.active_faults` 承载。RT-Control 不依赖地图，始终令
`map_version=""`。

`robot_interfaces_qos` 继续作为公共 vendor 包构建，RT-Control 已采用的
`control()`、`fast_state()`、`state()`、`latched()`、`diagnostic()` profile 不变。

## 验证

测试先按新契约进入 RED：旧实现产生字符串 `"1100"`，source-lock 仍为 0.6.1/旧 SHA，
readiness blocker 仍是动态诊断句子；实现后完成以下 GREEN 验证：

- 聚焦 pytest：27 tests，全部通过。
- `tools/quality_gate.sh`：187 tests 全部通过；repository gate 覆盖率 85%。
- `python3 tools/release_test_runner.py validate`：`OK: 33 cases valid`。
- 隔离工作区按上游 `d8236bd...` 构建
  `robot_system_interfaces`、`robot_rt_control_interfaces`、`robot_interfaces_qos`、
  `rt_control_interfaces`、`control_api_adapter` 共 5 包；`colcon test-result` 为
  22 tests、0 errors、0 failures、0 skipped。真实生成 `ErrorInfo` 与 RT-Control 适配器
  联合赋值 smoke 确认 `code=1100` 是整数。
- `ROS_DOMAIN_ID=142` 的同进程 ROS publisher/subscriber smoke 使用真实
  `rt_status_adapter` 节点和 `Q_LATCHED` 等价订阅，确认无输入时发布
  `UNAVAILABLE`、`map_version=""`、空 errors 和 6 个稳定 blocker token。
- 无代理 Docker clean build 完成 28 个包，镜像
  `sha256:59a476fbb5023a0b36aba47db8323c2b4c73a951cd7a05e42295abeaed5c82f1`；
  `--network none`、无设备容器 smoke 确认 vendor HEAD/source-lock 均为 `d8236bd...`、
  契约 0.7.0、整数 DREE 映射以及全部 5 个命名 QoS profile 可实例化。仅出现既有
  Lely 构建警告与 CANopen DCF `DynamicChannels`/`[607E]` 警告。

一次独立进程 Domain 142 订阅尝试未形成证据：这台主机上的隔离前缀节点没有出现在另一
进程的 ROS graph，订阅超时；stack dump 显示提供方 executor 正在 spin，未出现 schema 或
回调异常。因此本记录不声称完成多进程跨域 smoke，该项保留到各域同 SHA 的联合验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `ErrorInfo` 是跨域公共错误载荷；`code` 的唯一权威语义是 `uint32` DREE，导航/建图
  内部诊断字符串必须放在域内类型。
- F2: 当前 `DomainReadiness` 是最终公共字段集；`ready` 是唯一准入裁决，状态组合遵守
  上游 IDL 一致性规则，非地图依赖必须令 `map_version=""`，blockers 必须是稳定 token。
- F3: RT-Control、Motion/Navigation、Perception、Autonomy 及 external 调用方必须使用同一
  `robot_interfaces` SHA 原子升级；新旧 schema 不允许混跑或单域回滚。
- F4: `d8236bd...` 是上游 PR #6 的迁移验证 SHA，不是最终发布身份；PR 合并后必须把两个
  下游 pin 一起更新为最终 main SHA。0.6.1 的共同回滚基线是
  `1a60d83d52aa97952c8dbb3baafb50b6a95b9e86`。

## 遗留

等待上游 PR #6 取得 Ruleset 要求的至少两名批准并合并，随后回填最终 main SHA；
Motion/Navigation、Perception、Autonomy 和 external 调用方需分别提交同 SHA 迁移并完成多进程
跨域 smoke。上述条件满足前，本下游 PR、镜像发布和部署仍被阻止。当前镜像仅是本地验证
产物，不是批准的 release 镜像。
