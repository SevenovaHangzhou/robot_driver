---
id: io-power-20260817-01
area: io-power
title: 纠正左右电磁阀输出映射
date: 2026-08-17
type: corrective
trigger: 用户 2026-08-17 现场左右电磁阀逐点测试确认 / BQ-124
commits: [main]
env: native
risk: T4
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PASS
evidence: [用户 2026-08-17 现场左右电磁阀逐点测试确认, RED 9 failed/14 passed, GREEN plc_node 32/32, changed production coverage 95%, isolated colcon 37/37, quality_gate 187/187]
supersedes: []
related: [BQ-124, TC-FM-03]
---

## 背景

2026-07-28 的寄存器测试只证明三个服务分别操作 bit0/bit1/bit2，没有完成左右实体观察；
后续通过 `/vacuum/grip` 联调发现左右阀语义相反。用户完成现场逐点测试后确认旧左右输出
假设写反，需要纠正服务名、命令写位与 `%MW211` 状态解码的一致映射。

## 改动

`LEFT_SOLENOID_BIT` 从 0 改为 1，`RIGHT_SOLENOID_BIT` 从 1 改为 0，泵继续使用 bit2；
`/plc/left_solenoid`、`/plc/right_solenoid` 服务绑定以及 `%MW211` 输出状态解码随常量统一
纠正。回归测试覆盖位保持、命令/实际双回读、服务绑定和状态消息。`%MW210` 输入侧未改。

## 验证

- 现场左右电磁阀逐点测试由用户确认为通过。
- 把新增测试重放到修复前生产代码后形成有效 RED：23 项中 9 项因 bit0/bit1 左右断言失败、
  14 项通过；修复后全套 `plc_node` 测试 32/32 通过。
- 本次唯一修改的生产文件 `plc_logic.py` branch coverage 为 95%；全 `plc_node` 包覆盖率
  为既有的 52%，缺口集中在未改动的 ROS 节点生命周期和 Modbus 异常路径。
- 隔离输出目录构建 `rt_control_interfaces`、`plc_node` 两包成功；
  `colcon test-result` 为 37 tests、0 errors、0 failures、0 skipped。
- `py_compile`、`tools/quality_gate.sh` 187/187、release catalog 33 项和 `git diff --check`
  全部通过。

本次实机操作经现场授权，操作人：用户，确认短语入口：现场逐点测试。

## 结论与冻结事实

- F1: `%MW200/%MW211 bit0` 是右臂电磁阀，bit1 是左臂电磁阀，bit2 是共用真空泵。
- F2: 本次没有重新确认 `%MW210 bit0/bit1` 的左右真空输入身份；代码保持原输入映射，
  正式真空闭环验收仍需单独复核。

## 遗留

`%MW210` 左右真空输入映射仍需现场闭环复核；全 `plc_node` 既有覆盖率为 52%，后续应补
ROS 节点生命周期与 Modbus 异常路径测试。未构建新 Docker 镜像，也未执行发布部署。
