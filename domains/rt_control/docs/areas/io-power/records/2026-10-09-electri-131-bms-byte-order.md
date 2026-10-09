---
id: io-power-20261009-02
area: io-power
title: ELECTRI-131 BMS auto 字节序拒绝相同 SOC 歧义帧
date: 2026-10-09
type: fix
trigger: ELECTRI-131；PR 53 评审 F12；用户要求独立修复
commits: [bugfix/electri-131-bms-byte-order]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [/tmp/electri131-byte-order-build.log, /tmp/electri131-byte-order-test.log, /tmp/electri131-byte-order-results.log]
supersedes: []
related: [BQ-152]
---

## 背景

软件修复通过离线测试；实机字节序仍待确认。
默认 `auto` 曾在 SOC 两端解释相同时选择大端。
该行为与“唯一确定才锁定”的文档不一致。

## 改动

删除相同 SOC 值时返回大端的分支。
两个字节序都合法时，解码器返回拒绝。
拒绝不修改原样本或已接收时间，也不锁定字节序。
显式字节序和已经唯一锁定的字节序保持原行为。
本 PR 不引入双电池、D9 或 K2 控制。

## 验证

- 隔离 BMS/QoS 构建通过。
- 3 个 CTest 目标通过；汇总 14 条测试结果，零失败。
- `tools/quality_gate.sh`：207 passed、13 skipped，门禁覆盖率 83%。
  本机未安装 ShellCheck，由 CI 执行。
- 回归覆盖 `00 00`、`01 01`、`02 02`、`03 03` SOC 字节。
- 验证歧义帧不污染旧样本，后续唯一帧仍能确定小端。
- 验证显式大小端继续接受 SOC 为零的帧。
- 未读取实机 CAN，未核对 HMI 或设备字节序。

本记录不授权使能或运动。

## 结论与冻结事实

- F1：相同 SOC 值不能证明电压字段的字节序。
- F2：`auto` 仅在 SOC 范围唯一确定字节序时锁定。
- F3：等待唯一帧时保持既有数据过期策略，不伪造有效电量。

## 遗留

现场必须确认实际字节序及显示对照。
双电池状态机修复在独立分支实施。
