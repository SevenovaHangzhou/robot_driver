---
id: io-power-20261009-01
area: io-power
title: ELECTRI-131 双电池评审修复与启用门禁
date: 2026-10-09
type: fix
trigger: ELECTRI-131；用户对 PR 53 的逐项裁决
commits: [bugfix/electri-131-bms-review]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [/tmp/electri131-bms-build.log, /tmp/electri131-bms-test.log, /tmp/electri131-bms-results.log]
supersedes: []
related: [BQ-152, ELECTRI-140]
---

## 背景

部分完成：从 PR #53 独立提取 BMS。自动控制仍默认关闭。
本记录只冻结软件修复范围，不放开实机准入。

| 用语 | 含义 |
| --- | --- |
| 停负载 | 发布既有停负载请求，不等同于负载已停止 |
| MOS 回读 | `0x93` 中的放电 MOS 原始状态 |
| K2 回读 | 既有 Bool 接口报告的辅助触点状态 |

## 改动

- F01：单线程继续拥有 CAN socket。控制周期与重连期限独立。
  CAN 不可用时控制仍运行。D9 写失败不直接重置连接。
- F02：主、副电池 D9 重试前复核停负载、电流及数据新鲜度。
  条件失效时发送停负载请求。K2 重试语义本轮不修改。
- F03：接入和运行中监视主电池 MOS。意外关闭后只请求停负载。
  状态锁存为 `awaiting_primary_mos_isolation_policy`。
  软件禁止据此自动断开 K2、转移负载或恢复运行。
- F06：命令前等待时，K2 或主电池 MOS 回读过期只阻止接入并清空稳定计时。
  新鲜回读确认 K2 意外闭合时仍锁存故障。
  D9 发出后，等待 ACK/MOS 时的回读过期或意外闭合均锁存故障。
- F05：健康检查只接受运行状态 `0` 和 `2`。
- F04：节点检查四个控制话题。ROS 名称展开和 remap 后必须两两不同。
  本轮不新增执行端就绪握手。
- F09：正常阶段推进输出 INFO，故障处理输出 WARN，锁存输出 ERROR。
  状态不变时不重复刷日志。

## 验证

- 隔离 `colcon build`：BMS 与冻结 QoS 依赖通过。
- `ctest --test-dir /tmp/electri131-bms/build/bms_node --output-on-failure`：5 个目标通过。
- `colcon test-result --test-result-base /tmp/electri131-bms/build --verbose`：46 条汇总结果，零失败。
- `tools/quality_gate.sh`：207 passed、13 skipped，门禁覆盖率 83%。
  本机未安装 ShellCheck，由 CI 执行。
- ROS 测试使用独立域与不存在的 `bms_missing` 接口。
  验证离线控制周期仍输出停负载与 K2 保持命令。
- 配置测试覆盖六种两两冲突、相对名别名和 remap 冲突。
- 首次构建因新增测试缺少 executor 头文件失败。补齐后构建和测试通过。
- 未执行 CAN 总线、D9、K2、使能、运动或目标机部署。

本记录不授权使能或运动。

## 结论与冻结事实

- F1：断线不能停掉软件控制周期；D9 失败不改变该周期。
- F2：D9 每次重试必须重新通过低负载检查。
- F3：主电池 MOS 意外关闭只触发停止和锁存；隔离策略待定。
- F4：K2 回读与主电池 MOS 是接入阶段的持续条件。
- F5：默认监控模式不新增写动作。

## 遗留

以下四项必须记录到 ELECTRI-131。裁决前禁止启用自动控制：

| 待定项 | 必须冻结的内容 |
| --- | --- |
| F07 受控退出 | 节点、K2 驱动、负载侧与工控机供电的退出顺序及超时责任 |
| K2 命令语义 | 电平/边沿、防抖、命令丢失和心跳超时；重试失去低电流条件后的保持行为 |
| 主电池 MOS 意外关闭 | 停负载后的隔离目标、K2 状态和工控机供电保持；等待策略状态提前返回而不处理后续副电池故障的例外 |
| 故障锁存后的停负载 | 请求是否持续发送、执行端是否锁存及恢复授权 |

物理 K2、真实反馈、安全阈值、预充和独立保护继续受 BQ-152 限制。
F08、F10、F11 延期。F12 在独立字节序修复 PR 中处理。

## 等待阶段回归纠正

用户复核发现命令前等待阶段短暂丢失回读会锁存故障。
修复区分 `waiting_before_command` 与 `waiting_for_d9`。
仅主电池 MOS 帧过期的等待场景不锁存；其他健康状态丢失保持原故障策略。

新增回归覆盖副电池缺席时的反馈恢复、压差等待及稳定窗口重置。
两场景分别注入 K2 和主电池 MOS 帧过期。
额外确认副电池缺席时的新鲜意外闭合仍锁存。
真实故障、非 MOS 状态过期以及 D9 发出后的 MOS 回读过期仍保持锁存。
用户提供的三个探针在修复源码上通过；第三个仅确认策略等待的已知例外，
不是证明第二块电池故障处理已完成。

纠正后的隔离构建与 5 个 CTest 目标通过，汇总 51 条结果、零失败。
质量门禁仍为 207 passed、13 skipped，门禁覆盖率 83%。
日志：`/tmp/pr56-followup-build.log`、`/tmp/pr56-followup-test.log`、
`/tmp/pr56-followup-results.log`、`/tmp/pr56-followup-quality.log`。
上述数据为 rebase 前增量验证，rebase 后重新核对。
