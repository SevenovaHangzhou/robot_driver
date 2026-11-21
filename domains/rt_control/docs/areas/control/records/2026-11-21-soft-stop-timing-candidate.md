---
id: control-20261121-04
area: control
title: 头臂软急停采用待现场校核的台架时间候选值
date: 2026-11-21
type: decision
trigger: 用户明确授权参考市场参数估算软急停时间；SEV-135 / robot_driver#299
commits: [codex/control-backlog-fixes]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [control-20261121-03]
related: [SEV-135, robot_driver#299, SEV-73, robot_driver#300]
---

## 背景

用户允许依据公开资料估算本轮头部和双臂软急停参数。本记录只覆盖
control-20261121-03#F3 中“所有时间保持 TBD”的安排，保留其范围、锁存与验收约束。
仅配置 IO 地址已确认的 Alfa_V3-bench-01；其他台套仍有未确认接线，保持 TBD。

公开资料用于选择起始量级，不构成本机响应时间证据：

- [Acromag EtherStax 产品资料，ES2117 输入规格](https://www.acromag.com/wp-content/uploads/2022/06/Acromag-Etherstax_Series_Brochure_453Q.pdf)
  给出输入响应小于 5 ms。这是另一型号模块的输入响应，不是本机 Modbus/DDS 或机械停止时间。
- [Modbus TCP 实现指南 §4.4.1.4](https://www.modbus.org/docs/Modbus_Messaging_Implementation_Guide_V1_0b.pdf)
  没有统一规定事务响应时间；客户端应考虑网络延迟与设备合理响应上限。

## 改动

台套 deployment.yaml 的四项从 TBD 改为以下**工程估算候选值**，不添加节点隐式默认值：

| 参数 | 候选值 | 选择理由 |
| --- | --- | --- |
| poll_period_ms | 20 ms | 以 50 Hz 读取单点输入，作为毫秒级工业 IO 的起始轮询频率 |
| request_timeout_ms | 100 ms | 给单次连接/发送/接收留出 5 个轮询周期的总预算；超时即报告无效输入 |
| stale_timeout_ms | 200 ms | 最后有效样本超过 10 个轮询周期后报告 UNKNOWN，覆盖轮询未推进但发布仍在运行的情况 |
| receiver_timeout_ms | 300 ms | 为控制器设置最后有效样本的失联期限，覆盖输入节点退出或 DDS 无新样本 |

100/200/300 ms 是本轮选取的预算，不是厂家推荐值或行业统一标准。
请求与发布使用不同非实时回调组；请求超时通常先于样本过期触发。
控制器将样本年龄与传输年龄计入接收期限，重复发布旧样本不会刷新期限；
200 ms 和 300 ms 不是两段依次等待。收到按下状态后进入既有停机路径，不等待这些超时。
实际响应仍叠加设备扫描、网络、进程调度、控制周期及驱动/机械停止过程。

## 验证

本机离线验证，未连接现场地址：

- 直接读取本台套配置，8 个头臂 profile/scope 分别生成 real 与 mock artifact，16 项通过；
  输入节点与控制器收到相同台套参数，mock 不启动物理输入节点。
- 真实 periph_emergency_stop 节点使用这组时间，仅将对端替换为 localhost 临时 TCP 端口；
  按下/释放、请求无响应转 UNKNOWN/stop、重连、仅发送 FC02 和 SIGTERM 退出通过。
  本次无响应至 UNKNOWN 的一次本机观察约 119 ms，仅为离线样本，不是现场上界。
- 外设节点既有 3 项回归通过，包含样本过期与真实 DDS 晚加入缓存接收。
- 配置及启动下游的 352 项 Python 回归通过；quality_gate（含格式、ShellCheck 与 arch_check）通过。

可重复的仓库回归入口：integ_composer、integ_bringup、integ_head_move 的 pytest；
periph_emergency_stop/test/test_node.py；tools/quality_gate.sh。
本次离线日志保存在任务临时目录，未将构建产物或日志提交入库。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本台架四项软急停时间已有用户授权的工程候选值；部署状态仍为 draft、verified=false。
- F2: 参数仅供头部和双臂软急停，现场时延、停止时间和停止距离均未验证。
- F3: IO 地址未知的其他台套继续阻止真实 artifact；释放不自动使能、离线重启锁存语义不变。

## 遗留

按既有分级流程完成 T1 全图与 T2–T4，现场由操作台执行。
测量设备输入到 DDS、控制器停机请求与实际停止的延迟分布，在预期 CPU/网络负载下核对误停与漏检。
根据测量修订预算；当前值不能用于宣称硬件安全等级或机械停止上界。
