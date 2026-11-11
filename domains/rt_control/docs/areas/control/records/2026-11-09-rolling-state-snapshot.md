---
id: control-20261109-01
area: control
title: Rolling 补窗与状态发布使用一致的实时摘要
date: 2026-11-09
type: fix
trigger: 用户要求处理 Rolling 审查中的两处状态读取问题；SEV-72；Related to GitHub #250
commits: [bugfix/rolling-state-snapshot]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [control-20261105-03]
---

## 背景

原先的版本计数写入区覆盖整个 RT update，非实时侧连续八次读取可能全部撞上写入，
将有效补窗报告为 NOT_ACCEPTING。状态发布还独立读取运行状态与停止原因，不能保证两者
来自同一次更新。本次修复对应 SEV-72；GitHub #250 中历史现场拒绝的完整根因仍需现场时序证据。

## 改动

- RT 在周期完成时发布固定大小的执行摘要；补窗和状态发布经同一串行消费者读取最新完整摘要。
  写入中的槽不影响上一份完整摘要，删除八次紧循环及分散的速度原子反馈。
- 轨迹交接与状态摘要共用包内预分配三槽组件，保留单生产者/单消费者及槽所有权约束。
  RT 不等待、不分配；激活/失活仍按既有约束与 update 互斥。
- 摘要新鲜度使用已有 maximum_controller_period_ms，不新增数值阈值。过期摘要仍拒绝补窗。
  提交前继续读取最新执行时间、替换边界与已消费代次，防止滞后的摘要误判已接收的替换为迟到。
- 限值加载仍使用机器人模型加速度作为制动减速度。因此摘要滞后的执行时间可作为停车时间
  增长上界加入准入余量，随后再加既有调度余量；不修改模型、制动参数或窗口容量。
- 已接受首批而 RT 尚未观察到新会话时继续报告 STARTING；首批被拒绝时继续保留前次停止原因。
  其余执行状态与停止原因来自同一完整摘要；重新激活先发布本次初始状态。

## 验证

在 `main` 的 `16fa212ec2b1efd748486e06158e0d364191b755` 上隔离修改，ROS 2 Humble、
Release 构建；验证仅达到 T0，没有连接硬件。

- `colcon build --packages-select ctrl_rolling_trajectory integ_bringup`：PASS。
  宿主 `controller_manager 2.54.3` 最初被加载包的版本准入拒绝，随后在临时目录构建该包
  已支持的上游 `2.54.2` 并显式指定 CMake 前缀，未修改版本准入、宿主安装或上游源码。
  临时 Python 工具目录的 setuptools/packaging 冲突也在该目录解决。
- `tools/run_scoped_tests.sh --base origin/main`：Rolling 的 63 项全部通过；下游最初有
  8 项因测试环境缺少 `integ_composer` / `integ_diagnostics` 导入路径而失败。
  补齐本版本源码的 Python 路径后，
  `colcon test --packages-select integ_bringup --ctest-args --rerun-failed --output-on-failure`
  重跑失败目标通过。原始结果 XML 合计 Rolling 63 项、下游 33 项，均无失败或跳过。
- `colcon test-result --verbose`：PASS，汇总 105 项、0 errors、0 failures、0 skipped
  （该汇总包含 CTest 包装目标；96 项原始用例不重复计算）。
- 新增 7 项回归：未发布写槽不影响有效补窗；运行状态/停止原因成对发布；过期摘要拒绝；
  最新替换边界检查；重新激活清除旧摘要；未发布的消费确认；执行滞后的停车时间增长余量。
  并发补窗用例改为 80 个有效未来窗口必须全部接受，既有百万次交换及 RT 零分配测试通过。
- `REQUIRE_STYLE_TOOLS=1 tools/quality_gate.sh`：PASS，工具测试 185 通过、11 按环境跳过；
  两组门禁覆盖率分别为 85% / 84%；clang-format 14 与 ruff 0.11.13 校验通过。
  本机没有 ShellCheck，按既有门禁输出由 CI 执行；本次没有 shell 改动。

测试环境限制 DDS 网络访问；上述包测试通过直接调用真实控制器和仿真接口验证行为，
不构成 ROS 网络端到端、T1 加载运行或实机验证。未执行 sanitizer 验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: RT 写入中的状态摘要不会使读取方失去上一份完整摘要；准入仍检查摘要新鲜度和最新执行边界。
- F2: 对外执行状态与停止原因来自同一份完整摘要，首次接受与拒绝的会话反馈语义保留。
- F3: 本次没有实机运动证据，不能据此确认 GitHub #250 所有现场拒绝已解决。

## 遗留

实机连续补窗、停车与重新启动待验证；PR 在取得实机证据前保持 Draft。
