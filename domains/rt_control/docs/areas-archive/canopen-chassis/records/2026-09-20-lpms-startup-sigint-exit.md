---
id: canopen-chassis-20260920-01
area: canopen-chassis
title: LPMS 启动窗口 SIGINT 正常退出纠错
date: 2026-09-20
type: fix
trigger: PR #44 热缓存 CI 连续两次暴露 test_missing_interface_logs_failure_and_exits_cleanly_on_signal 返回 1
commits: [feature/v3-head-can-integration]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - "PR #44 run 35481600633 attempts 2/3: same LPMS SIGINT exit test failed"
  - "focused native build: lpms_nav3_can PASS"
  - "focused colcon test: 30 tests, 0 errors, 0 failures, 0 skipped"
  - "test_node_without_can.py: 9 passed"
  - "focused SIGINT process test: 50 consecutive passes under concurrent pytest load"
  - "tools/quality_gate.sh: 297 passed, 13 skipped, policy coverage 83%"
supersedes: []
related: [canopen-chassis-20260917-01, ELECTRI-105]
---

## 背景

LPMS 节点构造函数在缺少 CAN 接口时先记录降级日志，再返回给 `main` 进入
`rclcpp::spin`。进程测试看到该日志后立即发送 SIGINT；若 ROS context 在 `spin`
建立 executor 前完成 shutdown，`spin` 会抛异常，而原 `main` 无条件把所有异常映射为
启动失败退出码 1。该竞态在本机低负载下 50 次未出现，但在 PR #44 并行 CI 中连续两次
稳定出现。

## 改动

- `main.cpp` 捕获异常后先检查 `rclcpp::ok()`：context 已由 SIGINT 关闭时按正常停止
  返回 0；context 仍有效时保留错误输出和退出码 1。
- 进程测试改用 `communicate()` 收集 SIGINT 后的剩余输出，失败断言会携带完整节点日志，
  不再只报告退出码。
- CAN 解码、socket 重连、参数、topic、诊断、启动配置和生产 launch 均未修改。

## 验证

- 独立 workspace `colcon build --symlink-install --packages-select lpms_nav3_can`：通过。
- `python3 -m pytest -q .../test_node_without_can.py`：9 passed；其中六种无效配置继续
  要求退出码 1。
- 关键 SIGINT 进程用例在另一组 pytest 并发负载下独立运行 50 次：50/50 passed。
- `colcon test --packages-select lpms_nav3_can`：30 tests、0 errors、0 failures、0 skipped；
  `tools/quality_gate.sh`：297 passed、13 skipped，策略覆盖率 83%。
- PR #44 修复前同一 SHA 的完整 CI 首轮通过，随后两个热缓存 attempt 均在同一断言
  返回 1；修复后的远端完整闭包仍待确认，因此保持 PARTIAL。

未打开 SocketCAN 接口，未收发 CAN 帧。本记录不授权使能或运动。

## 结论与冻结事实

- F1: LPMS 节点在 ROS context 已由 SIGINT 关闭时必须以 0 正常退出，即使关闭发生在
  node 构造完成与 `spin` 建立之间。
- F2: 参数/构造异常发生时 ROS context 仍有效，必须保留错误日志并返回 1；SIGINT
  正常退出不得弱化 fail-fast 配置校验。

## 遗留

在 PR #44 完整 V3 CI 中确认修复后的进程测试与全部闭包通过。真实 CAN 连接、断流、
热插拔和跨进程 DDS 验证仍属于原 ELECTRI-105 遗留，不由本纠错关闭。
