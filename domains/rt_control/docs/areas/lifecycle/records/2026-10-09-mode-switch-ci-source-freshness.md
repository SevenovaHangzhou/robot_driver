---
id: lifecycle-20261009-03
area: lifecycle
title: 模式切换 CI 测试持续提供源控制器状态
date: 2026-10-09
type: fix
trigger: 用户要求修复 PR 53 的 CI 报错并重新提交
commits: [feature/rt-control-program-optimization]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /tmp/ci53-logs/0_build.txt
  - /tmp/ci53-build.log
  - /tmp/ci53-tests.log
  - /tmp/ci53-repeat.log
  - /tmp/ci53-single-sample.log
  - /tmp/ci53-quality.log
supersedes: []
related: ["robot_driver#52", "robot_driver#53"]
---

## 背景

CI run 37887591905 的 governance 通过，build 在测试步骤失败。39 个包执行结束，
唯一失败用例为 EnableManagerFixture.ModeServiceDoesNotWaitForRollingStateAndReplaysResult。
结果汇总显示 1989 条结果、2 failures、23 skipped，其中两个失败条目分别是同一
GTest 用例及其 CTest 包装结果，不是两个独立缺陷。

## 改动

仅修改 enable_manager 的测试及验证记录，不修改生产状态机、接口、硬件配置或阈值。
原测试仅在请求前注入一次 JTC 状态；CI 中停稳判定完成后，这个快照已经超过生产
100 ms 新鲜度限制，返回 SOURCE_STATE_STALE=11，因此未调用 list/switch 服务。
测试更新线程现在在每次 RT update 前提供新的 JTC 状态，模拟运行中的源控制器。
显式等待模式请求获得所有权后延迟 150 ms 再启动更新泵，覆盖超过新鲜度有效期的
调度延迟。保留停稳判定、严格切换、未收到 rolling 状态也返回和幂等重放断言。

## 验证

ROS 2 Humble；隔离 build/install 位于 /tmp/ci53。所有 ROS 测试使用独立本机
ROS_DOMAIN_ID=79、ROS_LOCALHOST_ONLY=1，无硬件访问。

- `colcon --log-base /tmp/ci53/log build --base-paths
  src/src/vendor/robot_interfaces/qos src/src/vendor/robot_interfaces/robot_system_interfaces
  src/src/vendor/robot_interfaces/robot_rt_control_interfaces src/interfaces/rt_control_interfaces
  src/rt_control/rt_control_semantic_components src/rt_control/enable_manager
  --build-base /tmp/ci53/build --install-base /tmp/ci53/install --parallel-workers 2
  --cmake-args -DBUILD_TESTING=ON`：PASS，六个包完成。
- `ctest --test-dir /tmp/ci53/build/enable_manager -T Test --output-on-failure`：PASS，
  完整 65 个 GTest 用例通过。
- `colcon test-result --test-result-base /tmp/ci53/build --verbose`：PASS，
  66 条汇总结果、0 errors / 0 failures / 0 skipped。
- `test_enable_manager_state_machine
  --gtest_filter=EnableManagerFixture.ModeServiceDoesNotWaitForRollingStateAndReplaysResult
  --gtest_repeat=30 --gtest_break_on_failure`：PASS，带 150 ms 延迟的修复版本连续 30 次通过。
- 隔离对照：复制测试源码到 /tmp，仅移除周期性 JTC 刷新行，保留相同 150 ms 延迟，
  用同一套编译/链接参数构建。该版本退出码 1 并返回 SOURCE_STATE_STALE=11，稳定复现
  CI 根因；用户源码与生成构建文件未因此改写。对照失败是预期结果。
- `COVERAGE_FILE=/tmp/ci53-quality.coverage bash tools/quality_gate.sh`：PASS，
  207 passed、13 skipped，门禁覆盖率 83%；本机无 ShellCheck，由 CI 强制检查。
- `git diff --check`：PASS。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 成功模式切换测试应持续提供有效源状态，不能假设一次快照在 CI 调度期间永久有效。
- F2: 固定 150 ms 延迟可区分单次旧快照与持续刷新行为；生产 100 ms 新鲜度和
  2 ms 停稳采样门禁均保持不变。

## 遗留

新的全仓 CI 待推送后验证；未进行实机使能、运动或生命周期验收。此修复不关闭
此前 BMS、力传感器、头部及标定相关的现场准入限制。
