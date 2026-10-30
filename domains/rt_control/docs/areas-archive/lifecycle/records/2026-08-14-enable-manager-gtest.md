---
id: lifecycle-20260814-02
area: lifecycle
title: enable_manager 状态机表驱动 gtest（ELECTRI-93）
date: 2026-08-14
type: feature
trigger: ELECTRI-93（T-DEV-NATIVE-003 丢弃后的干净实现）
commits: [a380879（脚手架）, 3ff153d（36 用例套件，工控机分支 feature/electri-79-clean-gtest，已 fetch 回主仓 electri-93-from-ipc）]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [工控机 /home/ar/rt-control-dev/electri-79-evidence.log（184 行），colcon test-result 37 tests 0 failures，连续 5 轮 36/36 无抖动]
supersedes: []
related: [ELECTRI-93, ELECTRI-92, lifecycle-20260814-01]
---

## 背景

enable_manager（1320 行、14 轴 5 批次状态机）此前零单测。基于干净
main @ 74399af 的线上原版语义实现表驱动 gtest，与 ELECTRI-92 修复解耦。

## 改动

- 新增 test/test_enable_manager_state_machine.cpp（1238 行，36 用例）：14 轴内存
  CiA402 模拟 + 虚拟时钟直驱 update()，不 mock controller_manager——其缺席让
  switchJtc 的 wait_for_service(0ms) 探测直接短路到 kAmbiguous，反而成为 f 项
  覆盖。a–f 六项全交付：a 五批次推进（控制字 0x0006→0x0007→0x000F、批次隔离、
  inter_batch_delay）；b 批次超时（5 例参数化覆盖全部批次，failed_batch/
  failed_joint/status_word/stage 双端断言）；c 两段式复位（0x0080 仅到 Fault 轴，
  4 组故障集参数化 + already_clear + fault_reset_timeout）；d 失能 staging
  （有序下行 + 超时升级 EmergencyQuickStop 0x0002）；e Owner 独占（立即拒绝
  operation_in_progress，无排队，语义按实际行为固化）；f kAmbiguous 可达并
  验证 restart_required_ 锁存。
- CMake BUILD_TESTING 块 + package.xml test_depend；生产代码仅 4 行可测性
  钩子（头文件 friend 声明 + 注释），enable_manager_controller.cpp 零改动；
  -DBUILD_TESTING=OFF 下无 add_test，生产镜像不受影响。

## 验证

- 已验证（T0，工控机实跑）：36/36 通过，连续 5 轮无抖动；并发防护全程
  无冲突（操作员 Domain 0 栈未受影响，agent 仅用 Domain 142 且已清理）。
- 未验证：真实硬件时序（驱动模型为模拟）；service_result_timeout_ms=30000
  生产壁钟路径（测试用 300ms 虚拟钟）；诊断发布、handleNonRtFaultStop、
  on_deactivate、JTC 成功路径（无 controller_manager 不可达）；本包 gcov
  覆盖率未测量。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: enable_manager 单元测试基线 = 36 用例 @ 3ff153d，覆盖 a–f 六项；后续
  行为修改必须先过此套件或显式修订对应用例。
- F2: 测试发现的三项行为特征（报告不修）：① finishDownward 仅在 owner 匹配时
  发布结果槽——经真实服务不可达，但重构前须知的脆弱耦合，与
  lifecycle-20260814-01#F1 的"RT 循环未填槽"病因假设相关；② 批次超时报告的是
  批内第一个未达标轴，未必是真卡住的轴；③ ReadyToSwitchOn 仅对 Ti5 四轴
  {1,2,7,8} 视为失能终态，批 0/1 与批 2/3/4 收敛路径不同（设计使然）。

## 遗留

- ELECTRI-92 步骤 2 建议在本套件上补"RT 循环不填槽 → 服务 handler 行为"的
  单元复现（F2① 是入口）。
- 本包 gcov 覆盖率测量与 CI 集成（colcon test 已在 CI，覆盖率未然）。
