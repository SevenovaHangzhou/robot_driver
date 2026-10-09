---
id: lifecycle-20261009-01
area: lifecycle
title: 达妙控制器切换响应生命周期修复
date: 2026-10-09
type: fix
trigger: 用户报告 /rt/head/enable 等待服务不可用
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /tmp/damiao-head-runtime/mock-disable-regression.log
  - /tmp/damiao-head-runtime/quality-gate.log
supersedes: []
related: []
---

## 背景

用户使能调用等待服务。检查时无头部控制进程；此前日志显示失能服务在 switch_position_controller 遍历控制器响应时发生段错误。最新一次启动日志曾成功加载全部控制器，故此前段错误并不能解释所有后续退出原因。

## 改动

damiao_head_controller 保留 list_future.get() 返回的响应 shared_ptr，再遍历 controller 数组，避免 C++17 范围循环引用已析构的临时响应。已编译安装到当前 install_head。

## 验证

- cmake --build build_head/damiao_head_controller -j2 和 cmake --install：通过。
- ctest --test-dir build_head/damiao_head_controller --output-on-failure：2/2 测试目标通过。
- 独立 ROS_DOMAIN_ID=76、use_mock_hardware=true：20 次 /rt/head/disable 调用均 ok，进程保持运行；测试结束已退出。
- tools/quality_gate.sh：未通过，扫描到用户此前构建的 build_head/log_head 生成物尾部空白；未删除这些目录。
- 未重新启动实机或执行实机使能、运动；实机服务与退出路径待验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 控制器切换过程必须保留 ListControllers 响应对象直到遍历完成。
- F2: 修复版本在 Mock 中连续20次失能服务调用通过；不代表真实使能验证通过。

## 遗留

实机重启前取消待执行使能调用，确认服务可发现后由现场执行使能。完整质量门禁受现有未忽略构建生成物阻塞。
