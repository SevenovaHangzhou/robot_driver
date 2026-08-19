---
id: contract-20260819-02
area: contract
title: ELECTRI-102 锁定 rolling 公共接口功能 SHA
date: 2026-08-19
type: feature
trigger: ELECTRI-102 / 用户要求接口可供 Motion 测试后再提 PR
commits: [d5bad6768101eeb1be252438fdc1fb1303e27c89]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [robot_interfaces@9cc937970736cd19fd3bf5283de8cc5c15926967, deps.repos, src/interfaces/source-lock.yaml, /tmp/e102-interfaces-final-build, /tmp/e102-driver-final-build, /tmp/e102-final-gate.a6QkWh]
supersedes: [contract-20260817-01]
related: [ELECTRI-102, contract-20260819-01, motion-20260819-12]
---

## 背景

ELECTRI-102 的 command batch 属 Motion provider，session/service/state 属 RT-Control provider；
driver 只有锁定含这两组 IDL 的不可变接口 SHA，Motion 才能用同一 wire schema 实际联调。
用户裁决允许推送功能分支，但明确要求暂不提接口 PR。

## 改动

- `robot_interfaces` 功能分支 `功能/视觉伺服-ELECTRI-102` 已推到 origin，SHA 固定为
  `9cc937970736cd19fd3bf5283de8cc5c15926967`；没有创建 PR、tag 或 release。
- `deps.repos` 与 `src/interfaces/source-lock.yaml` 同步锁该完整 SHA，`contract_version` 仍为
  当前包版本 `0.7.0`，不把 Unreleased 功能分支伪装成 0.8.0 正式发布。
- vendored package 清单新增 `robot_motion_interfaces`，与已有
  `robot_rt_control_interfaces`、`robot_system_interfaces`、`robot_interfaces_qos` 一起成为
  driver 的公开依赖闭包；repository gate 和锁定测试同步更新。

## 验证

- interface 仓：生成／check、contract、error-code、changelog 门禁全绿，54/54 unittest；
  ROS 2 Humble 全 6 包 clean build；rosdep check 无缺项。
- driver 与 interface 两个真实源树联合 clean build 13 包成功；driver package test-result
  汇总 276 tests、0 errors、0 failures、0 skipped，包含 7 项完整 mock launch 验证。
- 最终 HEAD 的 accelerated 14 项 acceptance matrix PASS；此前同 controller 实现已完成真实
  墙钟 600 秒长稳。
- public DDS producer 使用该接口 overlay 运行 1 秒，连续 31/31 batch 接受，完成双向 mode、
  open/prime/close/finalize，最终回到 FJT_READY；未启动 hardware/controller-manager/enable。
- repository quality gate 209 passed、1 个 external-overlay test 在通用环境显式 skipped；同一
  test 在锁定 overlay 下单独 PASS。skip 不被计作接口验证通过，专门门才是其通过证据。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本功能分支唯一 rolling 公共接口身份是 `9cc937970736cd19fd3bf5283de8cc5c15926967`。
- F2: driver 的公开依赖闭包包含 Motion、RT-Control、System 和命名 QoS 四个接口包。
- F3: 该 SHA 已通过 public producer/consumer mock，不是 main、PR、0.8.0 或跨域正式发布。
- F4: Motion 测试必须与 driver 使用同一 SHA；混用 f699f45 或未来 schema 不受支持。

## 遗留

等待 Motion 用真实生产端实现反馈接口可用性后，再由人工发起 robot_interfaces PR、决定
0.8.0、合并 SHA 和四域原子升级。接口 PR、merge、tag 和部署均未在本记录执行。
