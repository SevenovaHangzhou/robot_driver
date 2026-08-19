---
id: motion-20260819-12
area: motion
title: ELECTRI-102 mock 长稳与 Motion 公开接口交接
date: 2026-08-19
type: feature
trigger: ELECTRI-102 / T-18 / T-20 / E102-D28
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [tools/electri_102_mock_gate.py, tools/electri_102_motion_mock_producer.py, tools/electri_102_public_mock_harness.py, domains/rt_control/docs/electri-102-motion-integration-guide.md, domains/rt_control/docs/electri-102-mock-performance-report.md]
supersedes: []
related: [ELECTRI-102, motion-20260819-02, lifecycle-20260819-04, BQ-138]
---

## 背景

Rolling controller、模式切换和 provisional 包络完成后，还需要证明 10 Hz knot／30 Hz batch
能在 fake 250 Hz loop 中长期稳定，并给 Motion 一份只依赖公共 IDL 的可执行生产端示例。

## 改动

- 新增 deterministic acceptance runner，将 14 个异常／长稳场景分别绑定到明确 gtest，输出
  单项 log/JUnit、aggregate JSON/JUnit 和 soak metrics，缺二进制、零测试、缺字段或任一
  safety counter 非零都 fail-closed。
- 增加 10 Hz knot、30 Hz batch、fake 250 Hz 的环境可控 soak；实时路径启用 allocation trap，
  同时记录 update/validation/callback visibility 分位数。
- 增加 public-IDL-only Motion producer：默认 dry-run，live 路径需显式开关；实现 verified
  mode CAS、open/prime、30 Hz full suffix、两阶段 close 和回 FJT。
- 增加独立 public DDS peer，跨进程验证五端点、固定轴序、ID、sequence、六点 100 ms hold
  suffix 和完整生命周期。夹具不启动 controller-manager、hardware 或 enable。
- 写明非伺服轴必须从上一条 accepted trajectory 的对应未来时刻采样、Motion 负责 C1 整形、
  视觉丢帧继续保持链路以及全部 RejectCode 响应。

## 验证

- 14/14 场景 PASS，aggregate 0 failures/0 errors。
- 真实墙钟 600 秒：150,000/150,000 fake update 周期；18,001/18,001 batch accepted；reject、
  LateReplace、RT allocation、invariant failure、late cycle 均为 0；最大有效点数为 6。
- 桌面分位数：update p99.9 34.380 µs，validation p99.9 127.092 µs；direct test-peer
  callback→RT p99.9 23.424 µs，明确不作为 DDS 端到端数据。
- public DDS peer 一次验收接受 7 个连续 sequence，最终无 session 且回到 FJT_READY。
- producer/harness Python 测试 6/6 通过；默认 producer 不初始化 ROS。

标准 GenericSystem bringup 的公共 enable 因不模拟 CiA402 而在
`right_joint1/status_word=0x0040` fail-closed；没有绕过生产准入来制造假 PASS。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: ELECTRI-102 软件门包含 14 项机器可读矩阵和一次真实墙钟 600 秒 fake-250 Hz soak。
- F2: 10 Hz knot／30 Hz batch 在该 soak 中 18,001 批全部接受，全部零容忍计数为 0。
- F3: Motion 最小 producer 只依赖公共 robot_interfaces 包，默认 dry-run，FJT cancel/await 仍由调用方负责。
- F4: public DDS peer 与 controller fake-loop 是互补证据；前者不替代轨迹语义，后者不宣称 DDS 端到端。
- F5: standard GenericSystem 不模拟 CiA402 enable，不能宣称完整标准 bringup live session 已通过。

## 遗留

目标机 DDS／generation／STRICT switch 延迟和 callback PSR 未测；手眼外参、production envelope、
真实相机和硬件运动仍未完成。接口功能分支可推送供 Motion 使用，但按用户裁决暂不提 PR。
