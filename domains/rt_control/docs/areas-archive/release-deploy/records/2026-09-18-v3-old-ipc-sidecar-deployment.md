---
id: release-deploy-20260918-02
area: release-deploy
title: V3 分支旁路部署到旧工控机
date: 2026-09-18
type: feature
trigger: 用户指定将 robot_driver/v3 部署到旧工控机 ar@192.168.100.40
commits: [1256885ce3b99c794bc9ea68d4b353f880166337]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [release-deploy-20260918-01, contract-20260918-01, ELECTRI-118]
---

## 背景

用户确认 `ar@192.168.100.40` 是旧工控机，并要求部署远端 V3 分支。部署必须与当前
release 和其他工作树隔离，不修改 active release link，也不启动真实硬件。

## 改动

- 将精确 `v3@1256885` bundle 部署到
  `/home/ar/rt-control-v3/releases/1256885/robot_driver`；bundle 保存在 release artifacts。
- 在 release 根导入四个冻结 vendor 仓库、应用全部批准补丁并执行 merge/symlink build。
- 未修改 `/home/ar/rt-control-current`，未调用 V3 host `doctor`，未改旧机 host identity。

## 验证

- source bundle SHA-256：`064a5ff1db27e4b1afa70f6b6785416463528c6e5283bfe3b35c8a46b5478576`；clone clean、fsck PASS。
- 34 packages 构建和 runtime package/QoS closure PASS；二次 `prepare` 验证补丁幂等。
- 质量门禁 286 passed、13 skipped、83%；完整测试 1697 tests、0 errors/failures、23 skipped。
- 安装态 arms-only validation PASS：14 CSP + 2 PP，CANopen/达妙 CAN not_required。
- 最终 current link 仍为 `/home/ar/rt-control-operators/51f603c05650979aa9861f712a9b4d0b3f17879f/robot`，无 V3 ros2_control 进程。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `ar@192.168.100.40` 是旧工控机/V3 测试目标，不是当前批准生产 IPC。
- F2: V3 source/build/test 已旁路部署并可追溯，但没有切换当前 release 或启动硬件。
- F3: V3 host identity 锁定新工控机；旧机 validation 成功不能通过删除 identity gate 外推为真实 runtime 准入。

## 遗留

保留旁路 release 供源码/接口联调。真实机械臂 runtime 需在 V3 驱动、控制器、标定与安全
门禁闭合后，由单独现场授权在匹配 host profile 上验证；不得从本次 T1 结果直接启动。
