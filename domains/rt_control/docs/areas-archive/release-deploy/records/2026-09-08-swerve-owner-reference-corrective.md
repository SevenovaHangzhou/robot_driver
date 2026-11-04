---
id: release-deploy-20260908-01
area: release-deploy
title: 修复舵轮设计更新时误替换头部协议关联
date: 2026-09-08
type: corrective
trigger: ELECTRI-118，用户确认舵轮命名与控制约束后继续校验
commits: []
env: none
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-144, motion-20260908-02, release-deploy-20260907-01]
---

## 背景

前一轮将 `owner_refs.damiao_can` 替换为 `swerve`，混淆头部硬件协议与底盘算法包名，
导致 alfa_v3 manifest 校验失败。当前底盘实现尚未迁入，不能把设计更名表述为已完成包重命名。

## 改动

- 恢复头部既有 `damiao_can` 关联，保持校验器要求的三类硬件协议键，不放宽规则。
- 在底盘配置说明与 BQ-144 中记录迁移目标 `swerve_driver`、无 rezero、无 MIT 增益/驱动轮力矩前馈。
- 将已决定的内部环方案从 pending_facts 移出；4 ms SYNC 为设计要求，设备能力和真实时延仍待验证。
- 新增回归用例，验证 owner_refs 覆盖全部已声明硬件协议，避免底盘更名再次破坏头部关联。

## 验证

- TDD RED：新增测试准确报告缺少 `damiao_can`、多出 `swerve`。
- TDD GREEN：同一测试通过；三个 machine_profile/validator/launch 测试文件共 44 项通过。
- `validate_machine_profile.py --all`：5 个物理 profile、10 个 profile/scope 组合通过。
- `chassis_only --require-runtime-ready`：按预期返回 1，因 draft/TBD 阻止真实运行。
- 未执行底盘电机或编码器通信、ROS 硬件启动、镜像重建和实机验证；本次不改运行代码/公共接口。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: owner_refs 是硬件协议关联表，底盘算法包更名不删除或重新分配头部协议。
- F2: 4 ms 周期与 CSP/CSV 内部环是设计约束，不等同于设备时序或机械性能已验证。
- F3: 本次仅完成配置修复、设计澄清与回归；包迁移及 rezero/MIT/前馈的源码删除仍待实施。

## 遗留

按用户裁决迁入 `swerve_driver`，补齐步科 EtherCAT/CANopen 编码器后端及真实参数；
头部继续保留既有协议引用，独立驱动接入另行处理。不得把本次静态校验通过当成实机准入。
