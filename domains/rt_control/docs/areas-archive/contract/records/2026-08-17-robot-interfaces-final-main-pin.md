---
id: contract-20260817-01
area: contract
title: robot_interfaces 回填最终 main SHA
date: 2026-08-17
type: corrective
trigger: 用户 2026-08-17 要求 / upstream robot_interfaces PR #6 已合并
commits: [main]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [upstream robot_interfaces main f699f45972ad15bbbbbb3da1a4894faf209144c9, upstream gates 41/41, robot_driver quality_gate 187/187, affected ROS packages 22/22]
supersedes: []
related: [BQ-136, BQ-137, contract-20260814-01, contract-20260816-01, TC-IF-01]
---

## 背景

RT-Control 的公共接口已迁移到 `robot_interfaces` vendoring，但仍固定在上游 PR #6 的
验证提交 `d8236bda7e087a54a8ee7585bc7a2d6a94251af4`。PR #6 合并后需要回填不可变的
最终 `main` 身份，关闭 BQ-137 的 RT-Control source-lock 遗留项。

## 改动

`deps.repos` 与 `src/interfaces/source-lock.yaml` 同步改为
`f699f45972ad15bbbbbb3da1a4894faf209144c9`，契约版本保持 `0.7.0`；锁定一致性测试、
接口边界说明、BQ-137 当前状态和本区冻结事实同步更新。历史记录仍保留 `d8236bd...`
作为当时构建、Mock 和镜像验证的真实身份，不篡改既有证据。

## 验证

- `git ls-remote` 确认上游 `main=f699f459...`；祖先检查确认 `d8236bd...` 已包含在该
  提交中。两者差异为 12 个文件、`+128/-5`，只新增 Perception traceability 字段及公共
  DREE 常量，契约版本仍为 `0.7.0`。
- 补入缺失的完整 `robot_interfaces` vendor 后，按仓库内已批准补丁补齐
  `ros2_controllers` 命名 QoS patch；`bootstrap_native_dev.sh prepare` 随后确认四个
  vendor HEAD 和冻结补丁树完全一致。未 reset/checkout 既有 vendor。
- 上游 contract/error-code/changelog gate 与 unittest 共 41/41 通过；
  `tools/quality_gate.sh` 187/187 通过，release catalog 33 项有效，锁定聚焦测试 15/15 通过。
- 隔离输出目录构建 `robot_system_interfaces`、`robot_rt_control_interfaces`、
  `robot_interfaces_qos`、`rt_control_interfaces`、`control_api_adapter` 共 5 包成功；
  `colcon test-result` 为 22 tests、0 errors、0 failures、0 skipped。第一次 test 命令因未限制
  discovery 路径误扫 `/home/ar` 而在包识别阶段失败，改用与 build 相同的显式
  `--base-paths` 后通过，前次结果不作为代码失败证据。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: RT-Control 的 `robot_interfaces` 唯一锁定身份是上游最终 `main`
  `f699f45972ad15bbbbbb3da1a4894faf209144c9`。
- F2: `f699f45...` 包含 BQ-137 的 `d8236bd...`，并新增 Perception contract-completeness
  变更；公共契约版本仍为 `0.7.0`。
- F3: RT-Control 回填最终 SHA 不代表跨域发布完成；所有生产者和消费者仍必须锁定同一
  SHA 原子升级，并完成跨进程/跨域 smoke。

## 遗留

Motion/Navigation、Perception、Autonomy 与 external 调用方的同 SHA 迁移和联合 smoke
仍未在本记录中验证；未生成新的 RT-Control 发布镜像，也未部署到运行时。
