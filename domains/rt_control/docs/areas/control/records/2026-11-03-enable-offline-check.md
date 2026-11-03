---
id: control-20261103-01
area: control
title: 使能与故障复位前检查设备反馈新鲜度，离线时报 device_offline
date: 2026-11-03
type: fix
trigger: 用户 2026-11-03；台架 16:23 驱动断电期间点故障复位，按断电前最后一帧状态字报 right_joint1 fault_reset_timeout
commits: [feature/arch-enable-offline-check]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [integration-20261103-01]
---

## 背景

台架 dmesg：16:12:58 与 16:22:23 驱动断电，总线只剩 2 个从站，持续到 17:30。16:23:14 的组故障复位在此期间执行；
掉线从站不再更新过程数据，使能管理器按断电前最后一帧判断，right_joint1 停在 0x020F（故障反应进行中），
4 s 后报 `fault_reset_timeout`，轴名误导。

## 改动

- `ctrl_enable_manager`：新参数 `feedback_age_interfaces`（状态接口名，单位 ms）与 `feedback_age_limit_ms`
  （默认 100，用户 2026-11-03 选定，所有作用域共用）。空闲时的使能请求、空闲和故障状态下的复位请求开始前，
  任一年龄非有限或超过限值即拒绝：结果 `stage=device_offline`，阶段不变，不改写已记录的失败阶段
  （适配器据此判断复位是否仍可尝试），诊断新增 `offline_feedback`（第一个过期的接口名）。
  与其他用途已认领的同名状态接口只认领一次。
- 渲染器导出 `feedback_age_interfaces`：EtherCAT 每个主站一次 `ethercat_domain/process_data_age_ms`
  （距上次完整工作计数的时间，驱动断电时持续增长）；达妙头部各关节 `feedback_age_ms`。
- 组合器：参数来源 `exports` 汇总所选模块的同名导出；`head_only` 经作用域绑定取头部导出。
- 冻结 golden：arm、stationary、head 用例只新增 `feedback_age_interfaces`。

## 验证

见 PR。台架只做正常使能/失能回归；断电拒绝需现场断驱动电，另行授权。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 反馈年龄超过 100 ms 时，使能与故障复位均不开始，报 `device_offline`（用户 2026-11-03）。

## 遗留

- 公共错误码 `RT_DEVICE_OFFLINE`（1001）尚未进入锁定的 robot_interfaces，适配器暂按 `RT_ENABLE_FAILED`/`RT_RESET_FAILED` 返回，消息含 `stage=device_offline`。
- 使能后运行中掉线的处理不在本次范围。
