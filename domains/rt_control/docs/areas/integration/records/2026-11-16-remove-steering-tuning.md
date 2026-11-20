---
id: integration-20261116-01
area: integration
title: 删除底盘转向参数现场调参（网页与 scope 启动）
date: 2026-11-16
type: removal
trigger: ELECTRI-211；用户 2026-11-16：网页拆分前去掉调参功能，底盘页只需要能控制底盘
commits: [refactor/remove-steering-tuning]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [ELECTRI-209]
---

## 背景

ELECTRI-127 起，操作网页底盘页可以修改转向速度、加速度、加加速度和对齐门限，写入
`/var/lib/rt-control/tuning/<machine_unit>/swerve_controller.yaml`，再由 `integ_bringup` 的
`scope_operator_tuning` 在每次 scope 启动时应用。网页要拆成独立仓库（ELECTRI-209），这条链路让网页
读写 rt-control 的部署产物与内部文件格式。用户决定删除该功能。

## 改动

- 网页：删除转向参数卡片、`steering_params`/`steering_defaults` 消息、`FieldTuning`、对控制器
  `set_parameters` 的调用和 `generated_dir` 节点参数；底盘页保留使能、失能、故障复位、遥控、停止。
- scope 启动：删除 `scope_operator_tuning` 步骤，运动控制器加载后直接启动生命周期控制器。
- 控制器默认值不变，`ctrl_swerve` 参数与上限校验不变。

## 验证

- 研华 `/var/lib/rt-control/tuning/Alfa_V3-Advantech-01/` 只有 2026-10-21 的 `history.jsonl`，没有生效的
  调参文件，所以删除后实际运动参数不变。
- 离线测试结果见 PR。未验证：新版本在研华上启动的实机运行。

## 结论与冻结事实

- F1: 转向运动参数只来自部署产物（机型策略），运行时不再从现场文件覆盖。

## 遗留

- `ctrl_swerve` 的 `max_steering_*_limit` 参数原本是给现场调参设的上限，现在只用来校验运行时参数修改；是否删除另议。
