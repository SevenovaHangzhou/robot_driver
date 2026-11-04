---
id: integration-20261020-01
area: integration
title: 研华 V3 底盘与网页现场启动说明
date: 2026-10-20
type: investigation
trigger: 用户要求将已验证的底盘与网页启动命令写入对应包 README
commits: []
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /home/sev_v3/Robot/rt-control/v3/runs/chassis-20261020T090918Z/launch.log
supersedes: []
related: []
---

## 背景

主站启动后立即运行底盘脚本时，链路与八台从站尚未枚举完成，脚本仍继续尝试后台启动；
网页也不会随底盘脚本自动启动。

## 改动

新增 `rt_control_bringup/README.md`，记录先确认 EtherCAT `Link: UP`、`Slaves: 8`，
再运行 `v3-restart.sh`，随后检查启动日志并启动独立网页服务的命令。没有修改运行代码。

## 验证

- 2026-10-20 现场运行日志显示八台 EtherCAT 从站 OP、完整工作计数、
  `RT_SCOPE_HARDWARE_ACTIVE_DISABLED` 和 `enable_manager` 已配置并激活。
- 同次会话本机直连网页 `192.168.100.28:18110` 返回 HTTP 200。
- 本次文档修改通过 `git diff --check` 与 `tools/quality_gate.sh`
  （247 passed、13 skipped）；未再次启停总线或底盘。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前现场入口须等待主站 `Link: UP`、`Slaves: 8` 后再运行 `v3-restart.sh`。
- F2: 网页是独立的临时用户服务，端口为 18110；HTTP 200 不能替代底盘就绪检查。

## 遗留

`v3-restart.sh` 对从站缺席和后台启动失败仍可能返回成功；脚本级失败传播需另行修复和验证。
