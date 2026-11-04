---
id: chassis-control-20261029-99
area: chassis-control
title: 指令新鲜度取时竞态与单周期停轮诊断
date: 2026-10-29
type: fix
trigger: 用户要求修复 prediction_detail_20261029_152313 剩余抖动并在公共机备份部署
commits: []
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 变更与验收

command/IMU 的 RT 快照先读取，再采集用于新鲜度判断的单调时间，避免新接收数据被旧 now 误判。runtime 二次 IMU 快照也使用后采的时间。真实过期、generation、权限、反馈故障不放松。

沿用 ~/diagnostics 增加 execution_transition 事件项：RT 固定容量无阻塞 SPSC 队列保存单周期状态/分支/准入原因，非实时线程格式化发布。满队列丢弃新记录并累计 dropped，不能静默丢失。

现场新增 IMU 话题配置及并行生命周期/反馈年龄修复已保留，未覆盖其他包。新导航计划换向提前减速，驱动 90° 门控和紧急停止路径未改成舒适慢刹。

构建与测试日志、多人变更校验、未使能启动状态、回退说明见 navigation/maintenance/jitter-stop-fix-20261029；停止底盘使用原 chassis.sh 有序失能入口。未发送运动，未将 mock/编译或记录状态重放作为实机导航验收。
