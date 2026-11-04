---
id: chassis-control-20261025-01
area: chassis-control
title: 研华实机导航反馈增量部署与对齐默认保留
date: 2026-10-25
type: commissioning
trigger: 用户明确要求先备份、有序停止驱动、增量部署后恢复，网页保留对齐模式
commits: []
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /home/sev_v3/Robot/deployments/nav-driver-20261025/README.md
  - /home/sev_v3/Robot/deployments/nav-driver-20261025/driver-install-transaction.json
  - /home/sev_v3/Robot/deployments/nav-driver-20261025/runtime-verification.json
supersedes: []
related: [ELECTRI-127]
---

## 背景

按用户授权，将已发布的导航集成候选用于现有研华实机；保留现场未提交源码、配置与网页。

## 改动

以现场 swerve_driver 为基础合入 1f69f32 的专用 SI 关节反馈、纯平移分支连续性修复及库环境安装。默认 drive_mode 保持 align_first，不使用该提交的连续默认。保留现场源码 /wheel/odom 约定，因此运行发布地址由原安装 /swerve_controller/odom 更新为 /wheel/odom，导航部署入口同步。IMU 融合仍关闭，生成硬件参数、网页代码及启动脚本未改。

## 验证

备份完整归档并校验 SHA256。串行构建 swerve_driver，通过包测试，colcon 汇总 180 项结果、零失败。全仓质量检查被部署前已存在且本次未修改的 lpms_nav3_can/CMakeLists.txt 编译警告基线缺失挡住。

沿 chassis.sh stop --force 执行有序失能停机，日志确认控制器停用、两硬件组件清理；安装后 chassis.sh start，8 个 EtherCAT 从站，RT_SCOPE_HARDWARE_ACTIVE_DISABLED。网页直接 HTTP 200，运行态 IDLE、align_first=true、fault_stop=false，swerve inactive。新反馈和里程计发布端点存在，但未使能期间无数据。没有发送运动指令、导航目标或执行实机使能。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本次现场部署保留对齐启动默认；导航 FollowMPPI 仅经停稳模式服务进入连续模式。
- F2: 现场新轮式里程计入口为 /wheel/odom；未启用 IMU 内部融合。
- F3: 实机保持未使能；数据流、运动和到点验收尚未执行。

## 遗留

现场标定按原文件保留，未裁决 IMU 补偿注释/实现差异。导航单独完整构建与测试记录见部署 README。全仓 IMU CMake 门禁遗留，不能声称全仓质量通过。逐文件回退入口为部署目录 install_driver.py restore，执行前须有序停止运行栈。
