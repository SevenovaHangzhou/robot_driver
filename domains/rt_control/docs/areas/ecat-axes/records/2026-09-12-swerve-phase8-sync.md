---
id: ecat-axes-20260912-01
area: ecat-axes
title: 四舵轮 Phase 7.5/8 有限角规划、残差检测与 EtherCAT 后端同步
date: 2026-09-12
type: feature
trigger: 用户要求将已完成的四舵轮改动提交至 feature/damiao舵轮驱动
commits: ["feature/damiao舵轮驱动"]
env: native
risk: T1
writes: {reset: false, enable: false, motion: false, plc: false}
verified: PARTIAL
evidence:
  - src/rt_control/dm_swerve_driver/doc/development_plan.md
  - src/rt_control/dm_swerve_driver/doc/kinco_hardware_validation_status.md
supersedes: []
related: []
---

## 背景

按用户授权将独立开发仓库提交 `10ab59b` 的四舵轮包同步到指定功能分支。
主改动归属八轴 EtherCAT 适配，因此记录放在 ecat-axes；该包未接入既有十四轴生产
bringup，也未修改公共 Robot Model、域间接口或部署入口。任务号为用户开发计划
Phase 7.5/8，本次仅上传可评审源码，硬件参数和实机验收保持未验证。

## 改动

有限舵角规划保留 ±π 机械边界，删除输出层 PMAX 回中重选和静默位置钳位。
新增最小二乘逐轮速度残差、异常轮剔除与 odometry 协方差膨胀；新增 Kinco CSP/CSV、
可选 IgH 1.6 后端、BRT CANopen 编码器双源启动检查、主备切换与持久化标零。
当前方案为八台电机 EtherCAT、四外置编码器 CANopen；标定说明、现场手册和记录表
随包提交。包快照保持本地已验证实现；仅规范九个历史文件的末尾空行。

## 验证

独立开发仓库已有普通/Werror/ASan+UBSan/coverage CTest 各 29/29、源码覆盖率约 83.3%、
真实 IgH 1.6 用户库构建及节点链接证据，详见包内开发记录。
本次在 GitHub 分支独立检出执行包级干净 `colcon build`（Debug、`-Werror`）、
`colcon test` 和 `colcon test-result --verbose`：249 tests，0 errors，
0 failures，0 skipped；CTest 为 29/29。
`tools/quality_gate.sh` 通过：197 项仓库策略测试，门禁覆盖率 83%，架构/文件卫生、
EtherCAT 关停策略及 IPC 启动策略通过。本机未安装 ShellCheck，脚本明确提示由 CI 执行。
未新增大规模测试、未修改现有 CI 或硬件配置来绕过门禁。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本功能分支的四舵轮电机方案采用 EtherCAT CSP/CSV，保留四个 CANopen 外置绝对编码器。
- F2: 最小二乘残差用于差模异常轮剔除；共模一致打滑仍需上层跨传感器融合。
- F3: 本次上传不构成既有十四轴系统的生产集成或四舵轮实机验收。

## 遗留

FD/BRT 实际 PDO、DC/watchdog、背隙、标零、π 边界和拔线停车待台架验收；
CST 闭速度环尚未作为运行模式接入。目标主机实时性、整车部署和公共接口整合另行验证。
