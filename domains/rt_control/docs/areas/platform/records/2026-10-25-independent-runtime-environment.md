---
id: platform-20261025-01
area: platform
title: 独立实机环境入口与两套仓库组合
date: 2026-10-25
type: feature
trigger: 用户要求先备份，仿真使用导航加仿真驱动，实机使用导航加实机驱动
commits: []
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /home/robot/wheel_nav/deployment-artifacts/20261025/real-env-and-empty-graph.log
  - /home/robot/wheel_nav/deployment-artifacts/20261025/driver-quality-gate.log
supersedes: []
related: [BQ-145, BQ-150]
---

## 背景

用户要求先备份再拆分部署依赖，实机仓库不保存仿真数据，仿真能不依赖实机仓库构建运行。
范围为构建与环境入口；没有新的控制策略、硬件参数或公开接口变更。

## 改动

新增 tools/deployment_env.bash，按仓库相对位置或显式 ROBOT_DRIVER_INSTALL 加载本地安装，
支持独立 Ruckig/Livox SDK 前缀。无硬件启动、使能、模式切换或宿主配置动作。
原 driver_env.bash 保留，避免破坏现有使用习惯。仿真快照的导入发生在仿真仓库，
本仓库未加入 Gazebo、world、接触/真值插件或录包。

## 验证

bash -n 通过；干净环境 source 后 swerve_driver 解析至实机 install，导航库的
libruckig.so 解析至 wheel_driver/deps，未加载 Gazebo。空 DDS 域只读探测正确报告
缺少轮反馈、IMU、模式服务与参数，没有把无数据状态判为就绪。
最初质量门禁被缺少 jsonschema 和 Git 元数据阻塞。提交准备阶段已在独立 Git 检出中
恢复上游历史，并使用隔离的 jsonschema 3.2.0 重新执行 tools/quality_gate.sh：
架构/文件检查、248 项工具测试、覆盖率门槛和 EtherCAT 停机静态策略通过，13 项跳过。
证据为 /home/robot/wheel_nav/release-preparation/20261025/push/driver-quality-gate.log。
clang-format、ruff、shellcheck 本机不可用，对应强制样式检查仍由 CI 执行；
不将本地门禁通过等同于正式发布或实机验收。
新隔离工作区逐包构建 6 个舵轮核心及直接依赖包通过，证据为
`/home/robot/wheel_nav/deployment-artifacts/20261025/driver-core-build.log`；
没有构建完整硬件 bringup。未执行整机部署、总线操作或真实运动。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 实机环境入口仅 source ROS 与实机安装，不 source 仿真安装。
- F2: 仿真生产核心快照在仿真仓库维护，实机驱动仍为算法更新来源。
- F3: 本轮不改变 deps.repos 公共接口候选 pin、IMU draft 配置或任何硬件标定。

## 遗留

实机完整构建、公共接口同版本验证、现场标定与 BQ-145/BQ-150 准入仍待现场核对。
导航 deployment/README.md 记录依赖附件和隔离构建工具；driver-core 只构建核心，
不能替代现有整机 bootstrap/doctor/准入及 runtime 入口。
