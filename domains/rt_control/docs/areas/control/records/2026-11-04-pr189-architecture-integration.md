---
id: control-20261104-01
area: control
title: PR 189 远程功能迁移到 main 包架构
date: 2026-11-04
type: fix
trigger: 用户要求解决 PR 189 冲突，保留远程实际功能并迁移到 main 新架构；禁止修改 main
commits: [feature/rt-control-v3-chassis-integration-20261104]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [chassis-control-20261102-02, platform-20261104-01]
---

## 背景

PR 189 的工控机功能分支与 main 的包重命名、共享基础设施和配置归属调整冲突。
用户明确选择保留远程相对运动、NAV/OPERATION、适配器及日志功能，并迁移到新架构。
合并方向为 main 到功能分支，main 工作树和远端分支保持原状。

## 改动

- 底盘功能进入 `ctrl_swerve`，使能策略进入 `ctrl_enable_manager` 的非实时配置单元；
  仅运行中的指定驱动轴接受 CSP/CSV 交接，启动与控制器激活仍要求配置的固定模式。
- composer、bringup、Web 操作台与适配器使用 `integ_*`；外设使用 `periph_*`。
  设备通信复用 `infra_socketcan` 和 `infra_modbus`，诊断保留 main 的结构。
- 记录插件改为依赖 `infra_shm_ring`，移除重复环实现；保持 Runtime-LOG 二进制布局和读者压力测试。
- Kinco 的显式 Shutdown/Ready 准入策略迁移到返回诊断 findings 的预检接口；
  仍检查 Idle/PREOP、连续无故障状态、PDO 与位置窗口，不增加任何总线写入。
- 保留已配置 LPMS 六轴帧映射、安装方向和协方差，独立 launch 默认只做静态验证。
  生产者仍发布 `/imu/data`；底盘里程计保持远程消费者使用的 `/wheel/odom`。
- 删除已迁移目录的残留入口和失效测试注册，避免重复启动 robot_state_publisher。
- bringup 明确接受 controller_manager 2.53.1、2.54.2 和 2.54.3：使用的上游节点源码完全一致，
  SHA-256 和出处记录在包 README，其他版本仍拒绝。

## 验证

隔离目录 `/tmp/codex-pr189-validation`，ROS 2 Humble；不启动生产服务。

- `tools/quality_gate.sh`：PASS，231 passed、14 skipped。
- `PYTHONPATH=tools/runtime_log python3 -m pytest tools/runtime_log/tests -q`：PASS，139 passed。
- Web 包 pytest：PASS，304 passed。
- Shutdown/Ready 只读准入回归：PASS，30 passed。
- ROS 完整构建：PASS，31 包；修复后重新构建并复测受影响包。
- `colcon test-result --verbose`：PASS，1377 tests，0 errors，0 failures，1 skipped。
- GitHub CI：结果见 PR 189 最新提交的检查记录。
- 首次 GitHub governance 通过；完整构建恢复了旧 pin 的 vendor 缓存，被冻结检查正确拒绝。
  vendor 缓存改为仅按 `deps.repos` 精确命中，不再跨 pin 恢复，缓存命名空间升级以排除旧污染。
- GitHub 的 ROS apt 安装 2.54.3，初次编译被版本保护拦住；逐字节复核官方节点源码与冻结文件
  SHA-256 一致后明确允许该版本；本机 2.54.2 与 GitHub 2.54.3 的构建结果分别记录。
- GitHub 2.54.3 完整构建通过，ROS 汇总 2431 tests、0 errors、0 failures、24 skipped。
  Runtime-LOG 步骤随后因基础镜像缺少 pip 失败；该步骤显式安装 pip 与 pytest 后再安装锁定依赖。
- Runtime-LOG 在 root CI 中执行到权限拒绝用例时失败：root 可读取 chmod(0) 文件。
  测试改为在指定进程文件的读取边界注入 PermissionError，仍验证权限拒绝与进程退出的区别，
  不跳过用例、不改变生产读取逻辑。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 原远程功能迁移到 main 的包架构，公共接口 pin 和 Robot Model 沿用 main；域内相对运动接口保留。
- F2: Runtime-LOG 写入与使能 trace 共用唯一 `infra_shm_ring` 内存布局。
- F3: 软件、mock 和 CI 验证不构成实机准入、模式切换或运动验收。

## 遗留

真实设备通信、故障停机、继电器输出和运动验证未执行，需独立现场授权与验收。
