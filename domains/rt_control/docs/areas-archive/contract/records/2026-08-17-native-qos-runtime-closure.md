---
id: contract-20260817-02
area: contract
title: Native QoS 运行依赖闭包与 fail-closed 启动门禁
date: 2026-08-17
type: corrective
trigger: ELECTRI-103 / ELECTRI-96
commits: [fix/electri-103-native-qos-closure]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [/home/ar/rt-control-dev/log/native/rt-control-20260817-175729.log, /home/ar/rt-control-dev/log/build_2026-08-17_20-36-57, /home/ar/rt-control-dev/log/native/rt-control-20260817-204206.log, /home/ar/rt-control-dev/log/native/rt-control-20260817-205645.log, quality_gate 190/190, native launcher 48/48, io integration 15/15]
supersedes: []
related: [ELECTRI-96, ELECTRI-103, contract-20260817-01]
---

## 背景

目标机 Native 运行时在公共接口升级后出现 `rt_status_adapter`、`bms_node` 和
`vacuum_adapter` 同时退出，日志报
`ModuleNotFoundError: No module named 'robot_interfaces_qos'`。QoS profile 源码完整；实际
缺陷是历史手写 `colcon --packages-select` 构建只安装了消费者，未把
`robot_interfaces_qos` 纳入合并安装前缀，symlink-install 因而形成“新消费者源码 + 旧安装
闭包”。启动脚本只检查部分核心包，未在接触总线前阻断该状态。

## 改动

- 默认完整 Native build 继续使用 `--packages-up-to` 构建运行包闭包，并以
  `--cmake-clean-cache` 清除 vendor 路径迁移后遗留的 CMake source identity；构建完成后逐包
  执行 `ros2 pkg prefix`，再实例化 `control()`、`fast_state()`、`state()`、`latched()` 和
  `diagnostic()` 五个 QoS profile。
- `start`、`recover-power-loss` 和 `doctor` 在总线访问前检查公共接口、QoS、适配器、BMS、
  PLC、诊断和 bringup 的安装闭包，同时检查三个公共 Python 节点的已安装可执行文件。
- Native `READY` 不再只依据 DDS 中可发现的 service type。门禁要求实时调用
  `/controller_manager/list_controllers`，再验证 disabled 启动所需控制器状态与 EtherCAT OP。
  目标机完整初始化实测超过旧 90 秒预算，因此等待上限改为 150 秒；超时仍请求受控停机。
- 保留显式 `--packages-select` 作为开发者增量入口，但它不冒充完整运行闭包。需要部署或公共
  依赖发生变化时必须执行不带包选择参数的完整 build。

## 验证

- TDD RED 首先准确复现缺少依赖闭包检查和构建后验证；实现后 Native 启动器测试 48/48。
  旧 CMake cache 和旧 90 秒预算均先由新增回归约束复现失败，再完成修正。
- `./tools/bootstrap_native_dev.sh build` 完成 28 packages；构建后闭包检查 PASS，
  `robot_interfaces_qos` 安装前缀为 `/home/ar/rt-control-dev/install`，五个 profile 均可实例化。
- `tools/quality_gate.sh` 190/190，`tools/tests/test_rt_io_integration.py` 15/15；两个脚本
  `bash -n` 通过。当前环境没有 shellcheck，CI 仍需执行该检查。
- 受控 Native 启动后，三个原失败节点持续存活，BMS 连接 can1；四个公共状态 topic 各有
  publisher，`/vacuum/grip` 有 action server，`/vacuum/pump/set_enabled` 有正确 service。
  三轮修复后日志均无 `ModuleNotFoundError`。
- 最终验证没有调用 reset、enable、运动、PLC 输出、vacuum action 或电磁阀命令。启动 SSH
  断开后，目标机 logind 在约 60 秒后回收所属 session scope，`rt_control_start` 收到终止信号
  并执行既有受控停机；这不是适配器崩溃。最终 Native stopped，EtherCAT
  `Idle / Active: no`，16 个从站全部 PREOP。

本次实机操作经用户 2026-08-17 明确授权。开始时执行一次受控 `/rt/disable` 与 Native stop；
停止响应暴露既有 `right_joint2 fault_requires_reset`，但未执行 reset。后续启动均保持未使能。

## 结论与冻结事实

- F1: Native 完整运行构建必须安装全部 `runtime_packages`，并在构建后验证
  `robot_interfaces_qos` 五个命名 profile；手写增量选择不能作为部署闭包证据。
- F2: `start`、`recover-power-loss` 和 `doctor` 必须在硬件访问前 fail-closed 验证公共适配器
  依赖闭包；缺少 QoS ROS package、Python API 或已安装可执行文件时禁止启动。
- F3: Native 只有在 live controller-manager 调用、disabled 控制器状态和 EtherCAT OP 均通过
  后才能报告 READY；当前目标机启动预算为 150 秒。

## 遗留

- `right_joint2` 仍报告 `fault_requires_reset`，本记录未授权复位；该驱动状态不影响 QoS 依赖
  闭包结论，但阻止整机 readiness 为 healthy。
- 通过 SSH 远程启动时，目标机的 session cgroup 会在连接断开后回收后台进程；Native 工具
  不是持久化 daemon，操作终端必须保持连接，或后续另行设计受控 systemd service。
- 尚未重建 Docker 发布镜像，也未完成 Motion/Perception 等域同 SHA 的跨域 smoke。
