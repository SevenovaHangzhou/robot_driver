---
id: hw-config-20261026-01
area: hw-config
title: 独立右臂台架 scope 与 18121 入口
date: 2026-10-26
type: feature
trigger: 用户要求仿照双臂和单臂创建右臂 scope，并添加 arm start 选项供 18121 使用；本轮确认仅右臂接 X3，不接力传感器
commits: [feature/right-arm-scope-20261026]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: []
---

## 背景

> 已纠正：本记录的 9 从站配置依据预期接线，未核对实际总线。后续实扫为 18 从站，
> 1..7 实为左臂。当前配置与验证结论以
> [右臂物理映射纠正](2026-10-26-right-arm-physical-mapping-correction.md) 为准。

工控机现有 left/dual 运维入口，缺少独立右臂选择。本轮用户明确选择仅接右臂，
保留主耦合器 X3 分支，不接末端蓝点力传感器。旧的双臂位置 9..15 不能用于该接线。

## 改动

- 注册 `bench_right_arm` 模块、physical profile、普通运动 scope 与
  `bench_right_arm_stationary_enable` scope，沿用共享组合器、EtherCAT owner 和启停链。
- 右臂 J1..J7 使用位置 1..7，主/子耦合器为 0/8；X3 对应主 ESC port 1。
  零点、方向、CSP profile 和 J3/J4 单圈策略来自工控机既有双臂配置及其确认记录，
  仅重新编号，不推测新标定或增加扭矩 PDO。
- 添加右臂七轴使能/运动策略和 `right_arm_jtc`，档位和容差数值沿用既有七轴策略。
  发布 `arm_joint_trajectory` 能力到 `/right_arm_jtc/follow_joint_trajectory`。
- 工控机 `tools-local/arm` 增加 `arm start right` / `arm restart right`，沿用 18121，
  guard 使用 `right_arm_jtc`，位姿独立保存到 `console_poses_right.json`。
  右臂不扩展双臂现有自动复位策略，已有手工复位入口保持可用。
- 工控机原文件备份到工作区 `backups/right-arm-scope-20261026`。
  两端 manifest/controllers 分别增量修改，保留工控机既有双臂、蓝点和网页改动；
  本地未引入远端无关差异。

## 验证

- 操作台已有测试：38 passed（假驱动/本地 HTTP，不连接实机）。
- 扩展已有七轴轨迹能力测试覆盖左右选择，并核对 physical profile 与生成拓扑一致；
  完整组合器测试：226 passed。初次以源码 schema 路径运行时，安装 CLI 用例因路径不符失败；
  构建并使用安装后的 schema 路径后，全部通过。
- 工控机临时 overlay：普通/原位两种 scope × mock/真实配置，共四组组合和产物哈希
  往返校验通过，核对七轴受管集合、单轴批次、9 个 responder、X3 分支、零点与方向。
- `bash -n tools-local/arm` 与隔离调用 `select_scope right` 通过，核对 scope、JTC 和位姿路径，
  不执行启动函数。
- 工控机：`colcon build --base-paths robot_driver/src --merge-install --symlink-install
  --packages-select rt_control_machine_config enable_manager rolling_trajectory_controller`：3 包通过。
- 工控机：相同包选择的 `colcon test --merge-install` 完成；`colcon test-result` 分别报告
  enable_manager 87 tests / 0 errors / 0 failures，rolling_trajectory_controller
  127 tests / 0 errors / 0 failures / 1 skipped。
- 本地隔离目录 `colcon build --packages-select rt_control_machine_config rt_control_composer`：2 包通过。
- 安装后的 `compose_artifact --physical-profile bench_right_arm --control-scope bench_right_arm
  --real` 生成 11 个文件，`load_artifact` 往返通过；6 项源文件与安装文件字节一致。
- 本地 `tools/quality_gate.sh`：248 passed、13 skipped，门禁覆盖率 86%。
  缺少本地 ruff、clang-format 和 shellcheck，门禁如实提示；未宣称这些检查已执行。
  本地依赖来自既有 `/tmp/head-sensor-tf-20261024/deps`，未加入源码或运行默认值。

未执行新接线实扫、硬件生命周期、使能和运动。当前运行 scope 和记忆选择保持 left，
本次不重启 18121，不切换总线。`--real` 仅为离线组合选择，不代表访问了设备。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 独立右臂台架仅接 X3、无末端力传感器/夹爪，声明 9 个 responder；实际枚举待现场验证。
- F2: `bench_right_arm` 仅管理 `right_joint1..7`，轨迹入口为 `right_arm_jtc`，不含左臂控制资源。
- F3: `arm start right` 使用原 18121 操作台与独立右臂位姿文件；启动不使能，不增加自动故障复位。

## 遗留

现场停旧栈后按 F1 接线，核对从站身份/顺序，再按既有授权流程验证启动、使能和运动。
本轮未提交或推送 Git；已有用户改动全部保留。
