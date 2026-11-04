---
id: lifecycle-20260912-01
area: lifecycle
title: 原位使能入口和无轨迹控制器的 enable_manager 策略
date: 2026-09-12
type: feature
trigger: ELECTRI-118，用户要求三代机 14 CSP 加 2 PP 真实使能
commits: []
env: native
risk: T3
writes: {reset: yes, enable: yes, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [ecat-axes-20260912-11]
---

## 背景

使能测试不需要轨迹或夹爪开合命令，但仍需复用现有使能、失败回滚和有序停机。
现有 enable_manager 必须切换 JTC，因此加入明确的原位策略，不用伪造控制器替代 JTC。

## 改动

- enable_only 默认 false；true 时要求 jtc_name 为空，仅省略轨迹控制器切换，保留
  状态机、故障、超时、抢占、回滚和失能。参数成功 configure 后不可更改。
- disable_terminal_policy 默认 joint_list，保留旧机显式列表约束；显式
  switch_on_disabled 表示所有轴统一要求该终态，不允许同时声明关节例外。
  使用局部准入掩码，避免失败 configure 污染后续显式参数检查。
- 新 rt_control_enable_only.launch.py，默认 Mock；真实模式先只读核验 18 从站身份、
  X2/X3 和 Fault/控制字/错误码条件。用硬件包清单和零位生成原位控制描述，保留
  ecat_arms system 和 controller_manager；无 Robot Model/TF 发布，无新公共接口。
- rt_control_start --enable-only 选择该 launch，退出仍走原有 rt_disable_once：
  失能、停止 controllers、deactivate hardware 后转发信号。未改变默认生产入口。

## 验证

- TDD 先复现无 JTC 不能 configure、ROS YAML 空数组不能作为参数、失败 configure
  污染终态参数准入；新增测试分别覆盖完整 enable/disable 服务、显式终态和失败重配。
- Mock 成功加载三个 controllers，退出返回 already_disabled、controllers quiesced、
  hardware inactive，manager 退出 0。最初 YAML alias/空数组失败均已纠正，未用于实机。
- 真机启动、原位使能与停机详见 ecat-axes-20260912-11；不存在轨迹或 PP Action 命令。
  关闭测试栈后，原生产服务和部署软链未变。
- 最终 enable_manager 套件 62 条测试记录全部通过，包含最后的失败重配修正；
  硬件包 88 条测试记录与 Python 配置/launch 52 项通过。
- 最终本地质量门禁 214 项通过、受门禁覆盖率 83%；ShellCheck 本机未安装。

本次实机操作经现场授权，操作人：Codex（用户授权），确认短语入口：用户明确要求
“继续做14个CSP和2个PP模式的使能”；同一轮完成后的 configure 失败路径修正经单测验证，
不另行重复实机使能。

## 结论与冻结事实

- F1: 现有生命周期可管理只开放 control_word 的 16 轴原位测试，实际使能/失能均成功。
- F2: 标准运动栈仍采用默认 enable_only=false；此测试入口不能作为运动准入豁免。
- F3: 真实测试过程经校验的 update CPU 为 14/FIFO80，内核 EtherCAT-OP 线程保持现场
  原有配置，未因本次测试修改内核或系统服务。

## 遗留

开发环境复用入口（只有原位使能能力）：在已加载隔离环境的终端运行
`ros2 run rt_control_bringup rt_control_start --enable-only use_mock_hardware:=false`。
本机启动线程应使用记录中的 housekeeping CPU 集合；运行后先用既有
rt_control_thread_affinity.py 核对/绑定 CPU14，再检查 mode、WC、preload 和 drive 状态，
按需 /rt/reset_fault 后 /rt/enable。停止使用 Ctrl-C，由标准退出门负责失能和总线收尾。
运行时生成的 profile 保留在 /tmp/alfa-v3-enable-* 供审查，不回写权威模板。
所有后续运动、完整生产封装、长期时序和故障注入验证仍单独完成。
