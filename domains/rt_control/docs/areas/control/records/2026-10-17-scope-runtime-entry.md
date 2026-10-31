---
id: chassis-control-20261017-05
area: chassis-control
title: 通用 scope 运行入口、总线锁、分步硬件激活与有序收尾
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 同意先做底盘 scope 的通用运行入口
commits: [feature/electri-127-scope-runtime]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261017-04]
---

## 背景

研华部署的底盘入口、监督、预检、激活与收尾全部是底盘专用代码。本次把它们改为由组合结果驱动的通用实现。

## 改动

- composer：scope `staged_hardware_activation: true` 时，所有硬件系统初始为 unconfigured；
  激活顺序为“只提供状态的系统在前、执行器系统在后”；输出 `lifecycle.yaml`
  （激活顺序、运动/生命周期控制器、资源声明、总线锁 `ethercat-master-<id>` / `can-<iface>`）。
  未声明该选项的 scope 输出不变（冻结 golden 不变）。chassis_only 已声明。
- rt_control_bringup：`rt_control_scope_runtime.launch.py`：组合 → 真实路径先取总线锁 → 各协议包预检
  → controller_manager → `scope_hardware_start` 按顺序激活 → 加载运动控制器（inactive）→ 生命周期控制器
  → 公共使能适配器。`rt_control_start --scope <name>` 选择该入口，退出时运行 `scope_shutdown`：
  停运动控制器 → `/rt/disable` → 停生命周期控制器 → 硬件按激活逆序 inactive、unconfigured；
  控制器未停下时不动硬件。
- 预检拆到协议包：Kinco 真实路径要求主站空闲、从站数/身份/PREOP、驱动器未使能未故障，
  驱动器已保存的 PDO 分配与映射必须等于 profile 布局，转向起始位置在适配器范围内；
  CAN 要求接口 UP 且 ERROR-ACTIVE。均为只读命令。

## 验证

- `pytest rt_control_bringup/test/test_scope_lifecycle.py test_composer_entries.py`：9 passed。
- `pytest robot_hw_ethercat/test/*.py`：55 passed（含已保存 PDO 不符拒绝用例）；
  `robot_hw_canopen/test/test_canopen_preflight.py`：4 passed。
- composer 全部测试：191 passed，1 failed（安装布局 CLI 用例，环境问题，主线同样失败）。
- `tools/tests`：247 passed、13 skipped；quality_gate PASS。

未启动 ROS 图、未连接总线。

## 遗留

- 研华台架的单机部署数据（CAN 接口、主机身份）尚未建立，真实组合仍被 verified/背隙阻塞。
- 实时线程绑核检查与 Web 能力识别见后续变更。
