---
id: release-deploy-20260918-01
area: release-deploy
title: V3 专用分支、模型与 fail-closed 运行边界
date: 2026-09-18
type: feature
trigger: 用户要求从 main 建立只供三代机使用的 v3 分支，并按新总线/故障裁决推进迁移
commits: [v3]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - "robot_description 源包：27 tests、suction/gripper check_urdf PASS"
  - "V3 machine/profile/launch：63 tests PASS"
  - "rt_diagnostics：88 tests PASS"
  - "V3 默认安装入口：validation-only exit 0"
  - "tools/quality_gate.sh：286 passed、13 skipped、策略覆盖率 83%"
supersedes: [release-deploy-20260907-01]
related: [BQ-145, BQ-146, BQ-147, BQ-149, BQ-150, BQ-151, ELECTRI-126]
---

## 背景

main 同时保存二代生产入口和三代 draft，默认 launch、14 轴 controller、X503、履带
CANopen/diff-drive 仍绑定二代。用户决定建立 V3 专用长期分支，并裁决分模块总线准入、
完整物理配置全部使能及悬挂/底盘/机械臂故障依赖。

## 改动

- 从 `main@0ed8611` 建立 `v3`；导入并锁定
  `robot_description/robot_v3_suction_chassis@17f5bdc`，保留 suction/gripper 两套模型，
  硬件清单显式绑定 gripper 变体及左右 J1..J7/移动夹爪关节。
- alfa_v3 新增单轴 EtherCAT CSP `active_suspension` draft；只复用同毅/XMC 协议身份，
  不复制 Updown 机械换算。机械比例、方向、零位、行程安全和实机 PDO 保持 TBD。
- machine profile 派生 EtherCAT/CANopen/达妙 `required/not_required`；full_robot 只允许
  full scope。主动悬挂 fault 停止并阻断底盘，底盘 fault 停止机械臂，记录于 BQ-151。
- `rt_diagnostics` 允许零 CANopen 节点并发布 OK/not-required summary；配置节点时原有
  断线、STALE、EMCY 与 CiA402 检查不变。
- V3 安装产物只含 module validation 和双臂 enable-only；二代生产 launch、controllers、
  X503 PREOP 和 diff-drive 不安装。默认 `rt_control_start` 只做 validation。
- bootstrap、Docker 和 CI 显式登记 V3 模块闭包；CI 用 V3 来源/运行边界门禁取代二代
  diff_legacy 比较。

## 验证

- 权威 description 源提交隔离构建通过，27 项测试、两套 Xacro 展开和 check_urdf 通过。
- 导入后的 description 构建/测试结果相同；根级 V3 模型合同 5 项通过。
- machine profile 先建立 RED，完成后 profile/launch 共 63 项通过；非法故障依赖 fail closed。
- rt_diagnostics 构建通过，88 项测试、0 failure；status adapter 13 项通过。
- rt_control_bringup 构建通过，78 项测试、0 failure；安装边界断言通过。
- 安装后的默认 `rt_control_start` 在隔离 ROS domain 只输出 V3 arms_only validation，exit 0。
- `tools/quality_gate.sh`：286 passed、13 skipped，策略覆盖率 83%；本机无 ShellCheck，
  远端 CI 强制执行。

未运行完整干净 vendor CI、Docker 构建或整机 Mock；最终数字以推送前复验为准。未访问
EtherCAT/CAN 实机，未 reset、enable、运动、SDO write 或 PLC 输出。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: `v3` 的默认和安装运行面只支持 alfa_v3；二代资产可留作源码对照但不得进入安装闭包。
- F2: arms_only 未配置 CANopen/达妙时为 not_required；配置后异常才是 fault。
- F3: full_robot 只允许 full scope，活动执行器集合等于物理执行器集合。
- F4: 故障依赖按 BQ-151 配置；当前只完成合同和诊断语义，运行时停车执行仍待 controller
  组合完成后验证。
- F5: 当前 V3 整机保持 draft/fail-closed；分支存在不代表实机 runtime-ready。

## 遗留

完成 ZeroErr 正式 CSP/PP profile、主动悬挂机械/驱动事实、步科 PDO/SDO、外置编码器
EDS/Node ID、Updown 环位、头部 CAN 和蓝点安装标定。随后生成 scope-specific controller/
enable_manager 配置，实现 BQ-151 的运行时停车传播，并执行分模块 Mock、无运动使能与低速验收。
