---
id: ecat-axes-20260912-04
area: ecat-axes
title: 三代机双臂物理轴序与 CSP/PP 分配确认
date: 2026-09-12
type: decision
trigger: ELECTRI-118，用户确认两臂均沿总线按 J1..J7 后接夹爪
commits: []
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-146, ecat-axes-20260912-03]
---

## 背景

只读扫描已经确定左右臂分支范围，用户进一步确认两条链的物理关节顺序。
该确认解决了物理关节到总线位置的对应，不推定独立 Robot Model 的正式 joint 名称。

## 改动

新增 robot_hw_ethercat/config/machines/alfa_v3_arms_only.draft.yaml，归集主/子分支器与
零差身份、非运动响应设备、两臂端口和逐轴位置/目标模式。bringup manifest 通过
hardware_identity 文件引用关联硬件所有者，并将 arms_only 的可见位置由 TBD 改为 0..17。
其他物理组合仍待各自扫描。没有修改生产 variant、控制器或 enable_manager 资源。

## 验证

采用 python-testing 的先失败后通过流程：补充链内顺序、模式分配和跨配置一致性检查，
初次运行五项预期失败（布局仍为 TBD、物理清单缺失）；补齐配置后聚焦配置测试通过。
检查同时要求：18 位置完整无重复、16 运动轴与 2 响应设备分开、14 CSP+2 PP 与 manifest
一致、未确认参数仍为 TBD，且运行准入继续拒绝。实机模式回读与运动没有执行。

本地与目标机的 machine_profile/validator/launch 共 47 项测试均通过；本地质量门禁
214 项通过、受门禁代码覆盖率 83%（ShellCheck 本机未安装）。目标机增量构建
robot_hw_ethercat 与 rt_control_bringup 两包成功；安装副本哈希、部署校验脚本及
静态 launch 通过，输出 18 个 ring position、16 actuators、CSP=14、PP=2、status=draft。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 左臂 X2 的 J1..J7 对应绝对位置 1..7、夹爪对应 8；右臂 X3 的 J1..J7 对应
  9..15、夹爪对应 16。顺序由用户确认，位置依据此前只读扫描。
- F2: J1..J7 的目标模式为 CSP(8)，夹爪目标模式为 PP(1)，共 14+2 运动轴；
  0/17 的分支器不属于运动轴，不分配 CiA402 模式或使能资源。
- F3: 物理 J 编号与正式 Robot Model 名称分开保存；PDO/SDO、机械参数与运行描述尚未完成，
  该物理清单为 draft，不由生产 launch 加载。

## 遗留

下一步核对两类零差 PDO/SDO，再补 Robot Model、14 CSP 与两 PP 控制器、使能资源和
运行生命周期。分支器状态/DC、设备名称版本差异及目标 IgH PDO 保留补丁仍待验证。
