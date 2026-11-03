---
id: arm-control-20261102-01
area: arm-control
title: 当前装配重力前馈控制器离线草案
date: 2026-11-02
type: feature
trigger: 用户要求采用 static_gravity 添加重力前馈；D-004
commits: [codex/gravity-feedforward]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [D-004]
---

## 背景

用户选择当前两侧工具装配，提供左右 J1～J7 的具体 eRob 型号、两份厂家手册，并确认
全部电机为 MC2。相关聊天 `01a163fa-7bf2-7e22-9843-5f0e9b44d025` 的任务是腕部六维力
参考下的人工施力标定：保持位置控制，增加静态重力前馈，暂不叠加摩擦补偿。

## 改动

- 新增 draft `ctrl_gravity_feedforward`。权威 URDF 在 configure 时解析，KDL 计算
  `dU/dq`，固定工具及其它固定旁支质量并入所属链节；无复制的机械臂几何常量。
- 控制器仅申请 additive `effort`，不申请 position/control_word。参数必须显式提供
  力矩限幅、变化率、反馈时效及硬件健康接口；没有实机安全数值默认值。
- 激活检查完整资源、实际位置、CiA402 Operation Enabled 和反馈健康；更新失效时
  全组输出缓冲归零、返回 ERROR 并锁定，停用/error/cleanup/shutdown 清零。
  断链后是否送达零值仍由既有硬件停机/保护链决定，不能用控制器清零替代现场验证。
- 型号与手册证据保存在 hwcfg_machines/evidence/arm_gravity_feedforward_20261102.yaml。
  左/右 J4 的 BHS/BHM 差异保留，90 V6、70 V5 的跨版本 Kt 仍不外推。
- 未加入生产 scope，未实现 `effort → 0x60B2` 的真实 PDO 接线，未修改现有驱动映射。
  当前交付是离线候选，不是可直接上电使用的重力前馈。

## 验证

以下为原工作区离线草稿的验证；迁入最新主线后的复验见 `2026-11-03-gravity-feedforward-switch.md`。

按根 AGENTS.md 新分级：达到第 2 级的本包构建与自动化测试；未达到第 3～7 级。
本记录 frontmatter 的 T0 沿用历史模板，不能解释为实机或 mock 启动。

- 隔离目录 `/tmp/gravity-feedforward-review` 中构建 semantic_components 与新控制器包。
- 新控制器 12 项 C++ 测试通过；colcon 汇总含 CTest 容器计数为 13，0 失败/跳过。
- 重新编译用户提供的 `static_gravity` 为对照库；零位加 1000 个随机姿态、3 个重力向量、
  2 条臂链，共 6006 组单臂计算，最大绝对差 `1.5631940186722204e-13 N·m`。
- 比较使用当前 driver 的权威 URDF 构建副本，SHA-256
  `cc60c01753bf8bc7b95cdf21b1972a701e58a3a71596dc016820567239573119`；
  对照参数签名 `4cc1aaca4d64b74588496bb9473c4ae995bd1a4f5fa810408bd28a12495d888a`。
- 基座重力必须转换：对照库输入世界重力，而控制器输入 `R_world_base^T * g_world`。
  arm_carriage 的安装旋转会改变重力方向，不能直接把世界 `[0,0,-9.81]` 当基座重力。
- 仓库结构/文件门禁及 diff whitespace 检查通过。全量 quality_gate 初次运行因缺少
  `jsonschema` 在依赖检查阶段退出，不能宣称全仓质量门禁通过。
  本机还缺 clang-format 14 与 ruff；style_gate 仅返回缺工具提示，不作为格式验证通过。
  尝试在临时目录获取检查工具时网络超时；未安装系统包或修改宿主配置。

重现数值对照使用新包 `test/compare_static_gravity.py`，显式传入 preview 可执行文件、
对照库、parameters.json、当前 URDF、`--base arm_carriage --tips left_link7 right_link7`。
验证的是计算与既有模型的一致性，不验证目标机实时性能、真实电流标度、J6 传动关系、
基座实际姿态、安装偏置是否已经应用或现场停机。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 用户选择含当前工具的 static_gravity 装配；工具自重不可重复作为 payload 添加。
- F2: 用户确认左右所列电机全部 MC2；不能由 MC2 单独推断未列明 V 版本的 Kt。
- F3: 仍须补齐逐轴驱动电流/输出标度与符号、J6 映射、持续前馈限幅和变化率，再完成
  协议映射与启动组合。URDF effort 峰值及手册典型效率不作为猜测性默认值。
