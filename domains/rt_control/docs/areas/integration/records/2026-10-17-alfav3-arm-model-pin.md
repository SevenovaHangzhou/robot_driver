---
id: integration-20261017-02
area: integration
title: Robot Model 锁定切换为 Alfav3 双臂移动整机入口
date: 2026-10-17
type: decision
trigger: ELECTRI-127；用户 2026-10-17 要求把机械臂换成 Alfav3 导出模型（命名沿用现有、去掉中文“总装”改为 Alfav3）
commits: [feature/electri-127-bench-left-arm-scope]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [integration-20261017-01, ecat-axes-20261017-01]
---

## 背景

机械给出 2026-10-16 的 SolidWorks 双臂导出（17 个 link，关节 `L1_joint…`/`R1_joint…`，限位全为 0）。
按根契约 5.1，模型改在独立 robot_description 仓库完成：
SevenovaHangzhou/robot_description#4，分支 `feature/electri-127-alfav3-arms@4d4ad9a`（tree `35153ff`），
基于 `feature/electri-127-deployed-mobile-model@c49e8ed`。新增入口 `urdf/robot_alfav3_mobile.urdf`，旧入口保留。

## 改动

- `src/description/robot_description` 导入 4d4ad9a，tree `35153ff` 与上游一致；source-lock 更新。
- 机型 `robot_model.variants.mobile_proxy` 改用 `robot_alfav3_mobile.urdf`、`joint_limits_alfav3_mobile.yaml`、
  `robot_alfav3_mobile.srdf`；按用户决定删除命名姿态与初始位置，manifest schema 的 `initial_positions_file` 改为可选
  （组合器代码从不读取该文件）。
- 冻结 golden：arm 四例与 head 两例只有 `robot_description.xml`（内嵌模型）变化，按 integration-20261017-01 的方法改写，
  并保留文件末尾换行；`controllers.yaml` 与 stationary 两例不变。
- 门禁：`check_v3_branch_contract`、`test_tf_contract` 改锁 4d4ad9a 与新入口；`test_head_force` 改用新限位文件。

## 模型事实（详见 robot_description `config/alfav3_mobile_manifest.json`）

- 双臂零位 = Alfav3 导出零位 = 机械零位 = 驱动 2^18 计数零位（用户确认）。对应旧入口左臂 `[90,-15,180,0,90,0,0]°`、
  右臂 `[-90,15,-180,0,-90,0,0]°`；相对旧入口左 J3、右 J3/J4 轴向相反。旧入口的关节空间数值不适用。
- 限位按新零位对称：J1/J3/J5/J7 ±180°、J2 ±105°、J4 ±145°、J6 ±110°；与旧入口数值相同，故 envelope/容差策略无需改动。
- tool0 物理位姿与旧入口一致；新增 `left|right_tool_link`（法兰件）。

## 验证

- composer 212 passed；bringup scope/entry 与 composer 合计 225 passed；robot_description 23 passed；
  `tools/tests` 247 passed / 13 skipped；`check_v3_branch_contract`、`rt_control_layer_gate`、`repository_gate` PASS；
  `validate_machine_profile --all`（Alfa_V3-01 与 Alfa_V3-bench-01）通过；ruff 0.11.13 通过。

## 遗留

- `machines/Alfa_V3/arms.yaml` 的 14 轴方向（`user_provisional_plus_one`）与 `units/Alfa_V3-01/arms_zero.yaml` 零位是按旧模型记录的，
  对新模型不成立，需在整机上重新确认。
- 运控/感知域使用旧入口的位姿需迁移（用户决定本次不发通知）。
