---
id: integration-20261017-01
area: integration
title: Robot Model 锁定切换为研华部署移动模型并移出 PP 夹爪
date: 2026-10-17
type: decision
trigger: ELECTRI-127；用户 2026-10-17 选择方案 B（接受代理模型、放弃夹爪入口、修改主线门禁）
commits: [feature/electri-127-mobile-model]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261017-01, integration-20261014-04]
---

## 背景

研华底盘台架运行的模型是 `robot_analytic_proxy_mobile.urdf`（SHA-256
`bba180d064ff4fe672b9b3fdfa7e8e8fd1bc9d1cac4f7cbc13c751ebd0fbff1c`），
已推送到 robot_description `feature/electri-127-deployed-mobile-model@c49e8ed`（基于 e189bdb）。
该线由模型方 #14 切换为 V3.2.2 吸盘生产模型，删除了夹爪入口。用户选择让主线锁定该模型。

## 改动

- `src/description` 导入 c49e8ed，tree `ec4a8c4` 与上游一致；source-lock 更新。
- 机型 `robot_model`：`gripper` 变体 → `mobile_proxy`（mobile URDF、限位、初始位置、SRDF）；
  单机 `end_effector: gripper` → `mobile_proxy`；manifest schema 变体改为任意名称、至少一个。
- 从组合中移出左右 PP 夹爪：arms 16 → 14 轴，删除 `gripper_pp` 组、夹爪控制器、使能策略与零位条目。
- 冻结 golden：arm/stationary/head 八个用例按新模型重写，仅改写规范形式不同的文件；
  与旧 builder 的等价性仍以原 `source_commit` 冻结为证。force 用例不变。
- 门禁：`check_v3_branch_contract` 改锁新版本并要求 `mobile_proxy`；`test_tf_contract` 改查 mobile 模型；
  导入的 Robot Model 保留上游原始字节，不做行尾换行与 ruff 格式检查。

## 验证

- composer 全部测试：189 passed，1 failed（`test_cli_uses_installed_share_and_reports_rejection`，
  源码目录无安装布局，主线基线同样失败）。
- `tools/tests`：246 passed、13 skipped；`check_v3_branch_contract.py`：PASS；
  robot_description 包测试 18 passed；`tools/quality_gate.sh`：PASS。

不连接设备，不授权使能或运动。

## 结论与冻结事实

- F1: 主线 Robot Model 为 c49e8ed，部署入口 SHA-256 与研华一致。
- F2: 在提供带夹爪关节的模型前，PP 夹爪不参与任何组合；带夹爪的真实手臂总线会多出两个从站。

## 遗留

- 解析代理手臂仅为装配近似，不是真机标定模型；手臂真机使用前需模型方复核。
- control_api_adapter 夹爪资源与 `Alfa_V3_grippers.draft.yaml` 保留未删，`configured:false`。
