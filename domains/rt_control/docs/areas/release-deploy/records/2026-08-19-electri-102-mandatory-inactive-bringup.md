---
id: release-deploy-20260819-01
area: release-deploy
title: ELECTRI-102 mandatory-inactive motion writer bringup
date: 2026-08-19
type: feature
trigger: ELECTRI-102 / T-07 / E102-D24 / E102-D37
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rt_control_bringup/launch/rt_control.launch.py, src/rt_control/rt_control_bringup/test/test_rolling_bringup_contract.py, src/rt_control/rt_control_bringup/test/test_mock_contract.py]
supersedes: []
related: [ELECTRI-102, motion-20260819-01, motion-20260819-10, lifecycle-20260819-03]
---

## 改动

controller manager 注册 `rolling_trajectory_controller/RollingTrajectoryController`，bringup
运行依赖显式包含该插件。motion writer 不并行抢占接口：launch 先配置
`whole_body_jtc --inactive`，成功后才配置 `rolling_trajectory_controller --inactive`，成功后
才加载并激活 enable_manager；任一前驱非零退出都发出 Shutdown。

Humble controller 子节点不会读取注入 controller_manager 的同名 dotted 参数。launch 因此用
ament index 取得安装包内 provisional envelope 的绝对路径，生成 controller-node 专用临时
参数文件，再由 rolling spawner 同时传入主 controllers 文件与覆盖文件；shutdown 删除临时
文件。YAML 默认仍为空，绕过支持的 launch 时保持 fail-closed。

## 验证

- 独立 pytest 4/4：插件注册、运行依赖、`--inactive` 参数、双 param-file、失败 Shutdown、
  JTC→rolling→enable 源码链及绝对路径 artifact。
- 安装态完整 mock launch 通过：补丁版 JTC、JSB、diff-drive 与本分支 rolling 实际由 pluginlib
  加载；provisional WARN 中路径指向安装副本。
- controller_manager 运行态复核：whole_body_jtc=inactive、rolling=inactive、
  enable_manager=active；JTC `/whole_body_jtc/joint_trajectory` topic 不存在。
- 既有 graph/QoS/frequency suite 全绿，日志再次证明 JSB 配置保持 100 Hz、250 Hz 调度得到
  125 Hz。

mock 构建使用 deps.repos 固定的 ros2_controllers commit 加仓库两份冻结 patch；未访问真实
CANopen/EtherCAT 总线、未 reset、enable 或运动。宿主机缺少 Lely，因而 production CANopen
包闭包不是本记录的通过证据。

## 冻结事实

- F1: motion writer 启动链固定为 JTC INACTIVE → rolling INACTIVE → enable_manager ACTIVE。
- F2: rolling provisional 包络由 spawner 参数文件注入安装态绝对路径。
- F3: 任一 mandatory-INACTIVE spawner 失败均停止 launch，不启动 enable_manager。
