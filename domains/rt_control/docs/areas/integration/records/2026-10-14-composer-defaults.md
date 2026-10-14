---
id: integration-20261014-04
area: integration
title: 用户批准的组合器默认入口切换与V3删旧
date: 2026-10-14
type: feature
trigger: 用户确认头部参数并明确批准arm、arm-jtc、stationary、head、force-bench全部切默认
commits: [feature/kkozia-19-composer-defaults]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence:
  - ../attachments/composer-defaults/rehearsals.json
  - ../attachments/composer-defaults/actual-unit-admission.json
supersedes: []
related: [integration-20261014-01, integration-20261014-02, integration-20261014-03, BQ-154]
---

## 背景

用户在停止点三回复：“是的 使用的是这组参数，批准全部切默认”。
确认范围是已列头部ID、速率、运动profile与时序，以及五个入口的默认组合路径。
该授权不包含设备操作，不把T1缺口或未测标定写成已验证。

## 改动

- 四个runtime launch（arm包含独立jtc_only模式）默认只调用compose/preflight，删除use_composer、
  旧calibration_file/head_can_config开关。真实预检仍在创建Node之前；rt_control_start保留信号/失能
  收尾，只选择入口，增加被动force-sensors选择。
- 头部机型配置verified:true、authorities.confirmation_person:user，日期记录为本次确认接收的UTC日期。
  方向与模型零位仍TBD。手臂verified:false、底盘/全机拓扑等缺项保留真实拒绝。
- 删除六个旧builder/profile Python模块、对应旧测试和头部Xacro wrapper。
  旧数值oracle改为冻结golden；所有十份输出及其哈希不更新。
- 删除命名的旧V3配置；夹爪Action参数迁control_api_adapter。
  协议构建校验改消费owner profile；legacy V3专用shape检查退役，保留新renderer/schema/conformance。
  二代variants/alfa_v1.yaml、其它二代slaves及bringup/config/joint_limits.yaml保持不变。
- head/swerve controller文件去掉周期字面量，controller_manager来自主系统清单；头部JSB50Hz由
  controller_defaults的显式parameter_view补齐，golden参数保持相同。主环1kHz、原位250Hz未统一。
- 清单验证入口与CLI迁到composer公共边界，选装来自deployment；runtime-ready全局门仍只用于
  validation入口或ready profile。旧已安装参数文件不作为fallback。
- 过期250Hz概述替换为实时主环说明；BQ-154、头部command_rate与历史记录保留。
  swerve_driver/README.md的调度描述未替换，继续列入B9边界。

## 验证

- 头部参数确认后实际real离线compose通过；stationary/force-bench也通过，arm与arm-jtc仍按
  verified:false拒绝。所有结果未执行真实preflight，见实际单机JSON附件。
- b1ba303下一轮CI的rolling并发测试触发kSnapshotLateBoundary；RT测试线程无墙钟节拍，可在
  producer被调度前推进完整400ms虚拟时间，原测试使用最早边界没有留发布/消费预算。
  仅修测试为覆盖其100周期虚拟预算的替换边界，并消费最终发布；保留并发线程、停止断言，
  加强activeGeneration>=2确保真正消费替换。生产控制器、迟到停止条件及安全数值不改。
  修复用例连续10次通过，完整rt_update 15项通过，质量门禁245 passed/13 skipped。
- #65/#66/#67已按用户明确批准与各自CI成功结果顺序合并；本增量#68需修复后的CI成功再合并。
- 最新远端提交f4dd6af的完整构建/测试通过：2107 tests、0 errors、0 failures、23 skipped；
  最后V3契约step未source ROS/overlay导致ament_index_python导入失败，整体CI失败。
  已只补该step的bash/source环境，校验逻辑不变；修复后本地契约通过，远端待验证。
- 前置PR67源码CI已通过；[CI](https://github.com/SevenovaHangzhou/robot_driver/actions/runs/38545780780)。
  不代替本增量CI，前置原位/force T1验证缺口保留。
- 新隔离目录11包构建通过；10个受影响包实际colcon test通过：712 tests、0 errors、0 failures、0 skipped。
- 34个离线/入口用例通过，十份golden规范化零差异。四入口的新测试确保compose/preflight顺序，
  预检拒绝时Node未构造；独立JTC输出仍由冻结case检验。
- tools/check_v3_branch_contract.py通过：模型/接口锁定身份、14CSP、安全依赖、V3默认validation入口保持。
- 用户批准范围例外后，最终质量门禁通过244 passed、13 skipped；新增用例验证冻结文件改动时
  哈希与周期检查同时拒绝。原11包/712 tests的实现未变，不重跑未受影响的包级套件。
- 最终工具质量门禁通过243 passed、13 skipped；无新未豁免违规，固定样式工具通过。
  无参数validate_config_files.py检查已安装数据包全部devices/machines/units信封，通过；不读evidence。
- 三项演练实际执行git diff --name-only main并按包统计：方向1个machine文件；新增轴3个数据文件
  （arms+manifest+unit zero）、两策略、1模型；同族新设备3个数据文件。三例offline mock compose通过，
  integration文件均0。新增轴的单机zero是安全覆盖必需字段，比机型设计清单额外触碰units，未规避校验。
  演练main是候选5d8c9bd独立基线，不冒充远端已合并main；演练改动不推送、不部署。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 用户已批准五个默认入口与当前头部参数；审批不解除手臂/底盘/拓扑的未确认事实。
- F2: 默认入口已迁移，golden值/顺序与旧行为一致；未执行设备预检、使能或运动。
- F3: 旧V3实现退役，V2描述符和既有硬件行为保持；缺项不能用fixture补生产值。
- F4: 用户已批准冻结V2文件排除V3频率验收，V3基线清空；哈希约束防止排除范围接受文件改动。

## 遗留

- 用户随后批准冻结V2 Xacro排除本次V3 A3/清空基线验收。V3 baseline_violations现为空；
  该单一文件在frequency_scope_exclusions公开登记路径、SHA-256与授权理由。
  只有内容哈希不变才排除R6；若文件改动，冻结检查和周期字面量检查都失败，其它规则始终继续检查。
  这不是全包豁免，V2原文件未改动。
- 直接关键词grep会包含合法变量转发、schema、fixtures与冻结golden；R5/R6生产规则的结果和字面量
  证据独立报告，不通过删除字段或改golden消除命中。
- 第二步原位和第四步force T1未闭环；用户允许继续开发，不作为根因裁定或PASS。
- 本增量远端CI待完成；完整镜像、目标机RT、实机及部署未验证。
- 观察指标基线必须取本增量合并后的main SHA；目前未合并，记录为pending_merge。后续前20个合并PR
  触碰rt_control_bringup比例目标≤30%，不作当前验收。人工批准和CODEOWNERS状态不伪造。
