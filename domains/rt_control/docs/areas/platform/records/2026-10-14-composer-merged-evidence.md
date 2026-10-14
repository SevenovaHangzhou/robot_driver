---
id: platform-20261014-03
area: platform
title: 四层源码迁移合并证据与观察指标起点
date: 2026-10-14
type: investigation
trigger: KKOZIA-19；用户批准新增拒绝、安全策略原样迁移及CI通过后按序合并
commits: [c220e5ee0cd04f029b4f41070189766c9aa9ad12]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence:
  - https://github.com/SevenovaHangzhou/robot_driver/actions/runs/38583702242
  - https://github.com/SevenovaHangzhou/robot_driver/pull/68
supersedes: []
related: [platform-20261014-02, integration-20261014-04]
---

## 背景

#65、#66、#67、#68已在各自适用源码CI通过后按用户明确授权依次合并。
本记录补齐实际main观察指标起点，保留原记录的候选阶段证据。

## 改动

仅登记合并与验证事实，无运行代码、参数、配置或硬件变更。
四层源码迁移最终合并main：`c220e5ee0cd04f029b4f41070189766c9aa9ad12`。

## 验证

- #68最终提交b5a48d6完整CI全部适用检查通过：41包构建，2107 tests、0 errors、0 failures、23 skipped。
  两项governance、scope分类、build通过；Head-only按范围跳过；最终V3模型/机型/运行边界检查通过。
- 最新本地质量门禁245 passed、13 skipped；十份冻结金样、默认入口与实际单机离线准入证据见integration记录。
- 原位和force的T1查询/发现缺口仍保留；用户批准继续推进，不证明根因是环境，也不算T1通过。
- 手臂方向/运动标定、底盘有效计数/身份/拓扑/标定、全机传感拓扑等事实仍有TBD。
  actual real arm/arm-jtc仍拒绝；head/stationary/force-bench离线compose不代替真实设备预检。
- 没有生产镜像、目标机实时性、真实预检、使能、运动或部署证据；源码CI不冒充这些验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 四层源码迁移已合并main `c220e5ee0cd04f029b4f41070189766c9aa9ad12`，它是观察指标实际起点。
- F2: 统计窗口是此提交之后的前20个合并PR，包含文档PR；触碰`src/rt_control/rt_control_bringup/**`的比例目标≤30%。此指标不作为源码迁移验收。
- F3: V3迁移baseline为空；唯一冻结V2频率范围按用户批准排除并由SHA-256约束，原文件不变。
- F4: 合并和人工会话批准不等于GitHub Code Owner review、分支保护配置、设备准入或部署认证。

## 遗留

后续按独立验证任务补T1、硬件事实及设备认证。观察窗口尚未满20个PR，不提前声称达到指标。
