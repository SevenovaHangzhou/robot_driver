---
id: governance-20261013-01
area: governance
title: 固定格式工具与增量风格门禁
date: 2026-10-13
type: feature
trigger: "口头需求（kkozia）"
commits: [feature/rt-control-executable-style]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: []
---

## 背景

七个职责 agent 上线前建立可执行风格基线。用户要求本任务独立 PR、后续任务串联；本次明确允许仅用 governance 记录号追踪，不创建 Linear Issue。基线 main@95816cb8ed19426c5c7a49800c8fb2f21d0fb83f。
验收：固定工具版本、仅检查改动、覆盖新增规则、所有自有C++目标继承五项警告、不修改控制源码或行为。

## 改动

- 根 `.clang-format` 来源为 ament_lint humble 的 ament_clang_format 配置，固定 clang-format 主版本14；明确两空格、100列、Middle指针/引用对齐。选择关闭include排序和允许类内短函数，减少存量差异。
- Ruff固定0.11.13，Python target py310，100列；启用E9/F63/F7/F82/F401。导入排序I暂不启用：存量88处，与近零存量目标不符；F401仅2处，所选错误类0处。没有修复未涉及的存量文件。
- `style_gate.py`以merge-base检查工作树（包括已提交/暂存/未暂存的跟踪文件）：C++使用git-clang-format --diff，Python改动文件check、新增文件format --check。新增文件须先git add。删除文件排除；缺失base拒绝，禁止静默跳过。显式STYLE_BASE_REF用于串联PR基线。
- 质量门禁与pre-commit共用同一入口；CI获取完整历史与PR真实head/base，始终安装格式工具，即使使用缓存镜像。缺工具本地NOTICE、CI强制；错误版本拒绝。
- 14个自有C++包的CMake在目标定义前设置目录警告基线：-Wall -Wextra -Wpedantic -Wconversion -Wshadow；覆盖ament测试目标和循环生成目标。不加-Werror，不改源码消警告。repository_gate检查此受限形式和禁止关闭警告。

## 验证

- 全仓169个C++文件只读format测量：ament原始配置156文件/14208增删行；关闭include排序148文件/13968行；再允许类内短函数148文件/13752行。统计为逐文件格式化前后ndiff增删行之和，不是变更提交规模。
- `python3 -m pytest tools/tests/test_style_gate.py -q`：17通过，覆盖merge-base范围、错误base、工具缺失/版本、C++拒绝、Python新增文件策略和CMake通过/拒绝。
- `tools/quality_gate.sh`：224通过、13跳过；repository_gate/pr_contract_gate合并覆盖率84%。本地缺ShellCheck，按既有规则提示，CI强制。格式工具在忽略目录安装，未修改宿主配置。
- 自有包新增编译警告计数：全部UNVERIFIED。本机有Humble但缺/usr/local/etherlab用户态库及当前冻结vendor完整构建闭包，未声称完成全包构建；由PR build作业补充证据。具体包：bms_node、damiao_head_controller、enable_manager、lpms_nav3_can、modbus_tcp_rtu485、plc_io_modbus、robot_hw_can、robot_hw_canopen、robot_hw_ethercat、rolling_trajectory_controller、rt_control_semantic_components、rt_diagnostics、rt_force_torque_broadcaster、swerve_driver。
- 本次仅T0工具/配置验证，无控制运行、Docker构建或硬件验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 新改动使用clang-format14和ruff0.11.13；C++改动行、Python改动文件、新Python文件格式采用不同检查粒度。
- F2: 自有C++目标及测试在本包目录继承五项警告，不将-Werror加入基线。
- F3: 不对存量代码进行整体重排；导入排序的全仓推进另行评审。

## 遗留

CI及全包编译警告数量待验证；这不影响已记录的本地门禁结果。新文件未暂存时不进入Git diff，提交前必须暂存后重跑。后两个PR使用前一分支作base，合并由人完成。
