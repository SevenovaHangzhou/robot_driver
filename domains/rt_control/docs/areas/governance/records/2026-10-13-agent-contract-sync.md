---
id: governance-20261013-02
area: governance
title: 同步语言规范、三档测试与agent分支授权
date: 2026-10-13
type: decision
trigger: "口头需求（kkozia）"
commits: [feature/rt-control-agent-contract]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [governance-20261013-01]
---

## 背景

基于任务一分支同步agent工作规则。用户授权独立提交/推送/PR、不合并，并允许本次仅用governance记录号。验收：文档引用可执行配置、语言职责明确、三档测试一致、分支授权清楚，安全/实时/硬件授权及T0–T4定义不变。

## 改动

- 第9.2/9.3引用clang-format14与ruff0.11.13增量门禁，不再要求black/isort。
- 实时循环、硬件插件、控制器、总线节点使用C++17；组合器/校验器/launch/工具/非实时适配器使用Python3.10，不引入第三种实现语言。已有Bash/CMake不在本轮转换范围；plc_node不加新功能。
- 超800行文件不再增加新逻辑；独立可测单元接入，整体拆分另立任务。
- 测试分缺陷回归、随实现交付、无需新增三档；保留确定性、禁止放宽断言和实机边界。根第7节与域第4/5.2节同步引用，避免共同契约互相冲突。
- 根第8节和域第6节认可任务指派即自己分支提交/推送/开PR授权；合并仍由人完成，硬件/tag/发布物不从指派推导授权。

## 验证

- `STYLE_BASE_REF=feature/rt-control-executable-style tools/quality_gate.sh`：PASS，224通过、13跳过，门禁集84%；没有受影响的文档句子断言失败，无需改测试。
- 原有安全、实时、硬件条款和验证分级段落逐段比较：未修改。
- `git diff --check`、`git diff --cached --check`：PASS。
- 第三档纯文档变更，不新增测试。未编译/运行控制栈或硬件，验证级别T0，记录状态UNVERIFIED。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 新增测试按协作规范9.5三档执行，验收条件先写Issue文字，不要求先写测试代码。
- F2: 任务指派授权agent自己的分支提交/推送/PR，合并由人完成，硬件等授权独立。
- F3: C++17/Python3.10语言分工和超800行文件增量约束从本记录生效。

## 遗留

后续任务三更新功能区；本次未迁移源码、没有改变既有硬件裁决。前置PR必须先合并，由人维护串联顺序。
