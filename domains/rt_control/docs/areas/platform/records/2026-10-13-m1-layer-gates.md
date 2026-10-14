---
id: platform-20261014-01
area: platform
title: M1 四层门禁、包注册与 jsonschema 发布身份
date: 2026-10-14
type: feature
trigger: KKOZIA-6（SPEC v1.1 §5.5、M1、ADR-06/09/17）
commits: [agent/rt-control/25f929d47815]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-154]
---

## 背景

KKOZIA-6 为四层迁移提供离线门禁和构建注册，不迁移运行入口。实现与候选验证完成于 2026-10-13；
2026-10-14 按架构师在 KKOZIA-6 的分配登记为 platform-20261014-01，保留原文件路径。
本记录亦登记
2026-10-13 用户对 KKOZIA-2 子 issue 的架构师合并授权（ADR-17）。

## 改动

- 两包加入 CI 三处清单、Dockerfile、bootstrap 与 bringup 的 exec_depend。
  同步现有清单差异：Docker 补 robot_hw_can、damiao_head_controller、rolling_trajectory_controller；
  bootstrap 补 robot_hw_can、damiao_head_controller；五处顺序与 30 包集合一致。
- R1 从 manifest 派生包名并校验四层唯一归属；R2/R3 检查 XML 依赖和 C++ include；
  R4 检查纯数据包及代码 API 依赖；R5/R7 检查生产组合器/渲染器；R6 检查生产周期字面量；
  R8 检查四包 protocol 头文件。Python import / CMake find_package 已用于 R4，
  R2/R3 对这两类依赖的扩展仍留给 M2。
- 基线按规则、源包、目标包、类型、文件、内容、出现次数与到期里程碑匹配；行号仅作证据。
  实扫是 12 处周期字面量（M6）及 4 个依赖出现位置（两条逻辑边，M3d/M4c），
  与 issue 摘要的 10 处/两条表述有粒度差异；不删除或修改任何旧运行文件。
  R6 排除测试、schema、文档和非字面量转发；合法数值源仅为 manifest 与 controller_defaults。
- R9 的 CODEOWNERS 指向用户指定邮箱；merge-base 差异包含新增/修改/删除/重命名两端，
  PR 正文仅提醒，不能作为人工批准证据。分支保护需管理员配置。
- R10 禁止 apt 补丁版本锁定；CI/quality_gate 检查 Python 3.2.x；构建包装器在基础
  镜像查询 apt 候选，经 build-arg 写 LABEL，Docker RUN 核验已安装版本为 3.2.x 且与标签相等。
  不执行包装器真实构建，只用 fake Docker 验证参数与拒绝路径。
- BQ-154 保留原位使能 250 Hz 与主环 1 kHz 的差异，迁移 scope override 的落地归配置任务。
  协作规范 §3.3/§6 与域 AGENTS §6 同步架构师合并授权，安全策略仍需人批准。

## 验证

- 2026-10-14 同步已合入 M0b 的 `origin/main`（`a0a74e3`）后，实际任务分支
  `tools/quality_gate.sh`：PASS，239 passed、13 skipped；原两文件覆盖率 84%，
  含新模块 85%。此前 R1 两包缺失的阻塞已解除，规则未放宽。
  main 与此前 M0b 候选 `4fde3d3` 的树一致，沿用下述两包构建/测试证据。
- 隔离候选 = M0b PR #63 的 `4fde3d3220427152f25bdc3e99f35a1e8bca14cf` + 本任务改动：
  `tools/quality_gate.sh` PASS，239 项通过、13 项跳过，
  含新模块的门禁集覆盖率 85%，阈值仍为 80%。该结果不冒充 main 合并后结果。
- 任务分支 `python3 -m pytest tools/tests -q`：239 通过、13 跳过。
- 同一隔离候选的两包 `colcon build` / `colcon test --packages-select
  rt_control_composer rt_control_machine_config` / `colcon test-result --verbose`：PASS，
  153 tests、0 errors、0 failures、0 skipped（colcon 汇总口径）。
  首次构建缺 catkin_pkg，启用隔离 venv 的 system-site-packages 后恢复；首次测试
  因 pytest 9 与 Humble launch_testing 不兼容失败，对齐 pytest 6.2.5 后通过。
- 修改 shell 的 `bash -n`、`shellcheck --severity=warning`：PASS。
  Dockerfile 6 个 RUN 和 CI shell 块离线 `bash -n`：PASS；五处 30 包清单一致。
- ruff 0.11.13 检查与新增 Python 格式检查、Python 编译检查：PASS。
  clang-format-14 在本地不可用；本任务无 C++ 改动，CI 仍强制安装。
- 初次全工具测试发现 rolling 清单测试依赖旧代码字面量，已机械改为检查派生包集合；
  不放宽该包必须注册的断言。

2026-10-14 首次续作修正文档编号、日期和索引；随后 M0b 合入，合并 main 时保留
PROGRESS 两侧追加记录。在实际合并分支重跑必要质量门禁，无代码修正或新增测试。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 四层清单与逐条基线由 platform 管理；新的依赖/周期字面量不能复用整包豁免。
- F2: python3-jsonschema 的 apt 补丁版本不锁定；构建实际版本必须与 LABEL 一致并属于 3.2.x。
- F3: KKOZIA-2 子 issue 的 PR 由架构师在 CI 通过后合并（2026-10-13 用户授权），
  `safety_policy.yaml` 除外；该例外需 CODEOWNERS 人工批准，不由正文确认行代替。
- F4: BQ-154 未决；本任务不统一控制周期，不授权 T2–T4 或发布操作。

## 遗留

- M0b 已合入并同步；本地质量门禁通过。完整 CI 结果随 PR #64 交付。
- 12 处周期字面量和 4 个依赖位置已逐条登记；请架构师核对摘要数量差异。
- GitHub CODEOWNERS 邮箱解析与分支保护强制批准需独立核对/管理员配置。
- 不构建生产镜像，不部署、不运行硬件；T2 及以上未执行。若 BQ-154 裁定统一，
  由人另行授权并执行 1 kHz 原位使能 T3 复验。
