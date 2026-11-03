---
id: arm-control-20261103-01
area: arm-control
title: 重力前馈默认关闭的组合与启动开关
date: 2026-11-03
type: feature
trigger: 用户确认独立开关和接入限制，要求评估默认运行项无影响后上库
commits: [codex/gravity-feedforward]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [D-004]
---

## 背景

用户要求给重力前馈增加与 rolling 同层的模块配置开关。rolling 由启动/组合参数
`jtc_only` 选择；重力前馈采用独立的 `gravity_feedforward`，用户确认默认关闭且
在实机接入条件未满足时明确拒绝开启。

## 改动

- composer API 增加 `gravity_feedforward=False`，生成 CLI 增加
  `--gravity-feedforward`；省略该标志即关闭。
- 主线统一 scope runtime 增加 `gravity_feedforward:=false|true`，沿用 `--arm-runtime` /
  `--arm-jtc` 到该入口的既有转发。
  这是启动时的组合选择，不是运行中控制器参数。
- artifact 记录选择；旧 artifact 缺失该字段解释为关闭，选择不一致拒绝启动。
- 请求开启在配置生成/启动准入阶段报 D-004，先于 renderer、写配置、总线锁、
  preflight 和节点创建；mock 和 real 一致。未加入生产控制器/PDO 映射。

## 验证

评估基线：`origin/main` @ `4bf8c4f`，使用独立工作树，未带入原工作区的模型、rolling
及 mock 硬件改动。原草稿使用 BQ-155；迁入主线后依记录规则改记 D-004。

- ROS 2 Humble / Python 3.10：`hw_semantic_components`、`ctrl_gravity_feedforward`、
  `integ_composer`、`integ_bringup` 四包隔离构建通过。
- `pytest src/rt_control/integ_composer/test src/rt_control/integ_bringup/test/test_scope_lifecycle.py`：
  262 项通过；覆盖默认关闭、旧 artifact 兼容、与 JTC 选择独立、开启拒绝及既有 artifact 保留。
- 重力控制器 C++ 测试 12 项通过（colcon 含容器计数 13，0 失败/跳过）。
- 对主线与改动后的所有 28 个 profile/scope/mock/JTC 组合做内存组合比较：
  7 个可生成组合的 89 份输出文件逐字节哈希一致，控制器顺序和生命周期计划一致；
  21 个受既有标定/草稿等条件限制的组合保留相同拒绝原因。未填补或绕过这些条件。
- 使用主线生成的 5 份旧 artifact 检查启动构造：普通双臂、JTC 双臂、双臂静态使能、
  头部和力传感器；节点、参数、命令行、顶层动作与基线一致。只构造 launch 动作，未运行进程。
- 迁移后的控制器与外部 `static_gravity` 对照 6006 组单臂计算，最大绝对差
  `1.5631940186722204e-13 N·m`；主线模型 SHA-256 为
  `beb734cb900489b5792182364bfad3ae60c826a832e3860fad20bf16714b61b3`。
- `tools/quality_gate.sh` 通过；clang-format 14、ruff 0.11.13 实际执行通过。
  shellcheck 未安装，未执行；依赖和格式工具仅放在临时目录，未修改宿主配置。
- `git diff --check`、暂存区逐行审查通过；CI 增加新包的依赖解析、构建和测试选择，
  生产镜像包集合及运行配置不变。
- 构建、测试和默认项对比报告保存在本机 `/tmp/gravity-publish-review/`；未提交生成物。

未验证：完整 mock 栈加载/停机、真实总线、PDO 输出、电流换算和上电运动；
本记录不将离线测试声明为重力前馈实机验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `gravity_feedforward` 默认关闭，独立于 rolling 的 `jtc_only`。
- F2: 当前 `true` 明确报 D-004，不代表重力前馈可在生产路径启用。

## 遗留

D-004 的逐轴换算/符号、限幅、PDO 和生命周期接入条件仍未解除。
