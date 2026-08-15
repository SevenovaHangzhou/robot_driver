# RT-Control 发布前测试体系

本目录是 RT-Control 发布前测试用例的事实源：`README.md` 定义分层、分类、用例
schema 和发布门禁映射；`cases/` 按分类存放用例。运行器（runner）、JSON 报告和
baseline 工具是后续子任务，落地前用例按 `status` 字段人工执行、机器记录。

体系设计背景与裁决见
[governance-20260813-02](../docs/areas/governance/records/2026-08-13-test-case-catalog-v1.md)；
Linear 任务：ELECTRI-74。

## 风险分层：授权边界决定层级

分层直接映射既有授权机制（启动脚本确认短语、runbook L0–L3 危险等级、域契约
5.7 节 mock/实机分级），不新设授权：

| 层级 | 内容 | 允许的写动作 | 执行方式 |
| --- | --- | --- | --- |
| T0 | 静态：门禁、构建、包测试、语义比对、发布身份 | 无（不启动 ROS 进程） | CI 全自动 |
| T1 | Mock 运行：`use_mock_hardware:=true` 全图与契约探测 | 无（不碰任何总线） | CI 全自动 |
| T2 | 实机只读：总线观测、cyclictest、诊断快照 | 无（不 reset/enable/运动/写 PLC） | 目标机自动脚本 |
| T3 | 生命周期：启动、reset、enable、disable、停机、断电恢复 | 写控制字，不产生运动 | 现场人工授权执行 |
| T4 | 功能运动：FJT、底盘、真空/PLC 输出、长稳带载 | 真实运动与 PLC 写 | 现场人工授权执行 |

**发布门禁的自动部分只到 T2**。T3/T4 用例由现场人员按用例的 `entry` 执行确认
短语流程，结果与证据回填报告；任何自动化不得替人完成确认输入。

## 测试分类与用例文件

| 分类 | 文件 | 主要层级 |
| --- | --- | --- |
| 静态 | [cases/static.yaml](cases/static.yaml) | T0 |
| 只读运行 | [cases/readonly-runtime.yaml](cases/readonly-runtime.yaml) | T1/T2 |
| 生命周期 | [cases/lifecycle.yaml](cases/lifecycle.yaml) | T3 |
| 功能运动 | [cases/motion.yaml](cases/motion.yaml) | T4 |
| 实时性能 | [cases/realtime-perf.yaml](cases/realtime-perf.yaml) | T2 |
| 长稳 | [cases/endurance.yaml](cases/endurance.yaml) | T2/T4 |
| 异常恢复 | [cases/recovery.yaml](cases/recovery.yaml) | T3 |

用例 ID：`TC-<分类码>-<两位序号>`，分类码 ST/RO/LC/FM/RT/LS/AR 与上表一一对应。
ID 一经使用不复用；用例作废保留条目并置 `status: retired`。

## 用例字段（schema）

每个用例必含以下字段；`TBD` 表示事实未裁决，禁止填"合理默认值"：

```yaml
- id: TC-XX-00            # 用例 ID
  title: 一句话标题
  purpose: 目的与证明目标
  area: 九功能区之一        # 与 docs/areas/ 一致，增量执行按此分组
  risk: T0..T4
  writes: { reset: no, enable: no, motion: no, plc: no }   # 四个授权位
  env: native | docker | both   # 适用环境；有差异时加 env_note 说明
  covers: [路径 glob]       # 改动触发范围（scope resolver 依据）
  traces: [记录坐标/BQ/契约文档]  # 追溯：<记录id>#F<n>、BQ-xxx、权威文档路径
  preconditions: 前置条件与设备状态
  entry: 精确命令或自动化入口（多行）
  pass: [量化判据列表]      # 全部满足为 PASS
  evidence: 必须采集的证据及路径约定
  cleanup: 回滚/清理方法
  status: automated | manual | planned | blocked | retired
  v010_gate: run | reference | no   # V0.10 门禁：执行 / 引用历史证据 / 不要求
```

判据里引用权威文档（如 vendored `robot_interfaces/contract/views/rt_control.md`）时，以文档
为准，用例不复制其内容——避免第二事实源。

可选字段 `auto: <单条 shell 命令>`：声明该用例可被 runner 自动执行（退出码 0 即
PASS，输出归档为证据）。**runner 只会自动执行 risk ∈ {T0,T1,T2} 且 writes 四位
全 no 的用例**；对不满足条件的用例声明 `auto` 会被 runner 拒绝执行——这是测试
体系自身的安全边界。

## Runner

入口：`tools/release_test_runner.py`（子命令 `validate` / `plan` / `identity` /
`run` / `delta`）。典型用法：

```bash
# 校验用例目录 schema
python3 tools/release_test_runner.py validate

# V0.10 门禁选集 + 生成 T3/T4 人工执行卡
python3 tools/release_test_runner.py plan --gate v010 --cards cards.md

# 发布身份六元组（TC-ST-04；--image 可选）
python3 tools/release_test_runner.py identity --image rt-control:<sha>

# 执行 auto 用例、合并人工结果、产出报告
python3 tools/release_test_runner.py run --gate v010 --execute \
  --results-dir results/ --out report.json --summary summary.md

# 候选指标 vs baseline
python3 tools/release_test_runner.py delta \
  --baseline domains/rt_control/testing/baselines/v0.yaml --candidate metrics.yaml
```

人工结果回填约定：`results/<用例ID>.yaml`，字段 `status`（PASS/FAIL/UNVERIFIED）、
`evidence`（路径列表）、`operator`、`date`、`notes`；`v010_gate: reference` 的用例
必须附 `reference.record`（历史记录坐标）与 `reference.commit`。runner 绝不代替
现场人员输入任何硬件授权确认短语。

## 报告三态与 V0.10 最小门禁

报告状态只有 PASS / FAIL / **UNVERIFIED**；没有证据的结论一律 UNVERIFIED，
不得从源码检查推导运行结论。V0.10 最小门禁 = 所有 `v010_gate: run` 用例在冻结
候选 commit 上执行通过 + 所有 `v010_gate: reference` 用例给出被引用历史记录的
坐标及其 commit 与候选的差异说明（差异超出冻结范围则记 UNVERIFIED）。

报告头必须包含发布身份六元组（源码 commit、公共契约 SHA、vendor SHA、IgH
commit、镜像 label、镜像 digest），见 TC-ST-04。

## Baseline

首个 baseline 以既有实测登记，登记文件在 [baselines/](baselines/)：性能窗口指标来自
`domains/rt_control/docs/docker-deployment-performance-summary.md`（180 s 窗口，
当前主机）。**cyclictest 无可登记记录**：当前主机的 30 分钟运行被意外断电中止
（BQ-105），T-009 未完成；`host-setup-record.md` 中的完整 30 分钟记录属已退役
alfa-two 主机，仅作历史参考，不得用作当前主机 baseline——TC-RT-01 首次在当前
主机通过后补登记。候选版本跑同口径用例产出 delta（指标、baseline 值、差值、判定）。
delta 判定阈值已由 BQ-134 裁决，固化于 [thresholds.yaml](thresholds.yaml)。
