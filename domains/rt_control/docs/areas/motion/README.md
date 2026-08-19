# motion — 轨迹与运动执行

**范围**：whole_body_jtc 与 FJT 执行、diff_drive 底盘速度、控制器参数
（`controllers.yaml`）、关节限位（`joint_limits.yaml`）、控制环频率配置。
**Owner 包/资产**：`src/rt_control/rt_control_bringup` 的控制器/限位配置、
`patches/ros2_controllers`。

不属于本区：轨迹规划与任务编排（运控域，不在本仓库）、使能状态机（→ lifecycle）、
launch 结构与容器（→ release-deploy）。

## ELECTRI-102 交付入口

- [完成说明](../../electri-102-completion-report.md)
- [Motion 联调与生产端指南](../../electri-102-motion-integration-guide.md)
- [需求澄清与决策记录](../../electri-102-clarification-decisions.md)
- [mock 长稳与性能报告](../../electri-102-mock-performance-report.md)

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | 首点一致性准入启用时，whole_body_jtc 只接受 FJT action | [motion-20260819-01](records/2026-08-19-jtc-action-only-admission.md)#F1 | 有效（T1） |
| 01#F2 | 一致性准入关闭时保留上游 JTC topic 行为 | [motion-20260819-01](records/2026-08-19-jtc-action-only-admission.md)#F2 | 有效（T1） |
| 01#F3 | topic 存在性与准入开关绑定，不增设独立配置位 | [motion-20260819-01](records/2026-08-19-jtc-action-only-admission.md)#F3 | 有效（T1） |
| 02#F1 | rolling command 属于 Motion，session/state 属于 RT-Control | [motion-20260819-02](records/2026-08-19-electri-102-rolling-controller-port.md)#F1 | 有效（T1） |
| 02#F2 | rolling controller 核心已在 public feature-interface overlay 构建并加载 | [motion-20260819-02](records/2026-08-19-electri-102-rolling-controller-port.md)#F2 | 有效（T1） |
| 02#F3 | rolling service 细粒度结果与公共 DREE 映射逐项固定 | [motion-20260819-02](records/2026-08-19-electri-102-rolling-controller-port.md)#F3 | 有效（T1） |
| 02#F4 | controller 移植通过不等于 Motion 联调门槛已通过 | [motion-20260819-02](records/2026-08-19-electri-102-rolling-controller-port.md)#F4 | 有效（T1） |
| 03#F1 | Prime generation 不可改变 open 返回的 t=0 hold | [motion-20260819-03](records/2026-08-19-electri-102-rolling-correctness.md)#F1 | 有效（T1） |
| 03#F2 | accepted candidate 在 [E,R) 与 authoritative head 采样等价 | [motion-20260819-03](records/2026-08-19-electri-102-rolling-correctness.md)#F2 | 有效（T1） |
| 03#F3 | capacity 不累计 session 历史 | [motion-20260819-03](records/2026-08-19-electri-102-rolling-correctness.md)#F3 | 有效（T1） |
| 03#F4 | max_horizon 是 admission gate，不是只读 capability | [motion-20260819-03](records/2026-08-19-electri-102-rolling-correctness.md)#F4 | 有效（T1） |
| 04#F1 | 全量 validator 每个连续段只做一次 direct check | [motion-20260819-04](records/2026-08-19-electri-102-single-pass-validation.md)#F1 | 有效（T2） |
| 04#F2 | 删除重复循环不改变首个 RejectCode | [motion-20260819-04](records/2026-08-19-electri-102-single-pass-validation.md)#F2 | 有效（T2） |
| 05#F1 | Prime 全量校验，普通 replacement 只校验新 suffix | [motion-20260819-05](records/2026-08-19-electri-102-incremental-validation.md)#F1 | 有效（T2） |
| 05#F2 | checker 引入跨段状态时增量证明立即失效 | [motion-20260819-05](records/2026-08-19-electri-102-incremental-validation.md)#F2 | 有效（T2） |
| 05#F3 | production 不执行增量后再全量 | [motion-20260819-05](records/2026-08-19-electri-102-incremental-validation.md)#F3 | 有效（T2） |
| 06#F1 | 256 是 ceiling，不是每次复制量 | [motion-20260819-06](records/2026-08-19-electri-102-effective-trajectory-copy.md)#F1 | 有效（T2） |
| 06#F2 | capacity 64 有效 image copy 上界 14,936 bytes | [motion-20260819-06](records/2026-08-19-electri-102-effective-trajectory-copy.md)#F2 | 有效（T2） |
| 06#F3 | snapshot 尾部无语义，读路径不得越过 point_count | [motion-20260819-06](records/2026-08-19-electri-102-effective-trajectory-copy.md)#F3 | 有效（T2） |
| 07#F1 | 250 Hz Running 采样使用 generation-aware 单调 cursor | [motion-20260819-07](records/2026-08-19-electri-102-monotonic-sampler.md)#F1 | 有效（T2） |
| 07#F2 | 非 RT reference sampler 保留为差分 oracle | [motion-20260819-07](records/2026-08-19-electri-102-monotonic-sampler.md)#F2 | 有效（T2） |
| 07#F3 | splice 在 t=R 选择 right state | [motion-20260819-07](records/2026-08-19-electri-102-monotonic-sampler.md)#F3 | 有效（T2） |
| 08#F1 | provisional 与 test-only 使用独立授权位 | [motion-20260819-08](records/2026-08-19-electri-102-provisional-authority.md)#F1 | 有效（T1） |
| 08#F2 | public/local limits-source 枚举固定一致，provisional=3 | [motion-20260819-08](records/2026-08-19-electri-102-provisional-authority.md)#F2 | 有效（T1） |
| 08#F3 | 非生产来源 opt-in 不绕过版本与字段校验 | [motion-20260819-08](records/2026-08-19-electri-102-provisional-authority.md)#F3 | 有效（T1） |
| 09#F1 | rolling 运行参数是 configure-time 冻结快照 | [motion-20260819-09](records/2026-08-19-electri-102-runtime-parameters.md)#F1 | 有效（T1） |
| 09#F2 | tolerance 参数均为固定协议轴序的 14 值数组 | [motion-20260819-09](records/2026-08-19-electri-102-runtime-parameters.md)#F2 | 有效（T1） |
| 09#F3 | Open 回报实际参数，非法配置不静默钳制 | [motion-20260819-09](records/2026-08-19-electri-102-runtime-parameters.md)#F3 | 有效（T1） |
| 10#F1 | provisional 文件字节 SHA-256 是 public limits version | [motion-20260819-10](records/2026-08-19-electri-102-provisional-envelope.md)#F1 | 有效（T1） |
| 10#F2 | provisional schema 严格完整且无隐式 fallback | [motion-20260819-10](records/2026-08-19-electri-102-provisional-envelope.md)#F2 | 有效（T1） |
| 10#F3 | provisional authority 在 configure/activate/public state 一致 | [motion-20260819-10](records/2026-08-19-electri-102-provisional-envelope.md)#F3 | 有效（T1） |
| 10#F4 | provisional 不替代 BQ-138 台架实测或运动授权 | [motion-20260819-10](records/2026-08-19-electri-102-provisional-envelope.md)#F4 | 有效（T1） |
| 11#F1 | rolling state 保持唯一公共 publisher | [motion-20260819-11](records/2026-08-19-electri-102-mode-result-state.md)#F1 | 有效（T1） |
| 11#F2 | controller 间 mode result 只经私有有序事件交接 | [motion-20260819-11](records/2026-08-19-electri-102-mode-result-state.md)#F2 | 有效（T1） |
| 11#F3 | rolling 严格校验新事件且 deactivate 不复用旧证据 | [motion-20260819-11](records/2026-08-19-electri-102-mode-result-state.md)#F3 | 有效（T1） |
| 12#F1 | 软件门包含 14 项矩阵和真实墙钟 600 秒 fake-250 Hz soak | [motion-20260819-12](records/2026-08-19-electri-102-mock-and-motion-handoff.md)#F1 | 有效（T2） |
| 12#F2 | 10/30/250 Hz soak 的 18,001 批全部接受且零容忍计数全零 | [motion-20260819-12](records/2026-08-19-electri-102-mock-and-motion-handoff.md)#F2 | 有效（T2） |
| 12#F3 | Motion 示例只依赖公共接口且默认 dry-run | [motion-20260819-12](records/2026-08-19-electri-102-mock-and-motion-handoff.md)#F3 | 有效（T1） |
| 12#F4 | public DDS peer 与 controller fake loop 是互补证据 | [motion-20260819-12](records/2026-08-19-electri-102-mock-and-motion-handoff.md)#F4 | 有效（T1/T2） |
| 12#F5 | GenericSystem 不模拟 CiA402 enable，不构成完整 live bringup 证据 | [motion-20260819-12](records/2026-08-19-electri-102-mock-and-motion-handoff.md)#F5 | 有效（T1） |

## 记录索引（倒序）

- 2026-08-19 [ELECTRI-102 mock 长稳与 Motion 公开接口交接](records/2026-08-19-electri-102-mock-and-motion-handoff.md) — feature，PASS（T2）
- 2026-08-19 [ELECTRI-102 模式结果进入 rolling 公共状态](records/2026-08-19-electri-102-mode-result-state.md) — feature，PASS（T1）
- 2026-08-19 [ELECTRI-102 14 轴 provisional 包络严格加载](records/2026-08-19-electri-102-provisional-envelope.md) — feat，PASS（T1）
- 2026-08-19 [ELECTRI-102 rolling 运行参数 YAML 化并冻结](records/2026-08-19-electri-102-runtime-parameters.md) — feat，PASS（T1）
- 2026-08-19 [ELECTRI-102 provisional 与 test-only 限值授权隔离](records/2026-08-19-electri-102-provisional-authority.md) — feat，PASS（T1）
- 2026-08-19 [ELECTRI-102 RT 轨迹采样改单调游标](records/2026-08-19-electri-102-monotonic-sampler.md) — perf，PASS（T2）
- 2026-08-19 [ELECTRI-102 snapshot 只复制有效轨迹节点](records/2026-08-19-electri-102-effective-trajectory-copy.md) — perf，PASS（T2）
- 2026-08-19 [ELECTRI-102 rolling 增量后缀校验](records/2026-08-19-electri-102-incremental-validation.md) — perf，PASS（T2）
- 2026-08-19 [ELECTRI-102 删除 rolling 重复 segment 校验](records/2026-08-19-electri-102-single-pass-validation.md) — perf，PASS（T2）
- 2026-08-19 [ELECTRI-102 rolling 前缀与 horizon 正确性闭环](records/2026-08-19-electri-102-rolling-correctness.md) — fix，PASS（T1）
- 2026-08-19 [ELECTRI-102 rolling 控制器移植到公共接口契约](records/2026-08-19-electri-102-rolling-controller-port.md) — feat，PASS（T1）
- 2026-08-19 [JTC 首点一致性启用时关闭 topic 命令旁路](records/2026-08-19-jtc-action-only-admission.md) — fix，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-019 三条（partial / corrective / 14-axis minimal FJT）
- `docs/fjt-14axis-low-speed-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-119、BQ-120、BQ-122
