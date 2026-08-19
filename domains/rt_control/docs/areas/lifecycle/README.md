# lifecycle — 使能/失能/故障复位/停机收敛/断电恢复

**范围**：14 轴分批使能状态机、失能与停机收敛、故障复位流程、断电恢复入口、
控制字/状态字语义。
**Owner 包/资产**：`src/rt_control/rt_control_semantic_components`、
`src/rt_control/enable_manager`（含 `rt_disable_once`）。

不属于本区：驱动器侧 PDO/SDO 定义（→ ecat-axes）、启动脚本与确认短语
（→ release-deploy）、公共 `/control/set_enabled` 适配（→ contract）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | /rt/disable 挂起症状真实（4 例日志）；病因指向服务端 waitForResult 30s 自旋+RT 循环未填槽，非串行预算叠加 | [lifecycle-20260814-01](records/2026-08-14-rt-disable-log-forensics.md)#F1 | 有效 |
| 01#F2 | rt_disable_once 三阶段共享单一绝对 deadline 是已验证现状（29 例日志佐证），非缺陷 | [lifecycle-20260814-01](records/2026-08-14-rt-disable-log-forensics.md)#F2 | 有效 |
| 02#F1 | enable_manager 单元测试基线 = 36 用例 @ 3ff153d（a–f 六项）；行为修改须先过套件 | [lifecycle-20260814-02](records/2026-08-14-enable-manager-gtest.md)#F1 | 有效 |
| 03#F1 | Cia402Axis 只拥有类型化接口绑定/读写/解码，不拥有拓扑、批次、策略或硬件生命周期 | [lifecycle-20260819-03](records/2026-08-19-semantic-component-policy-split.md)#F1 | 有效（Docker/T1） |
| 03#F2 | enable_manager topology/safety 参数无机型默认值，必须显式配置，并在 configure 阶段校验冻结 managed joints、批次、时序和 Ti5 终态策略 | [lifecycle-20260819-03](records/2026-08-19-semantic-component-policy-split.md)#F2 | 有效（Docker/T1；powered 生命周期待验证） |
| 03#F3 | ELECTRI-94 不改变 BQ-122 的退出顺序 | [lifecycle-20260819-03](records/2026-08-19-semantic-component-policy-split.md)#F3 | 有效（最终镜像 Mock 已复核） |

## 记录索引（倒序）

- 2026-08-19 [CiA402 semantic component 与使能策略分层](records/2026-08-19-semantic-component-policy-split.md) — feature，PASS（Docker/T1；powered 生命周期未执行）
- 2026-08-14 [enable_manager 状态机表驱动 gtest（ELECTRI-93）](records/2026-08-14-enable-manager-gtest.md) — feature，PASS（工控机实跑 36/36）
- 2026-08-14 [/rt/disable 缺陷声称的日志取证：症状真实、机制误诊](records/2026-08-14-rt-disable-log-forensics.md) — investigation，PASS（T2 只读）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-DEV-NATIVE-002 系列中 reset/JTC/staged-disable 各 corrective、
  T-IF-RT-003 (reset staged-disable corrective)
- `docs/xmc-updown-enable-commissioning-20260727.md`、
  `docs/ethercat_enable_disable_commissioning.md`
- 相关 BQ（不完全）：BQ-006（rt_watchdog 移除）、BQ-114
