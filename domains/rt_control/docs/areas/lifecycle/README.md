# lifecycle — 使能/失能/故障复位/停机收敛/断电恢复

**范围**：14 轴分批使能状态机、失能与停机收敛、故障复位流程、断电恢复入口、
控制字/状态字语义。
**Owner 包/资产**：`src/rt_control/enable_manager`（含 `rt_disable_once`）。

不属于本区：驱动器侧 PDO/SDO 定义（→ ecat-axes）、启动脚本与确认短语
（→ release-deploy）、公共 `/control/set_enabled` 适配（→ contract）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | /rt/disable 挂起症状真实（4 例日志）；病因指向服务端 waitForResult 30s 自旋+RT 循环未填槽，非串行预算叠加 | [lifecycle-20260814-01](records/2026-08-14-rt-disable-log-forensics.md)#F1 | 有效 |
| 01#F2 | rt_disable_once 三阶段共享单一绝对 deadline 是已验证现状（29 例日志佐证），非缺陷 | [lifecycle-20260814-01](records/2026-08-14-rt-disable-log-forensics.md)#F2 | 有效 |
| 02#F1 | enable_manager 单元测试基线 = 36 用例 @ 3ff153d（a–f 六项）；行为修改须先过套件 | [lifecycle-20260814-02](records/2026-08-14-enable-manager-gtest.md)#F1 | 有效 |
| 03#F1 | enable_manager 命令 writer 来自显式非空无重复注册表 | [lifecycle-20260819-03](records/2026-08-19-electri-102-motion-controller-registry.md)#F1 | 有效（T1） |
| 03#F2 | 普通 enable 只保留 whole_body_jtc ACTIVE | [lifecycle-20260819-03](records/2026-08-19-electri-102-motion-controller-registry.md)#F2 | 有效（T1） |
| 03#F3 | 失能成功要求所有注册成员复核为 INACTIVE | [lifecycle-20260819-03](records/2026-08-19-electri-102-motion-controller-registry.md)#F3 | 有效（T1） |
| 03#F4 | controller 状态无法复核按 ambiguous/restart-required 收敛 | [lifecycle-20260819-03](records/2026-08-19-electri-102-motion-controller-registry.md)#F4 | 有效（T1） |

## 记录索引（倒序）

- 2026-08-19 [ELECTRI-102 enable_manager motion controller 注册表](records/2026-08-19-electri-102-motion-controller-registry.md) — feat，PASS（T1）
- 2026-08-14 [enable_manager 状态机表驱动 gtest（ELECTRI-93）](records/2026-08-14-enable-manager-gtest.md) — feature，PASS（工控机实跑 36/36）
- 2026-08-14 [/rt/disable 缺陷声称的日志取证：症状真实、机制误诊](records/2026-08-14-rt-disable-log-forensics.md) — investigation，PASS（T2 只读）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-DEV-NATIVE-002 系列中 reset/JTC/staged-disable 各 corrective、
  T-IF-RT-003 (reset staged-disable corrective)
- `docs/xmc-updown-enable-commissioning-20260727.md`、
  `docs/ethercat_enable_disable_commissioning.md`
- 相关 BQ（不完全）：BQ-006（rt_watchdog 移除）、BQ-114
