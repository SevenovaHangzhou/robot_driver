# release-deploy — 构建、镜像、发布与部署

**范围**：Docker 镜像与 entrypoint、compose 与运行时变量、发布物导出与校验、
版本/SHA 锁定（`versions.env`、`deps.repos`）、目标机部署与回滚、启动脚本与
确认短语入口、launch 结构。
**Owner 包/资产**：`docker/`、`tools/rt_control_ipc.sh`、`tools/rt_control_native.sh`、
`tools/rt_control_compose.sh`、`tools/bootstrap_native_dev.sh`、`versions.env`、
`deps.repos`、`src/rt_control/rt_control_bringup` 的 launch 与启动脚本。

不属于本区：控制器参数（→ motion）、宿主机配置（→ realtime-host）、CI 工作流
（→ governance）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | bringup 只选择并组合 EtherCAT/CANopen 两个 package-owned real/mock system，配置对齐在 Node 创建前 fail closed | [release-deploy-20260903-01](records/2026-09-03-port-hardware-composition-to-main.md)#F1-F3 | PASS（T1 Docker/Mock） |
| 01#F2 | ecat_icube 补丁顺序为 0001..0006，PR #16 fixed-PDO 0004 必须先于 HardwareInfo 0005/0006；ros2_canopen 为 0001..0005 | [release-deploy-20260903-01](records/2026-09-03-port-hardware-composition-to-main.md)#F4 | PASS（冻结 SHA apply-check + Docker build） |
| 01#F3 | ecat_icube 当前补丁顺序为 0001..0007；0007 使 EtherLab 前缀默认兼容且可由 CMake CACHE PATH 重定位，`RT_CONTROL_ETHERLAB_PREFIX` 统一运行时库路径，旧 patched vendor 原地增量 prepare 需改用干净 workspace | [release-deploy-20260904-01](records/2026-09-04-electri-94-native-closure.md)#运维限制 | PASS（Native build/test + ldd） |

## 记录索引（倒序）

- 2026-09-04 [ELECTRI-94 Native 依赖闭包与可移植 EtherLab 前缀](records/2026-09-04-electri-94-native-closure.md) — corrective，PASS（T1 Native；Docker/实机待验）
- 2026-09-03 [将 ELECTRI-94 硬件配置分层移植到双 X503 main](records/2026-09-03-port-hardware-composition-to-main.md) — feature，PASS（T1 Docker/Mock；CI/实机待验）
- 2026-08-19 [Franka 风格双硬件插件组合入口（未合并分支历史）](records/2026-08-19-franka-style-hardware-composition.md) — feature，历史 T1；当前由 01#F1/F2 取代

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-021/T-022 系列、T-DEV-NATIVE-001/002 中启动与 parity 系列、
  T-REL-010（V0.10 发布物策略）
- `docs/deployment-operations-runbook.md`、`docs/one-command-start.md`、
  `docs/native-development-workflow.md`、`docs/docker-deployment-performance-summary.md`
- 相关 Linear：ELECTRI-75（V0.10 发布）
