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
| 01#F1 | motion writer 启动链固定为 JTC INACTIVE → rolling INACTIVE → enable_manager ACTIVE | [release-deploy-20260819-01](records/2026-08-19-electri-102-mandatory-inactive-bringup.md)#F1 | 有效（T1） |
| 01#F2 | rolling provisional 包络由 spawner 参数文件注入安装态绝对路径 | [release-deploy-20260819-01](records/2026-08-19-electri-102-mandatory-inactive-bringup.md)#F2 | 有效（T1） |
| 01#F3 | 任一 mandatory-INACTIVE spawner 失败均停止 launch，不启动 enable_manager | [release-deploy-20260819-01](records/2026-08-19-electri-102-mandatory-inactive-bringup.md)#F3 | 有效（T1） |

## 记录索引（倒序）

- 2026-08-19 [ELECTRI-102 mandatory-inactive motion writer bringup](records/2026-08-19-electri-102-mandatory-inactive-bringup.md) — feature，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-021/T-022 系列、T-DEV-NATIVE-001/002 中启动与 parity 系列、
  T-REL-010（V0.10 发布物策略）
- `docs/deployment-operations-runbook.md`、`docs/one-command-start.md`、
  `docs/native-development-workflow.md`、`docs/docker-deployment-performance-summary.md`
- 相关 Linear：ELECTRI-75（V0.10 发布）
