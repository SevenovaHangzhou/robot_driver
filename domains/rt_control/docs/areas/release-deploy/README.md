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
| 01#F1 | 操作员 UI 使用独立非特权 Compose 服务，复用同版 RT-Control 镜像且不映射任何硬件 | [release-deploy-20260903-01](records/2026-09-03-operator-ui-container.md)#F1 | 有效 |
| 01#F2 | UI 容器固定非 root、housekeeping cpuset、host network/ipc、cap-drop ALL、restart=no | [release-deploy-20260903-01](records/2026-09-03-operator-ui-container.md)#F2 | 有效；镜像 T1 PASS，目标机 X11/UID/CPU 集待验证 |

## 记录索引（倒序）

- 2026-09-03 [操作员 UI 最小权限容器封装](records/2026-09-03-operator-ui-container.md) — feature，PASS（T1；目标机验收待完成）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-021/T-022 系列、T-DEV-NATIVE-001/002 中启动与 parity 系列、
  T-REL-010（V0.10 发布物策略）
- `docs/deployment-operations-runbook.md`、`docs/one-command-start.md`、
  `docs/native-development-workflow.md`、`docs/docker-deployment-performance-summary.md`
- 相关 Linear：ELECTRI-75（V0.10 发布）
