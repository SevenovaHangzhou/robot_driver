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
| 01#F1 | 一个 controller_manager 组合 ecat_arms 与 canopen_mobile_axes 两个独立 hardware system | [release-deploy-20260819-01](records/2026-08-19-franka-style-hardware-composition.md)#F1 | 有效（Docker/T1） |
| 01#F2 | 当前受控 seam 仅允许 alfa_v1 与相同 logical ABI，不承诺任意电机数量/类型热切换 | [release-deploy-20260819-01](records/2026-08-19-franka-style-hardware-composition.md)#F2 | T1 通过；其他变体/HIL 待验证 |
| 01#F3 | 两个 owner-local descriptors 同时驱动 hardware Xacro 与 diagnostics topology；无独立 composition YAML，controller/safety 仍由各 owner 显式维护 | [release-deploy-20260819-01](records/2026-08-19-franka-style-hardware-composition.md)#F3 | 有效（Docker/T1） |
| 01#F4 | 新变体不得改变 BQ-122 的确定性停机顺序和 30 s 总 deadline | [release-deploy-20260819-01](records/2026-08-19-franka-style-hardware-composition.md)#F4 | 有效（最终镜像 Mock 已复核） |

## 记录索引（倒序）

- 2026-08-19 [Franka 风格双硬件插件组合入口](records/2026-08-19-franka-style-hardware-composition.md) — feature，PASS（Docker/T1；实机/HIL 未执行）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-021/T-022 系列、T-DEV-NATIVE-001/002 中启动与 parity 系列、
  T-REL-010（V0.10 发布物策略）
- `docs/deployment-operations-runbook.md`、`docs/one-command-start.md`、
  `docs/native-development-workflow.md`、`docs/docker-deployment-performance-summary.md`
- 相关 Linear：ELECTRI-75（V0.10 发布）
