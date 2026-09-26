# release-deploy — 构建、镜像、发布与部署

**范围**：Docker 镜像与 entrypoint、compose 与运行时变量、发布物导出与校验、
版本/SHA 锁定（`versions.env`、`deps.repos`）、目标机部署与回滚、启动脚本与
确认短语入口、launch 结构。
**Owner 包/资产**：`docker/`、`tools/rt_control_compose.sh`、
`tools/bootstrap_native_dev.sh`、`versions.env`、`deps.repos`、
`src/rt_control/rt_control_bringup` 的 V3 launch 与启动脚本。旧履带
`rt_control_ipc.sh`/`rt_control_native.sh` 已退役。

不属于本区：控制器参数（→ motion）、宿主机配置（→ realtime-host）、CI 工作流
（→ governance）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 2026-09-24#F1 | 旧履带启动、差速控制、Leadshine CANopen 生成链及宿主 can0 已移除；上游补丁仅保留共享生命周期/RPDO 与 Lely 队列修复，编码器 CAN 身份仍 TBD | [旧履带运行链退役](records/2026-09-24-retire-track-runtime.md)#F1-F2 | UNVERIFIED（T0；隔离 9 包/150 测试、Docker 39 包与无设备静态 validation 通过；实机运行待验） |
| 07#F1 | 三代机底盘 scope 显式绑定 swerve_controller 与四个必需外置编码器；只有静态控制计划，真实后端/标定未准入 | [ELECTRI-117 底盘模块控制器绑定](records/2026-09-13-swerve-controller-module-binding.md)#F1-F3 | UNVERIFIED（T0 配置；无实机/容器启动） |
| V3-OLD-IPC-1 | `v3@1256885` 已旁路部署到旧工控机 `ar@192.168.100.40`，34 包/1697 测试/validation 通过；current link 未切换且未启动硬件 | [旧工控机 V3 旁路部署](records/2026-09-18-v3-old-ipc-sidecar-deployment.md)#F1-F3 | PARTIAL（T1；真实 runtime 不准入） |
| V3-ARM-RUNTIME-1 | V3 机械臂运动入口显式选择，JTC/rolling 先 INACTIVE、enable_manager 后 ACTIVE；默认入口仍为 validation | [V3 arm runtime 入口](records/2026-09-19-v3-arm-runtime-entry.md)#F1-F3 | PARTIAL（T1 Mock；旧机/真实总线未验） |
| V3-HEAD-CAN-1 | V3 头部使用 `head_joint/head_pitch_joint`、非 JTC ForwardCommand + `/rt/head/*` manager；达妙 T 型由 ACC/DEC/MAX_SPD 承担，参数工具默认不写 flash | [V3 达妙头部非 JTC 位置速度运行链](records/2026-09-19-v3-damiao-head-runtime.md)#F1-F4 | PARTIAL（T1 Mock；CAN/机械/整机 full runtime 待验） |
| PR25-INTEGRATION-1 | ELECTRI-118 已整合 main@8fc332f，保留 PREOP/CoE/Ti5 约束；32包完整镜像及旧机/三代机 Mock 有序退出通过 | [最新main集成与PR验证](records/2026-09-13-electri-118-pr-integration.md)#F1-F3 | PARTIAL（T1；新基线实机待验） |
| 01#F1 | bringup 只选择并组合 EtherCAT/CANopen 两个 package-owned real/mock system，配置对齐在 Node 创建前 fail closed | [release-deploy-20260903-01](records/2026-09-03-port-hardware-composition-to-main.md)#F1-F3 | PASS（T1 Docker/Mock） |
| 01#F2 | 历史：ecat_icube 0001..0006；ros2_canopen 0001..0005 的原补丁顺序已由 2026-09-24#F1 取代，不能按本行重建 V3 | [release-deploy-20260903-01](records/2026-09-03-port-hardware-composition-to-main.md)#F4 | HISTORICAL |
| 01#F3 | ecat_icube 当前补丁顺序为 0001..0007；0007 使 EtherLab 前缀默认兼容且可由 CMake CACHE PATH 重定位，`RT_CONTROL_ETHERLAB_PREFIX` 统一运行时库路径，旧 patched vendor 原地增量 prepare 需改用干净 workspace | [release-deploy-20260904-01](records/2026-09-04-electri-94-native-closure.md)#运维限制 | PASS（Native build/test + ldd） |
| 02#F1 | ELECTRI-118 machine manifest 统一登记三代机模块、五个 physical profile 和 control scope；当前为静态 draft，TBD 不得进入真实启动 | [release-deploy-20260907-01](records/2026-09-07-electri-118-machine-profiles.md)#F1-F5 | UNVERIFIED（T0 静态） |
| 03#F1 | owner_refs 按硬件协议登记，底盘包更名不删除头部 damiao_can；4 ms SYNC 为设计约束而非实机时序结论 | [舵轮关联修复](records/2026-09-08-swerve-owner-reference-corrective.md)#F1 | UNVERIFIED（T0 配置回归通过） |
| 04#F1 | swerve_driver 已进入 bringup 构建闭包，但未加入生产启动；增量容器测试不替代真实后端及整机验收 | [舵轮包集成](records/2026-09-09-swerve-package-integration.md) | PARTIAL（T1 增量容器） |
| 05#F1 | 三代机开发快照已在用户指定测试主机隔离部署，30 包构建和相关测试通过；真实启动仍受 topology/runtime 与 IgH 补丁准入限制 | [测试机部署](records/2026-09-12-gen3-test-host-deployment.md) | PARTIAL（T1；未使能） |
| 06#F1 | ELECTRI-118 已在隔离同步分支整合 main@34acbae，五个冲突文件已解决；周期钩子改0013、PP事项改BQ-147，原部署不变 | [main同步记录](records/2026-09-12-electri-118-main-sync.md) | UNVERIFIED（同步/配置验证；新基线构建实机待验） |

## 记录索引（倒序）

- 2026-09-24 [旧履带运行链退役](records/2026-09-24-retire-track-runtime.md)：UNVERIFIED（T0；CANopen 150 测试、Docker 39 包及静态 validation 通过；实机运行待验）。

- 2026-09-19 [V3 机械臂运动 runtime 显式入口](records/2026-09-19-v3-arm-runtime-entry.md)：PARTIAL（T1 Mock；未部署）。
- 2026-09-19 [V3 达妙头部非 JTC 位置速度运行链](records/2026-09-19-v3-damiao-head-runtime.md)：PARTIAL（T1 Mock；CAN/机械/整机 full runtime 待验）。
- 2026-09-18 [V3 分支旁路部署到旧工控机](records/2026-09-18-v3-old-ipc-sidecar-deployment.md)：PARTIAL（T1；未切换/未启动硬件）。
- 2026-09-18 [V3 专用分支、模型与 fail-closed 运行边界](records/2026-09-18-v3-branch-baseline.md)：PARTIAL（T1 validation；整机 runtime/实机待验）。
- 2026-09-13 [ELECTRI-117 底盘模块控制器绑定](records/2026-09-13-swerve-controller-module-binding.md)：UNVERIFIED（T0，静态接入，实机门禁保留）。
- 2026-09-13 [ELECTRI-118对齐PR25后的main与容器验证](records/2026-09-13-electri-118-pr-integration.md)：corrective，PARTIAL（T1）。
- 2026-09-12 [ELECTRI-118同步main与五处冲突解决](records/2026-09-12-electri-118-main-sync.md)：源码同步完成，最终发布验证待做。
- 2026-09-12 [三代机测试主机隔离部署](records/2026-09-12-gen3-test-host-deployment.md)：feature，PARTIAL。
- 2026-09-09 [swerve_driver 构建与增量容器验证](records/2026-09-09-swerve-package-integration.md)：feature，PARTIAL。
- 2026-09-08 [舵轮设计更新的协议关联修复](records/2026-09-08-swerve-owner-reference-corrective.md)：corrective，配置回归通过，包迁移待实施。
- 2026-09-07 [ELECTRI-118 三代机模块化物理 Profile 与控制范围校验骨架](records/2026-09-07-electri-118-machine-profiles.md) — feature，UNVERIFIED（T0 静态配置；真实硬件接入待后续）
- 2026-09-04 [ELECTRI-94 Native 依赖闭包与可移植 EtherLab 前缀](records/2026-09-04-electri-94-native-closure.md) — corrective，PASS（T1 Native；Docker/实机待验）
- 2026-09-03 [将 ELECTRI-94 硬件配置分层移植到双 X503 main](records/2026-09-03-port-hardware-composition-to-main.md) — feature，PASS（T1 Docker/Mock；CI/实机待验）
- 2026-08-19 [Franka 风格双硬件插件组合入口（未合并分支历史）](records/2026-08-19-franka-style-hardware-composition.md) — feature，历史 T1；当前由 01#F1/F2 取代

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-021/T-022 系列、T-DEV-NATIVE-001/002 中启动与 parity 系列、
  T-REL-010（V0.10 发布物策略）
- `docs/deployment-operations-runbook.md`、`docs/one-command-start.md`、
  `docs/native-development-workflow.md`、`docs/docker-deployment-performance-summary.md`
- 相关 Linear：ELECTRI-75（V0.10 发布）
