# RT-Control 功能区开发记录

本目录是 RT-Control 域按功能区组织的开发记录事实源。自 2026-08-13 起，所有开发、
调试、验收和裁决落地事件必须在对应功能区留下一条记录；
`domains/rt_control/PROGRESS.md` 自同日起降级为纯时间线索引，其 2026-08-13 之前的
表格内容冻结为历史段，不迁移、不修改。

## 功能区划分

| 功能区 | 范围 | Owner 包/资产 |
| --- | --- | --- |
| [ecat-axes](ecat-axes/README.md) | EtherCAT 14 轴伺服：拓扑、slave profile、PDO/SDO、极性 | `robot_hw_ethercat`、`patches/ecat_icube` |
| [canopen-chassis](canopen-chassis/README.md) | CANopen 履带底盘：bus 配置、EDS/DCF、换算、心跳 | `robot_hw_canopen`、`patches/ros2_canopen` |
| [lifecycle](lifecycle/README.md) | 使能/失能/故障复位/停机收敛/断电恢复 | `enable_manager`（含 `rt_disable_once`） |
| [motion](motion/README.md) | 轨迹与运动执行：JTC/FJT、底盘速度、控制器配置与限位 | `rt_control_bringup` 控制器配置、`patches/ros2_controllers` |
| [io-power](io-power/README.md) | PLC IO、真空执行、BMS 电池状态 | `plc_node`、`bms_node` |
| [contract](contract/README.md) | 公共契约与适配层：公共/私有接口、适配器、诊断归一化 | `control_api_adapter`、`src/interfaces/*`、`rt_diagnostics` |
| [realtime-host](realtime-host/README.md) | 实时调度与宿主：CPU 隔离、内核、亲和性、cyclictest | `hostsetup/`、`tools/rt_cpu_contamination_check.sh`、`tools/rt_control_thread_affinity.py` |
| [release-deploy](release-deploy/README.md) | 构建、镜像、发布物、目标机部署与回滚 | `docker/`、`tools/rt_control_ipc.sh`、`tools/rt_control_compose.sh`、`versions.env`、`deps.repos` |
| [governance](governance/README.md) | 门禁、CI、测试体系、协作规则与本记录体系自身 | `tools/quality_gate.sh`、`tools/repository_gate.py`、`.github/workflows/`、各级 `AGENTS.md` |

历史迁移工作（T-001 一族）已关闭，不设功能区；其证据保留在
`domains/rt_control/docs/baseline_report.md` 与 PROGRESS.md 历史段。

## 归属规则

1. **每条记录唯一归属一个功能区**，按被改动资产的 owner 包判定，与仓库
   "节点 owner 唯一" 原则一致。
2. 跨区改动拆成多条记录，各自归属，互相以 `related` 字段引用。
3. 拿不准归属时按主要改动落区，并在记录"背景"段说明取舍；禁止为省事新建功能区，
   新增功能区必须先在 governance 区留 decision 记录。

## 记录规则

1. **一条记录 = 一次可验证的变更事件**：一次任务收尾、一次实机调试、一次
   corrective、一次裁决落地，各写一条。不按 commit 拆分，也不把多次事件压进一条。
2. 记录文件放在 `<area>/records/YYYY-MM-DD-<slug>.md`，格式必须使用
   [TEMPLATE.md](TEMPLATE.md)：YAML frontmatter 字段齐全 + 五段正文标题不增不减。
3. **记录合并后不可修改**。结论被推翻时写新记录并在 `supersedes` 字段指向旧记录；
   旧记录原文保留。
4. 每区 README 的"冻结事实"表是该区当前有效结论的唯一可变视图；记录合入时必须
   同步维护：新增事实加行，被覆盖事实改状态并指向新来源。
5. 验证状态如实填写：mock/静态层结论不得冒充实机结论；没有证据的写 `UNVERIFIED`，
   后续实机验证补新记录。
6. 小改动同样走此格式，但允许正文合计 15 行以内；必填的是 frontmatter 字段和
   五个段标题，不是字数。
