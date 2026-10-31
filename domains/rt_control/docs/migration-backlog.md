# robot_driver 架构迁移待办

本文件是 [`ARCHITECTURE.md`](../../../ARCHITECTURE.md) 的迁移待办，内容为规范原稿的 §5、§5a、§8，章节号保持不变。每项做完由对应 PR 勾掉（`[ ]` → `[x]`）并注明 PR（§8.1–8.3 由 `feature/arch-s2-agents-slim` 完成）；全部完成后删除本文件。规则与迁移顺序以 `ARCHITECTURE.md` §7、§9 为准。

## 5. 现状已知违反清单（基于 `43d4c64`，作为迁移待办）

按 §7 的顺序消化；每项独立 PR、不改行为。

| 完成 | 类别 | 项 | 违反条款 |
| --- | --- | --- | --- |
| [x] | A 死活不分 | `robot_hw_ethercat/urdf/ecat.ros2_control.xacro`、`variants/alfa_v1.yaml`、`config/slaves/*`（二代）只被测试/golden 引用 | 1.4 |
| [x] | A | `plc_node` 与 `plc_io_modbus` 均产出 `PlcIoState`，均无生产入口 | 1.4、3 复用 |
| [x] | A | `rt_control_interface_runtime.launch.py` 自称 draft preview（arch-b2：用户 2026-10-30 裁定删除，连同 `rt_io.yaml`、`joint_limits.yaml`） | 1.4 |
| [x] | A | `onboarding-knowledge-map.md` 自称历史却是推荐入口（arch-s3d：移入域 README 的历史资料段） | 1.4 |
| [x] | A | `docs/areas/` 9 旧区 + 7 新区共存（arch-s3d：旧区移至 `docs/areas-archive/`） | 1.4 |
| [x] | B 多处落地 | `rolling_types.hpp` 的 `kJointNames[14]` 硬编码机型事实（arch-s6：用户 2026-10-29 裁定关节名只写在 machine_config；控制器改读 `joints` 参数，轴集合哈希按配置计算，对现有关节名与原协议常量逐字节一致） | 1.1、1.2 |
| [x] | B | `enable_manager/config/Alfa_V3_*`、`rolling_trajectory_controller/config/Alfa_V3_*`、`control_api_adapter/config/Alfa_V3_grippers.draft.yaml` 带机型名 | 1.1 |
| [x] | B | `controllers.yaml` 的 `authorities.source` 指向不存在的 `rt_control_bringup/config/machines/alfa_v3.yaml` | 1.2 |
| [ ] | B | 改一个关节需触碰约 20 个非测试文件 | 0 需求 1 |
| [x] | C 重复 | 3 套 Modbus TCP 客户端（`plc_io_modbus`、`modbus_tcp_rtu485`、`plc_node`）（`plc_node` 已删；#130、#131 统一到 `infra_modbus`） | 3 |
| [x] | C | 3 套 SocketCAN 封装（`robot_hw_can`、`bms_node`、`lpms_nav3_can`）（#128 统一到 `infra_socketcan`） | 3 |
| [x] | C | 3 个协议包 renderer 各自复制 `_share/_validate/_param/validate_*`（#148 `infra_renderer`） | 3 |
| [ ] | C | CiA402 状态字解码在 5 个包各写一份；`semantic_components` 未成为唯一出口 | 1.1、3 |
| [ ] | C | 新鲜度/超时、`DiagnosticArray` 构造在十余个包各写一份 | 3 |
| [x] | C | 5 个 launch 骨架相同；`rt_control_start` 6 个分支各带专属 shutdown（arch-s7/s8：运行入口统一为分阶段 scope 入口，旧参数保留为别名；待实机验证） | 3、§2"新增启动模式" |
| [ ] | D 超大单元 | `enable_manager_controller.cpp` 2998 行 / 69 方法；测试 2644 行单文件 | 3 硬指标 |
| [ ] | D | `rolling_trajectory_controller.cpp` 2007 行、`vacuum_adapter.py` 956 行（arch-b2 已删）、`manifest.yaml` 736 行（arch-d2 拆为主文件 + `manifest/{modules,profiles,scopes}.yaml`，均 < 400 行） | 3 硬指标 |
| [x] | D | `BLOCKED-questions.md` 2852 行 161 条，只能全文读 | 需求 4 |
| [ ] | E 规则过载 | 开工前必读文档约 1.6 MB；`tools/tests` 多个源码字符串契约测试 | 1.5、4.6 |
| [x] | F | `composer/test/golden/generate_baseline.py` 反向 import `bringup`；composer 测试 import 三个协议包但 `package.xml` 只声明一个（arch-f1：两个一次性历史生成脚本删除——它们读取的是 `source_commit` 旧检出的 bringup；补齐 `robot_hw_can`、`robot_hw_canopen` 测试依赖） | 1.3 |
| [x] | F | `controllers.yaml` 每个条目的 `config_file` 字段无任何读取方，且相对哪个包的 share 不明确（arch-s4 发现）；建议从 schema 与数据中删除（arch-cleanup：已删除） | 1.4、1.2 |
| [x] | F | `patches/ros2_canopen` 编号 0001、0002、0006、0007 缺号无说明（arch-cleanup：`patches/ros2_canopen/README.md` 记录退役补丁） | 1.4 |
| [ ] | F | `robot_description` 内多份展开 `.urdf` 与多套 `joint_limits_*`/`named_poses_*` 变体并存 | 1.1、4.5 |
| [x] | 6 重复校验 | `load_artifact` 运行时重新 compose 比对每个文件（arch-b5：用户 2026-10-30 裁定删除；artifact 自带运行时参数键与 preflight 拓扑，启动只校验哈希） | 1.6 |
| [ ] | 6 | `rolling_trajectory_controller` 运行时用 OpenSSL 算 envelope SHA | 1.6 |
| [x] | A | `rt_diagnostics` 无生产入口且含二代履带诊断（arch-b3：用户 2026-10-30 裁定先删除旧实现，重写待需求明确后另做） | 1.4 |
| [x] | A | `control_api_adapter` 4/5 适配器只被 draft 入口引用（arch-b2：vacuum/position_action/module_state/status 四个适配器删除，只留 `control_enable_adapter`） | 1.4 |
| [x] | B | `damiao_head_controller` 是 `enable_manager` 的第二份实现（arch-s7：头部作为握手式设备并入 enable_manager，包删除；待实机验证） | 1.1、3 |
| [x] | B | `semantic_components` 混入 `runtime_log_shm_ring`（infra）和 `head_phase`（用户 2026-10-30 修正：`head_phase` 是握手式使能设备的通用阶段语义，改名 `HandshakePhase` 留在本包；`runtime_log_shm_ring` 按 §3 例外移入 `infra_shm_ring`；#120、#121 完成） | 1.1、1.3 |
| [x] | C | 四个控制器包共同重复骨架（见 §5a 末；用户 2026-10-31 裁定不建 `infra_rt_utils`） | 3 |
| [x] | C | `rt_disable_once` 与 `head_disable_once` 同一件事两种写法（#110 头部并入 enable_manager、#111 删除 `rt_disable_once`，两者均已不存在） | 3 |
| [ ] | C | `rt_control_bringup/ros2_control_node.cpp` fork 上游 main | 3、§6 窄补丁 |
| [x] | D | `composer.compose()` 单函数 300 行（arch-d1：拆为加载校验→模型骨架→模块渲染→挂接硬件系统→控制器参数→输出与 lifecycle；1404 个组合输出逐字节不变） | 4.3 |
| [x] | D | `swerve_controller.cpp` 1042 行；`relative_move` 与控制器同包；mock 在 `src/`（arch-b1：用户 2026-10-30 裁定删除整个 relative 扩展——relative_move、ChassisRuntime、chassis_mode_handoff 与 6 个 IDL；`swerve_controller.cpp` 降至 893 行，mock 随之删除） | 3 硬指标、1.4、4.6 |

## 5a. 逐包处置表（基于 `43d4c64`）

层前缀：`infra_` / `hwcfg_` / `hw_` / `ctrl_` / `periph_` / `integ_`（六层，已确认）。原则：死的不改名先删；名字说职责不说传输；协议名只出现在 protocol 层。

| 完成 | 现名 | 层 | 处置 | 新名 | 审查结论 |
| --- | --- | --- | --- | --- | --- |
| [ ] | `robot_description` | 共享 | 保留 | 不改 | 构建副本内有 4 份展开 `.urdf`、4 套 `joint_limits_*`、3 套 `named_poses_*`、5 个 `*_manifest.json`；生成物不入库，本仓库只留一个 pin。 |
| [ ] | `rt_control_interfaces` | 共享 | 保留 | 不改 | 健康。9 个 IDL 均为 V3 在用。 |
| [x] | `rt_control_machine_config` | hw_config | 保留 | `hwcfg_machines` | （arch-r4 已改名；arch-d2 manifest 已拆分）结构对；114 处 draft/TBD/verified:false；`manifest.yaml` 736 行按 module 拆文件，manifest 只留索引。 |
| [x] | `robot_hw_ethercat` | protocol | 先删后改名 | `hw_ethercat` | （二代残留已删；x503 已删；arch-r3 已改名；#148 renderer 骨架已抽 infra）最杂：二代残留（xacro、`variants/`、`config/slaves/`、681 行 `validate_ethercat_variants.py`）约占 40%；3 个 renderer 各自复制 `_param`；吸收 `x503_force_sensor` 为 `tools/x503/`。 |
| [x] | `robot_hw_canopen` | protocol | 保留 | `hw_canopen` | （arch-r3 已改名；arch-d3 EDS 兼容件标注为原件 + diff 派生并由测试校验）健康。EDS `.compat.*` 三件套按 §4.5 标注生成源头。 |
| [x] | `robot_hw_can` | protocol | 保留 | `hw_can_damiao` | （arch-r3 已改名；SocketCAN 已由 #128 下沉 infra）健康。`socketcan.cpp` 作为 `infra_socketcan` 种子。 |
| [ ] | `rt_control_semantic_components` | protocol | 瘦身 | `hw_semantic_components` | （#120、#121 已瘦身；arch-r3 已改名；剩余：CiA402 唯一出口）名不副实：`runtime_log_shm_ring.hpp`（跨仓库拷贝，与 Runtime-LOG 共享内存布局，本仓库只供 enable_manager）→ `infra_shm_ring`（§3 例外）。`head_phase.hpp` 原判为达妙专属；头部并入 enable_manager 后它是“插件自执行使能”设备（`HandshakeDevice`）与硬件插件之间的通用阶段约定，与 `cia402_axis` 同类，改名 `handshake_phase.hpp`/`HandshakePhase` 留在本包（用户 2026-10-30 修正）。留 `cia402_axis`、`handshake_phase`、`force_torque_sensor`，并成为 CiA402 解码唯一出口。 |
| [ ] | `enable_manager` | control | 拆 | `ctrl_enable_manager` | （arch-r2 已改名；剩余：拆分）2998 行/69 方法；实时 FSM 留核心，模式切换/诊断/trace/controller 名解析搬出。拆前每条迁移一个行为测试。吸收 `damiao_head_controller`。 |
| [x] | `damiao_head_controller` | control | **合入 enable_manager** | — | 是第二个 enable_manager（`handle_enable/disable/reset + switch_position_controller + publish_diagnostics + encode_double`），只是 14 轴变 2 轴。应是同一控制器配不同 policy。`head_disable_once` 与 `rt_disable_once` 同理合并。 |
| [ ] | `rolling_trajectory_controller` | control | 拆 | `ctrl_rolling_trajectory` | （arch-r2 已改名；剩余：拆分、去 OpenSSL）`rolling_trajectory_core`（无 ROS 纯库 1800 行）健康；主文件 2007 行/80 方法/21 参数要拆；去掉 OpenSSL（envelope SHA 校验应在 composer 做一次）；`kJointNames[14]` 改为参数。 |
| [x] | `swerve_driver` | control | 拆 + 删 | `ctrl_swerve` | （arch-b1 已删 relative 扩展，`swerve_controller.cpp` 降至 893 行；arch-r2 已改名）内部 include 图干净。**删 `relative_move`**（session/planner/mock_node/mock_main 约 1000 行，及 `rt_control_interfaces` 中 `ChassisRelativeMove.action`、`ChassisMoveState.msg`、`ChassisMoveLimits.msg`；外部无消费者，仅 `bringup/test/test_cyclic_controller.cpp` 和 `tools/tests/test_rt_io_integration.py` 引用；行为变更，独立 PR）；拆 `swerve_controller.cpp`（1042 行）为 update 路径 + 生命周期两文件；`chassis_runtime` 去掉 relative_move 依赖后重评；`chassis_mode_handoff` 23 个过程方法改显式状态表。 |
| [x] | `rt_force_torque_broadcaster` | control | 保留 | `ctrl_force_torque_broadcaster` | （arch-r2 已改名）健康。 |
| [x] | `x503_force_sensor` | control | ~~并入 `hw_ethercat`~~ **删** | — | X503 PREOP 标定/SDO 快照脚本，纯协议工具。arch-b4：用户 2026-10-30 裁定 X503 只用于二代机，连同 `robot_hw_ethercat/config/x503b_readback.yaml` 删除。 |
| [x] | `bms_node` | periph | 保留 | `periph_bms` | （arch-r1 已改名，传输层已下沉 infra）`protocol/` 子目录无 rclcpp，健康。SocketCAN 下沉 infra。`dual_battery_controller` 是策略不是监控，但 opt-in 默认关，记一笔不动。 |
| [x] | `lpms_nav3_can` | periph | 保留 | `periph_imu_lpms` | （arch-r1 已改名，传输层已下沉 infra）健康。SocketCAN 下沉 infra。 |
| [x] | `modbus_tcp_rtu485` | periph | 保留 | `periph_led_ultrasonic` | （arch-r1 已改名，传输层已下沉 infra）健康。现名是传输不是职责；Modbus 客户端下沉 infra。 |
| [x] | `plc_io_modbus` | periph | 保留 | `periph_plc_io` | （arch-r1 已改名，传输层已下沉 infra）健康。Modbus 客户端下沉 infra。 |
| [x] | `plc_node` | periph | **删** | — | 与 `plc_io_modbus` 重复，Python 版，无生产入口。 |
| [x] | `rt_control_composer` | integration | 保留 | `integ_composer` | （#133 去反向引用、#136 拆 compose、arch-r5 已改名）纯函数 + schema 齐全，结构好。`compose()` 单函数 300 行拆为加载→校验→渲染→绑定→输出五段。测试反向 import bringup 要改。 |
| [ ] | `rt_control_bringup` | integration | 瘦身 | `integ_bringup` | （旧 launch、多入口、rt_io.yaml/joint_limits.yaml 已删；arch-r5 已改名；剩余：`ros2_control_node.cpp` fork）5 个旧 launch 删，只留 `scope_runtime`；`rt_control_start` 6 个 `--xxx` 收敛为 `--scope`；`ros2_control_node.cpp` 是上游 fork（188 行）→ 改补丁或论证；`rt_io.yaml`、`joint_limits.yaml` 只被 draft 入口用。 |
| [x] | `control_api_adapter` | integration | 隔离或删 4/5 | `integ_api_adapter` | （arch-b2 已删 4 个；arch-r5 已改名）5 个适配器中 4 个（vacuum 956 行、position_action、module_state、status）只被 draft 入口引用；仅 `control_enable_adapter` 在生产路径。 |
| [ ] | `rt_diagnostics` | integration | **重做** | `integ_diagnostics` | （arch-b3 已删旧实现；剩余：按 V3 需求重写）无任何生产入口引用；`canopen_native_diagnostic.cpp` 仍解析二代履带 NMT/EMCY。重做为 V3 诊断聚合器：输入为各控制器/硬件的 `/diagnostics` 行，输出按 scope 聚合的就绪/故障摘要；拓扑从 artifact 派生，不维护第二份设备清单；先删现有实现再按 V3 需求重写，不在旧代码上改。 |
| [ ] | `rt_control_operator_web` + `rt_control_arm_console` | integration | **合并** | `integ_operator_web` | 两套 token/aiohttp/controller_manager 桥接，重复约 1/3。一个服务多个页面。 |
| [x] | （新增） | infra | 按清单 C 逐个创建 | `infra_socketcan`、`infra_modbus`、`infra_renderer`（`_share/_validate/_param/validate_*` 骨架；arch-c5 已建：`validated_spec`、`xml_param`，`_share` 因测试替换与包专属而保留）、`infra_shm_ring` | 每个库一个 PR，进库条件 ≥ 2 现有调用方。`infra_shm_ring` 已由 #121 建立（§3 例外）。（用户 2026-10-31 裁定：不建 `infra_rt_utils`；CiA402 唯一出口在 `hw_semantic_components`（#146），不另建 `infra_cia402`） |

净效果：24 包 → 删 1（`plc_node`）、合 3 为 1（web×2、head→enable_manager）、并入 1（x503）、重做 1（`rt_diagnostics`）、新增 infra 5–6，最终 25–26 包，每包职责单一。

### 四个控制器包的共同重复骨架

`enable_manager`、`damiao_head_controller`、`swerve_driver`、`rolling_trajectory_controller` 各自实现：`encode_double/decode_double`（double 塑进 atomic<uint64>）、`steady_now`、`wait_for_result`（controller switch 同步等待）、`publish_diagnostics`、`fault_latched_` 原子标志组。~~进 `infra_rt_utils`~~。处置（用户 2026-10-31 裁定不建 `infra_rt_utils`）：`damiao_head_controller` 已并入 enable_manager；double 编解码改用 `std::atomic<double>` 删除（#147）；`steady_now` 每份 3 行、底盘一份语义不同，留在各包；等待控制器切换只 enable_manager 一处；诊断行随 `rt_diagnostics` 重写再定。

## 8. 本规范取代的现有条款

五份治理文档合计 74 KB（根 AGENTS 11.5K、域 AGENTS 22K、协作规范 21.7K、根 README 5.4K、域 README 13.6K）。目标：开工必读 = `ARCHITECTURE.md` + 根 `AGENTS.md` ≈ 15 KB。

处置代码：**吸收** = 内容进本规范某节，原处删；**保留@根** = 留在瘦身后的根 AGENTS；**门禁** = 转为可执行检查，文字删；**作废** = 过时或重复，删；**下沉** = 移到按需查阅的工具/流程文档，不在必读。

### 8.1 根 `AGENTS.md`

| 完成 | 章节 | 处置 | 说明 |
| --- | --- | --- | --- |
| [x] | 1 适用范围 | 保留@根 | 压缩到 3 行 |
| [x] | 2 开始任务前 + 2.1 路由表 | 作废 | 必读清单由 §7 步骤 2 重定；路由表被 §2 改动对照表取代 |
| [x] | 3 架构底线 | 吸收 → §1、§6 | 域边界一段保留@根 |
| [x] | 4 分支与封装（4.1–4.3） | 作废 | `native`/`v3` 已不存在，`main` 是唯一主线；封装规则下沉到 `docker/README` |
| [x] | 5 共享包与权威事实源 | 吸收 → §1.1 | Robot Model/Interfaces 所有权各留一句 |
| [x] | 6 配置、密钥和安全 | 保留@根 | 安全底线，已经很短 |
| [x] | 7 通用变更流程 | 作废 | 与域 AGENTS §4 重复；由 §2 + §4 取代 |
| [x] | 8 验证与提交底线 | 门禁 + 保留@根 | 命令清单进 `quality_gate.sh`；授权规则留 5 行 |
| [x] | 9 必须暂停 | 保留@根 | 三份文档各有一份，只留这一份 |
| [x] | 10 交付自审格式 | 下沉 | 进 PR 模板 |
| [x] | 七个职责功能区 | 作废 | 由 §1.3 六层 + §5a 包表取代 |

### 8.2 域 `domains/rt_control/AGENTS.md`

| 完成 | 章节 | 处置 | 说明 |
| --- | --- | --- | --- |
| [x] | 1 适用范围 | 作废 | |
| [x] | 2 必读与检查（7 步） | 作废 | 被 §7 步骤 2 的两文件必读取代 |
| [x] | 3.1 目录职责 | 吸收 → §5a | 包清单已过时（仍写履带 CANopen） |
| [x] | 3.2 依赖方向 | 吸收 → §1.3 | |
| [x] | 3.3 robot_description | 吸收 → §1.1 | 留一段 |
| [x] | 3.4 接口契约 | 吸收 → §1.1 + §6 | 评审清单下沉到 PR 模板 |
| [x] | 3.5 上游补丁 | 保留@根 | 窄补丁规则 |
| [x] | 3.6 实时与安全边界 | 保留@根 | 含"不得恢复"裁决清单，是真正的约束 |
| [x] | 3.7 容器与宿主 | 下沉 | 进 `docker/README`、`hostsetup/README` |
| [x] | 4 标准变更流程（9 步） | 作废 | 与根 §7 重复；"每次变更在 areas 新增记录 + PROGRESS 追加一行"改为只在行为变更或硬件事实变更时记录 |
| [x] | 5.1 最低门禁 | 门禁 | 命令进脚本 |
| [x] | 5.2–5.6 各类验证命令 | 下沉 | 进 `tools/README` 按改动类型索引，与 §2 对照表对齐 |
| [x] | 5.7 Mock 与实机分级（7 级） | 保留@根 | 压缩为一张表 |
| [x] | 6 提交推送 | 作废 | 与根 §8 重复 |
| [x] | 7 自审格式 | 作废 | 与根 §10 重复 |
| [x] | 8 必须暂停 | 作废 | 与根 §9 重复 |
| [x] | 七个职责功能区 | 作废 | |

域 AGENTS 瘦身后只剩 3.5、3.6、5.7 三段，并入根 AGENTS，文件删除。

### 8.3 `collaboration-and-commit-standards.md`

| 完成 | 章节 | 处置 | 说明 |
| --- | --- | --- | --- |
| [x] | 1–2 关系与仓库组织 | 作废 | |
| [x] | 3 分支模型 | 作废 + 保留@根 | `native`/`v3` 删；分支命名和保护留 5 行 |
| [x] | 4 Commit 规范 | 下沉 | 进 commit 模板；格式由 pre-commit 检查 |
| [x] | 5 提交前门禁 | 门禁 | |
| [x] | 6 PR 规范 | 下沉 | 进 PR 模板 |
| [x] | 7 Issue 管理 | 下沉 | 进 Issue 模板 |
| [x] | 8 版本与发布 | 下沉 | 进 `docker/README` |
| [x] | 9.1 通用风格 | 吸收 → §3 + §4 | **它已写明"文件 < 800 行、单函数 < 50 行"，从未执行**（enable_manager 2998 行）。是 §5c"规范自身要可执行"的直接证据 |
| [x] | 9.2 Python / 9.3 C++ | 门禁 | 已由 `style_gate.py` 执行，文字删，留版本号一行 |
| [x] | 9.4 ROS 2 包 | 吸收 → §1.2、§1.3 | |
| [x] | 9.5 测试三档 | 作废 | 被 §4.6 取代 |
| [x] | 10 CI | 下沉 | 进 `.github/README` |
| [x] | 11 权限与 Team | 下沉 | CODEOWNERS |
| [x] | 12 必须暂停 | 作废 | 第三份重复 |
| [x] | 13 禁止事项 | 保留@根 | 与根 §6/§8 合并去重 |
| [x] | 14 最终原则 | 保留@根 | 一句话 |

协作规范瘦身后不再是独立必读；剩余流程内容按对象下沉到模板和工具 README。

### 8.4 两份 README 与 `docs/areas`

| 完成 | 文件 | 处置 |
| --- | --- | --- |
| [x] | 根 `README.md` | 保留，删分支模型表；仓库结构图改为六层视图（分支表 arch-s2 已删；六层视图放在域 README，包改名后再同步根结构图） |
| [x] | 域 `README.md` | 重写为一屏地图（§0 需求 4）；"新同事从这里开始"里 5 份标注"历史"的文档移出入口 |
| [x] | `docs/areas/` 9 旧区 | 归档到 `docs/archive/`，从索引移除（实际为同层级 `docs/areas-archive/`，保持旧记录内相对链接有效） |
| [x] | `docs/areas/` 7 新区 | 归属按 §5a 包表重排；记录规则改为"行为/硬件事实变更才记"（arch-d5：arm-control+chassis-control 合为 control，新开 infra，各区 owner 改为现包名） |
| [x] | `BLOCKED-questions.md` 2852 行（arch-s5：用户裁定改为备份后清空、重新记录，写入前须确认；不再按模块拆分） | 按模块拆为 `docs/decisions/<module>.md`，只保留仍生效的裁决；已被取代的归档 |
| [x] | `PROGRESS.md` | 改为只索引实机验证记录，不再每 PR 一行（规则已改，历史行不动） |
