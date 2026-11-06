# robot_driver 架构规范

本文是 `robot_driver` 后续一切结构性改动的裁判。与 `AGENTS.md` 冲突时，架构与编码问题以本文为准；实时、安全、硬件事实、上游补丁与 PR 门禁的既有底线不因本文放宽（§6）。

按本文执行的迁移已于 2026-11-06 完成，迁移待办文件已删除（最后版本见 `1ba008a` 的 `domains/rt_control/docs/migration-backlog.md`）。

## 0. 目标（五条需求的复述）

| # | 需求 | 复述 |
| --- | --- | --- |
| 1 | AI 易读、易增删改查 | 一个没有记忆、上下文有限的执行者，面对任何改动，都能从目录结构直接定位唯一权威位置，一次改全，并用一条本地命令验证。文档只回答"为什么"，不回答"去哪改"。 |
| 2 | 解耦 | 每个事实只有一个拥有者，每个边界只用一种方式跨越。换硬件、换机型、并行改、单独跑，是同一原则的四个结果。 |
| 3 | 复用 | 复用三级：上游 → 仓库内共享叶子库 → 自己写。出现第二份拷贝必须抽取，不允许第三份。 |
| 4 | 人类易读 | 一张能背下来的地图、看名字就知道职责的命名、按模块索引的"为什么"。单文件有上限。 |
| 5 | 补充 | 死活分明、改动可局部验证、规范自身可执行、迁移可分步可回退、既有底线不动。 |

1 与 2 是同一原则的两个视角（唯一权威位置 = 唯一拥有者）；3、4 在本仓库中被 1、2 覆盖大半；5 是 1–4 守得住的前提。

## 1. 架构原则

### 1.1 唯一拥有者

- 硬件事实（PDO/SDO、状态字、模式号、EDS/ESI、波特率）只在协议包。
- 机型事实（轴数、关节名、顺序、模块组合、几何）只在 `hwcfg_machines`。
- 台套事实（零位、标定、部署）只在 `units/<unit>/`。
- 策略事实（使能批次、限幅、容差）只在策略拥有者包，但**不得带机型名**；机型选择哪份策略由 `machine_config` 的 manifest 引用。
- 公共契约只在 vendored `robot_interfaces`；域内接口只在 `rt_control_interfaces`。

违反判据：同一事实在两个文件出现，且改一处不改另一处会导致门禁失败或运行差异。

### 1.2 唯一跨越方式

跨包只允许四种方式：ros2_control 接口名、ROS 契约（topic/service/action）、composer 显式绑定（`controller_parameter_bindings` / `parameter_exports`）、共享叶子库的函数调用。

禁止：共享字符串常量表达机型事实（如 `kJointNames[14]`）、配置文件之间用路径字符串互指而无校验、"我知道你叫什么名字"式的 controller 名硬编码。

### 1.3 分层与依赖方向

保留现有四层模型，新增一层基础设施：

```
infra（共享叶子库：传输、协议解码、实时基础件、renderer 骨架）
  ↑
hw_config（机型/台套数据，无代码 API）
  ↑
protocol（总线与设备适配、renderer）
  ↑
control（控制器、广播器、外设节点）
  ↑
integration（composer、bringup、适配层、诊断、操作面）
```

- 下层不得依赖上层；`infra` 不依赖任何域包、不含 ROS 节点、不含策略、不含机型。
- control 只经 `hw_semantic_components` 依赖 protocol：该包定义硬件层与控制器共用的状态/命令接口语义（CiA402 状态字、握手式使能阶段、力传感器），控制器不直接依赖具体协议包（`tools/rt_control_layer_gate.py` R2 已执行）。
- 测试与 golden 生成脚本同样遵守方向。
- 包名用统一前缀表达所属层，看名字即知层，门禁按前缀检查：`infra_`、`hwcfg_`、`hw_`、`ctrl_`、`periph_`（独立外设节点，不与 protocol/control 互相依赖）、`integ_`。包与层的对照见域 [`README.md`](domains/rt_control/README.md)。

### 1.4 死活分明

每个包、每个配置文件、每个 launch 入口必须声明状态：`production | draft | retired`。

- `retired`：删除，或移入 `legacy/` 并从构建、测试、门禁中摘除。不允许靠 SHA 冻结 + 测试引用把死代码留在树里。
- `draft`：不得被 `production` 入口引用。
- 文档同理：自称"历史"的文档不得作为推荐入口。

### 1.5 改动可局部验证

每一类改动对应一条本地命令，秒级返回语义化结果（"`left_joint7` 在 enable_policy 缺失"而非"golden 对比失败"）。契约测试只断言行为，不断言源码字符串。

### 1.6 入口校验一次

外部数据只在入口层校验一次。本仓库的入口是 composer：artifact 生成时完成全部 schema 与交叉校验；launch 与控制器信任 artifact，不再重新 compose 比对。内部传递已确定类型的值，不重复防御。

## 2. 改动类型对照表与验证命令

验证入口为 `tools/arch_check.py`（待实现），子命令与输出约定见 [`tools/README.md`](tools/README.md#arch_check-设计待实现)。

### 2.1 改动类型对照表

| 改什么 | 唯一位置 | 改完跑 |
| --- | --- | --- |
| 新增/修改一个设备型号 | `hwcfg_machines/devices/<device>.yaml` | `arch_check facts device <id>` + `arch_check compose …` |
| 新增/修改一个模块（轴组、传感器） | `hwcfg_machines/machines/<variant>/<module>.yaml` + manifest 索引 | `arch_check facts module <name>` + `arch_check compose …` |
| 修改台套零位/标定 | `hwcfg_machines/units/<unit>/` | `arch_check compose …` |
| 修改使能批次/策略 | 策略拥有者 `config/`（无机型名） | `arch_check names` + `arch_check compose …` |
| 新增一种总线/协议 | `hw_<bus>` 新包 + renderer 注册 | `arch_check layers` + `arch_check status` |
| 新增一个控制器 | `ctrl_<name>` 新包 + `controllers.yaml` 数据条目 | `arch_check layers` + `arch_check refs` |
| 新增一个外设节点 | `periph_<name>` 新包 | `arch_check layers` + `arch_check duplicates` |
| 新增一个启动模式 | manifest `scopes`（不新增 launch 文件） | `arch_check compose …` |
| 抽一个共享库 | `infra_<name>` 新包，进库条件见 §3 | `arch_check layers` + `arch_check duplicates` |
| 修改公共契约 | `robot_interfaces` 仓库 → `deps.repos` pin | 既有流程 |
| 任何改动提交前 | — | `arch_check all` + `run_scoped_tests.sh` |

表里没有的改动类型，先补表再动手。

## 3. 硬指标

| 指标 | 值 | 执行方式 |
| --- | --- | --- |
| 单文件上限 | C++ ≤ 1000 行、Python ≤ 600 行、YAML ≤ 400 行 | 门禁；存量超限已消化。唯一例外：`ctrl_enable_manager` 的实时使能状态机主文件（用户 2026-11-03 接受：状态机不拆，非实时部分移到同包独立文件后主文件仍可超限；使能流程不再扩展） |
| 复用三级 | 上游 → infra → 自写；第二份拷贝即抽取 | 人工审查 + 门禁查重复实现清单 |
| infra 进库条件 | ≥ 2 个现有调用方；或是与仓库外程序共享的二进制/内存布局约定（如 Runtime-LOG 共享内存环），此时本仓库调用方可只有 1 个 | 人工审查 |
| 状态标记 | 每包/配置/入口必须有 `status` | 门禁 |
| 跨包引用 | 只允许 §1.2 四种 | 门禁（扩展现有 layer gate） |
| 机型名出现位置 | 仅 `machine_config` 与 `units` | 门禁（现有 PRODUCT_NAMES 规则扩到全层） |

## 4. 编码与测试原则

### 4.1 复杂度

- 通用规则不为猜测出来、几乎不会发生的竞态或边界加复杂度。真觉得要加，先说清楚具体怎么出错、概率多大、要多写多少代码，用户同意后再做。不悄悄扩大改动范围。
- 此规则适用于内部数据流与非实时代码。安全路径（使能/失能/故障收敛、硬件命令限幅、总线超时）的**边界处理代码**不属于"猜测的边界"，按既有契约保留；但同样要求说清楚具体错误场景，不许用"安全"二字当万能理由。

### 4.2 校验

外部数据只在入口校验一次（§1.6）。往里传已确定类型的值，内部不做重复防御性检查。

### 4.3 函数与结构

- 不提前抽只用一次的函数。只有会复用、包住了确实复杂的逻辑、或代表一个能起清楚名字的概念时才抽。
- 主函数只写正常流程，校验和细节放到它下面的小函数里。

### 4.4 注释

只写不明显的约束和反直觉的行为。不给显而易见的代码写注释。

### 4.5 生成物

自动生成的文件不手改，改源头后重新生成。生成物必须能从源头确定性重建。

### 4.6 测试

- 测真实实现，不把业务逻辑抄一份进测试。
- 少用 mock。保留 `use_mock_hardware`（ros2_control 标准 mock 硬件后端，CI 无真机的唯一手段）；砍掉替换业务逻辑的 test double。
- 契约测试只断言行为，不断言源码字符串、行号或文本片段。
- `enable_manager` 的实时状态机不拆（用户 2026-11-03）；只把运动模式切换、参数读取、控制器切换客户端等非实时代码移到同包独立文件，类与逻辑不变，现有状态机测试即为安全网。

## 6. 不动的东西

本文不放宽以下既有契约：实时路径禁止阻塞/分配/锁竞争；硬件事实、安全阈值、CPU 拓扑不猜；上游只通过 `deps.repos` 锁定 + 窄补丁；PR 门禁与提交/推送授权规则；硬件操作需单独授权；软件诊断不替代急停/STO/安全链。

## 7. 改动规则

- 一个 PR 只做一个目的。结构变更不改行为：改前改后对每个 scope 生成的 artifact 逐字节一致（`arch_check compose` 实现前，用 `compose_artifact` 生成到临时目录做 `diff -r`）。
- 行为变更单独开 PR，描述第一行写"行为变更"，列出被删除的 topic/service/action。
- 遇到 §6 的底线、硬件事实，或本文没覆盖的情况：停下来问，不猜。
- 发现本文有错或不可执行：改本文（独立 PR），不绕过。
- 本文与其门禁工具的后续改动沿用 `feature/arch-<slug>` 分支，不走 Linear（用户 2026-10-28 裁定）。

迁移记录：2026-10-28 至 2026-11-06 按本文完成死代码清理、配置归属收敛、`infra` 层、超大单元拆分、包改名、诊断重做与 Rolling 精简。开工必读原定约 15 KB，现 `ARCHITECTURE.md` + 根 `AGENTS.md` 约 18 KB，用户 2026-11-06 决定不再瘦身。
