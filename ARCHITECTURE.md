# robot_driver 架构规范

本文是 `robot_driver` 后续一切结构性改动的裁判。与 `AGENTS.md` 冲突时，架构与编码问题以本文为准；实时、安全、硬件事实、上游补丁与 PR 门禁的既有底线不因本文放宽（§6）。

迁移待办（§5 现状违反清单、§5a 逐包处置表、§8 被取代条款）不在本文，见 [`domains/rt_control/docs/migration-backlog.md`](domains/rt_control/docs/migration-backlog.md)；本文提到的 §5、§5a、§8 与清单 A–F 均指该文件。

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
- 机型事实（轴数、关节名、顺序、模块组合、几何）只在 `rt_control_machine_config`。
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
- control 只经 `rt_control_semantic_components` 依赖 protocol：该包定义硬件层与控制器共用的状态/命令接口语义（CiA402 状态字、握手式使能阶段、力传感器），控制器不直接依赖具体协议包（`tools/rt_control_layer_gate.py` R2 已执行）。
- 测试与 golden 生成脚本同样遵守方向（当前 `composer/test/golden/generate_baseline.py` 反向 import `bringup`，属违反）。
- 包名用统一前缀表达所属层，看名字即知层，门禁按前缀检查。具体前缀词与存量包重命名映射表另行讨论（open item）。

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

### 2.1 验证入口设计（待实现，§7 步骤 7）

单一入口 `tools/arch_check.py`，吸收现有 `rt_control_layer_gate.py`，不另起第二套。约定：

- 输出格式固定为 `path:line: RULE-ID 一句话`，RULE-ID 对应本规范章节（如 `OWN-1.1`、`SIZE-3`、`STATUS-1.4`）。
- 退出码 0 通过、1 有违反、2 工具自身错误。
- 只读源码树与安装的 share 目录，不访问硬件、网络、构建目录。
- 子命令按"改动类型"而非"规则类型"组织，让执行者改完只跑一条。
- 秒级返回；超过 10 s 的检查（如 colcon build）不属于本入口，由 `run_scoped_tests.sh` 承担。

| 子命令 | 检查什么 | 对应规则 |
| --- | --- | --- |
| `arch_check all` | 下列全部 | — |
| `arch_check layers` | 包前缀 ↔ 层一致；依赖方向（package.xml、C++ include、Python import、CMake find_package、测试同样计入）；跨包只用 §1.2 四种方式 | 1.2、1.3 |
| `arch_check status` | 每个 `package.xml`、`config/**.yaml`、`launch/*.py` 有 `status: production\|draft\|retired`；`production` 入口不引用 `draft`/`retired`；`retired` 不在构建闭包 | 1.4 |
| `arch_check size` | C++ ≤ 1000、Python ≤ 600、YAML ≤ 400 | 3 |
| `arch_check refs` | `machine_config`、`controllers.yaml`、manifest、policy 文件中所有 `package/file/path` 字符串引用可解析；`authorities.source` 指向存在的文件 | 1.2 |
| `arch_check facts <kind> <name>` | 列出一个事实（`joint`/`module`/`device`/`controller`）在源码树的全部出现位置，并按 §1.1 判定每处是"拥有者"还是"违反"；输出形如 `enable_policy.yaml:12: OWN-1.1 left_joint7 在策略文件中按名引用，应通过 managed_joints 绑定` | 1.1 |
| `arch_check names` | 机型名（`Alfa_V3`、`alfa_v3`）、协议名（`damiao`、`zeroerr`、`kinco`）只出现在允许的层 | 1.1、§5a 原则 |
| `arch_check duplicates` | 已知重复实现签名清单（SocketCAN open、Modbus 读保持寄存器、CiA402 状态字解码、`encode_double`、新鲜度判定）在非 infra 包出现即报 | 3 |
| `arch_check compose <variant> <unit> <profile> <scope>` | 内存 compose 一次并跑全部 schema/交叉校验，不写文件；是 `validate_machine_profile.py` 的替代入口 | 1.6 |

`facts` 的事实来源是 `hwcfg_machines` 的 manifest；工具不维护第二份清单。

### 2.2 改动类型对照表

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
| 单文件上限 | C++ ≤ 1000 行、Python ≤ 600 行、YAML ≤ 400 行 | 门禁；存量超限按 §7 顺序消化 |
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
- 拆分 `enable_manager` 之前，状态机每条已定义迁移保留一个行为测试作为拆分安全网（测已存在的逻辑，不测猜测场景；现有 2644 行测试文件照砍）。拆完后，新增迁移是否补测按 4.1 规则。

## 6. 不动的东西

本文不放宽以下既有契约：实时路径禁止阻塞/分配/锁竞争；硬件事实、安全阈值、CPU 拓扑不猜；上游只通过 `deps.repos` 锁定 + 窄补丁；PR 门禁与提交/推送授权规则；硬件操作需单独授权；软件诊断不替代急停/STO/安全链。

## 7. 迁移顺序

1. 本文定稿，迁入仓库为 `ARCHITECTURE.md`；`AGENTS.md` 加一行"架构与编码以 `ARCHITECTURE.md` 为准"。
2. 瘦身 `AGENTS.md`、域 `AGENTS.md`、协作规范：按 §8 清单吸收或作废条款（文档本身是最大的"死活不分"）。目标：开工前必读只剩 `ARCHITECTURE.md` + 根 `AGENTS.md`，合计约 15 KB；域 `AGENTS.md` 的规则要么并入、要么变门禁；`PROGRESS.md`/`BLOCKED-questions.md` 改为按模块索引、按需查。
3. 删死代码、收敛文档入口（清单 A）。
4. 收敛配置归属与机型事实（清单 B）。
5. 建 `infra` 层，抽公共库（清单 C），每抽一个库一个 PR。
6. 拆超大单元（清单 D），实时/非实时分界：实时状态机留小核心，模式切换、诊断、trace、controller 名解析搬出去。拆到撞上实时约束即为边界。
7. 门禁补齐（§3 硬指标），替换源码字符串契约测试。

行为变更项（不归入上述结构步骤，各自独立 PR，需单独说明影响）：删 `relative_move` 及其 3 个 IDL；重做 `rt_diagnostics`。

每步：独立 PR、不改行为、行为变更与结构变更不混。

## 9. 执行指引（给执行本规范迁移的 Agent）

本节是给按本文档执行的 Agent 的硬约束，优先级高于其自身习惯。

### 9.1 第一个 PR 的范围（§7 步骤 1）

只做三件事，不做其他：

1. 把架构规范原稿拆为两个文件落入仓库：
   - `ARCHITECTURE.md`（根目录）：§0、§1、§2、§3、§4、§6、§7、§9。去掉 frontmatter 与过程性状态标记。
   - `domains/rt_control/docs/migration-backlog.md`：§5、§5a、§8。每项带复选框，做完勾掉；全部完成后删除此文件。
2. 根 `AGENTS.md` 第 1 节末尾加一行：`架构、分层、编码与测试规则以 ARCHITECTURE.md 为准；本文件与其冲突时以 ARCHITECTURE.md 为准。`
3. `tools/repository_gate.py` 的 `REQUIRED_GOVERNANCE_FILES` 加入 `ARCHITECTURE.md`。

不改任何源码、配置、测试、其他文档。不在本 PR 里开始瘦身 AGENTS.md。

### 9.2 后续每个 PR 的通用约束

- 一个 PR 对应 `domains/rt_control/docs/migration-backlog.md` 里的一项或一组同类项，标题写明对应项。
- 结构变更 PR 不改行为：改前改后 `arch_check compose` 对每个 scope 的输出逐字节一致（`arch_check` 未实现前，用 `compose_artifact` 生成到临时目录做 diff）。
- 行为变更 PR（删 relative_move、重做 rt_diagnostics、删 draft 适配器）单独开，PR 描述第一行写"行为变更"，列出被删除的 topic/service/action。
- 每个 PR 在描述中写清：对应 backlog 项、改前/改后验证命令及结果、未验证项。PR 顶部摘要格式见 `.github/CONTRIBUTING.md` §3。
- 遇到 §6 列出的底线、硬件事实、或规范没覆盖的情况：停下来问，不猜。
- 发现规范本身有错或不可执行：改规范（独立 PR）而不是绕过。

### 9.3 停止点

以下动作完成后必须停下等用户确认，不自动进入下一步：

- §7 步骤 1 的 PR 开出后；
- 每个行为变更 PR 动手前；
- 任何需要删除 ≥ 1000 行代码的 PR 动手前；
- `arch_check` 首版实现完成、接入 `quality_gate.sh` 前（因为会让现状全红，需要先定 baseline 策略：按 §5 清单逐项消化，不做豁免清单）。

### 9.4 分支与工作树（本轮不走 Linear，用户 2026-10-28 裁定）

- 工作树：`repos/features/ARCH-spec-v1`
- 分支：`feature/arch-spec-v1`
- 后续迁移 PR 按步骤编号：`feature/arch-s2-agents-slim`、`feature/arch-s3-dead-code`、`feature/arch-s4-config-ownership`、`feature/arch-s5-infra-<lib>`、`feature/arch-s6-split-<pkg>`、`feature/arch-s7-arch-check`。
- 偏离说明：控制工作区约定要求工作树引用 Linear Issue；本轮用户明确不走 Linear，分支名以 `arch-` 前缀代替 Linear ID。若后续补建 Issue，分支改名为 `feature/<linear-id>-arch-…`。
- 基线：`main` @ `43d4c64` 或更新的 `origin/main`；开工前先 fetch 并记录实际 base SHA。
