# robot_driver 协作与提交指南

团队协作的入口。每条规则这里只讲要点，细节以链接的权威文档为准；本文与权威文档不一致时以权威文档为准，并改本文。

## 0. 文档索引

| 想知道 | 看哪里（权威） |
| --- | --- |
| 分层、事实归属、改什么去哪、硬指标、编码与测试原则 | [`ARCHITECTURE.md`](../ARCHITECTURE.md) |
| 安全底线、硬件与宿主授权、验证分级、提交纪律、何时必须暂停 | [`AGENTS.md`](../AGENTS.md) |
| 分支、Issue、commit、PR、合并、CI、权限 | 本文 |
| 每类改动的验证命令、`arch_check` 规则表 | [`tools/README.md`](../tools/README.md) |
| 包与层对照、数据流、启动入口 | [`domains/rt_control/README.md`](../domains/rt_control/README.md) |
| 功能区归属与开发记录规则、记录模板 | [`domains/rt_control/docs/areas/`](../domains/rt_control/docs/areas/README.md) |
| 阻塞问题与用户裁决 | [`domains/rt_control/BLOCKED-questions.md`](../domains/rt_control/BLOCKED-questions.md) |
| 实机验证时间线 | [`domains/rt_control/PROGRESS.md`](../domains/rt_control/PROGRESS.md) |
| 架构规则例外 | [`tools/arch_rules.yaml`](../tools/arch_rules.yaml) |
| 公共接口变更流程 | `robot_interfaces` 仓库的 `docs/robot_interfaces_ai_change_playbook.md` |
| 容器、宿主、发布 | [`docker/README.md`](../docker/README.md)、[`hostsetup/README.md`](../hostsetup/README.md) |

文档之间冲突、或与已裁决问题冲突时：停下相关工作，在 Issue 中写明冲突并请求裁决，不自行取舍。

## 1. 架构速览

- **分层**（包名前缀即层）：`infra_` → `hwcfg_` → `hw_` → `ctrl_` → `integ_`，只能向下依赖；`periph_`（独立外设节点）只依赖 `infra_`，只有 `integ_` 可以依赖它。任何包不得依赖 `integ_bringup`。`infra_` 新库要有至少 2 个现有调用方。
- **一个事实一个拥有者**：硬件事实在协议包（`hw_*`）；机型事实（轴、关节名、模块、几何）在 `hwcfg_machines/machines/`；台套事实（零位、标定、部署）在 `hwcfg_machines/units/<unit>/`；策略在拥有者包且不带机型名；机器人模型和限位在 `robot_description`；公共接口在 `robot_interfaces`，公共 ROS 名在 `integ_composer/config/interface_naming.yaml`。同一事实出现在两处就是缺陷。
- **跨包只用 4 种方式**：ros2_control 接口名、ROS 契约（topic/service/action）、composer 显式绑定、共享叶子库函数调用。
- **入口校验一次**：composer 生成 artifact 时完成全部校验，launch 和控制器信任 artifact。
- **改什么去哪**：查 `ARCHITECTURE.md` §2.1 的对照表；表里没有的改动类型，先补表再动手。

## 2. 一次改动的流程

```text
Linear Issue + GitHub Issue → 分支 → worktree → 本地验证 → PR → 评审与 CI → squash 合并 → 记录
```

### 2.1 Issue（Linear 与 GitHub 两边都建）

- 任务不得只在群里或口头安排。每个交付任务同时有一个 Linear Issue 和一个 GitHub Issue，两边互相引用：
  - GitHub Issue 标题以 Linear ID 开头，如 `[ELECTRI-140] 电池双总线通信`；正文第一行写 Linear 链接。使用 [`ISSUE_TEMPLATE/task.md`](ISSUE_TEMPLATE/task.md)。
  - Linear Issue 中贴 GitHub Issue 链接。
- 分工：Linear 承载背景、验收标准、阻塞、裁决与进度评论；GitHub Issue 承载与代码直接相关的讨论，并由 PR 关闭。两边结论不一致时以 Linear 为准，并回填 GitHub。
- 跨域接口任务先在 `robot_interfaces` 开 design Issue 承载裁决。
- 例外：架构规范及其门禁工具（`ARCHITECTURE.md`、`tools/arch_check.py` 等）的改动用 `feature/arch-<slug>` 分支，不建 Issue（用户 2026-10-28）。

### 2.2 分支与 worktree

- `main` 是唯一长期分支，也是 V3 主线。禁止直接 push、force push 和改写共享历史。
- 工作分支：`feature/<slug>`、`bugfix/<slug>`、`hotfix/<slug>`、`release/vX.Y`。`<slug>` 用看名字就懂的英文短语描述改动内容，如 `feature/base-control-mode`、`bugfix/kinco-stale-position`；不放 Linear ID（任务号写在 commit 和 PR 里）。小写、连字符分词，禁止 `tmp`、`wip`、`test` 等无信息名称。
- 一个 worktree 只有一个写入者；并行开发用各自的 worktree 和不重叠的范围。分支落后时把 `main` 合进来，不 rebase 后强推。

### 2.3 一个 PR 一个目的

- **结构变更**（改结构不改行为）与**行为变更**分开提 PR。
  - 结构变更：改前改后每个 scope 生成的 artifact 逐字节一致，PR 附 `tools/arch_check.py compose --against origin/main` 的结果。
  - 行为变更：PR 描述第一行写"行为变更"，列出新增、删除或语义改变的 topic/service/action 与参数。
- 范围过大或有互不相关的目的时拆成多个 PR 或子 Issue，不用更长的描述掩盖。

### 2.4 跨域接口与上游补丁

- 公共接口：先在 `robot_interfaces` 按 playbook 改契约（changelog 写全原因、提出人、影响域四要素，门禁与全包构建通过），合并后本仓库在同一个 PR 中更新 `deps.repos` 与 `src/interfaces/source-lock.yaml` 的 SHA。禁止在本仓库复制或修改公共 schema。
- 上游补丁：放 `patches/<上游>/NNNN-<slug>.patch`，一个补丁解决一个问题，对冻结提交能 `git apply --check`；同一 PR 中同步 `tools/bootstrap_native_dev.sh`（应用与校验两处）和 `docker/rt-control/Dockerfile`。

## 3. 编码规则

完整原则见 `ARCHITECTURE.md` §3–§4 与 `AGENTS.md` §3。

- **通用**
  - 外部数据只在入口校验一次，内部不重复防御；不为猜测的竞态或边界加复杂度（安全路径的边界处理除外，但要说清具体错误场景）。
  - 硬件 ID、对象字典、单位、方向、标定、安全阈值、CPU 拓扑不猜，也不给代码默认值；不知道就写 `TBD`，只阻塞受影响范围。
  - 启动必需的参数（如 `robot_variant`、`machine_unit`）必须显式给出，不设默认值。
  - 不留兼容层、别名和死代码；不用的包、配置、入口直接删除。
  - 不提前抽只用一次的函数；主流程只写正常路径，校验与细节放在下面的小函数。
- **硬指标**（`arch_check` 执行）
  - 单文件：C++ ≤ 1000 行、Python ≤ 600 行、YAML ≤ 400 行（生产源码与配置；测试、金样、上游代码不计）。
  - 第二份拷贝即抽取到 `infra_`，不允许第三份。
  - 机型名、关节名只出现在 `hwcfg_machines`；厂家/产品名只出现在 `hw_*`、`hwcfg_machines`、`periph_*`（注释也算）。
  - 例外只登记在 `tools/arch_rules.yaml`（规则、文件、原因、批准人、日期），新增例外须用户批准。
- **C++**：clang-format 14；实时路径禁止无界循环、阻塞 IO、动态内存分配、锁竞争、日志刷屏和动态参数查询；实时与非实时之间用预分配、有界的方式交换（原子量、无锁缓冲）。
- **Python**：ruff 0.11.13，`ruff format` 与 `ruff check` 都要通过。
- **配置 YAML**：数值旁写来源（实测、手册页、裁决人与日期）；配置必须过 schema 校验；YAML 文件之间不用无校验的路径字符串互指，通过 manifest 或 composer 绑定引用。
- **注释**：只写不明显的约束和反直觉行为，说明为什么；引用实测或裁决时写日期和人。
- **生成物**：不手改，改源头后重新生成。composer 金样变化要重算，并在 `integ_composer/test/golden/manifest.yaml` 登记原因、方法和用例。

## 4. 测试要求

- **少而精**：测真实实现的行为，不断言源码字符串、行号或文本片段；少用替换业务逻辑的 test double（`use_mock_hardware` 除外）。
- 安全路径（使能、失能、故障收敛、限幅、超时）的测试只合并不删除；没有调用方的工具连同它的测试一起删除。
- **提交前本地必跑**：
  1. `tools/quality_gate.sh`（与 pre-commit、CI governance 同一份，含 `arch_check`）；
  2. 受影响包及下游的 `colcon test`（日常可用 `tools/run_scoped_tests.sh`）；
  3. 结构变更另跑 `tools/arch_check.py compose --against origin/main`。

  其余按改动类型查 `tools/README.md`。
- **验证分级**：T0 静态与构建、T1 mock、T2 实机只读、T3 实机使能/失能不运动、T4 实机运动（定义见 `AGENTS.md` §6）；逐级升级，前一级失败不进入后一级；汇报时写到实际达到的那一级，编译、单元测试、mock、台架、实车分开写，没跑的写"未验证"。

## 5. 硬件与实机

- 代码任务不构成硬件授权。上电、使能、运动、写 SDO、启停总线、root 脚本、修改宿主都需要针对该动作的单独明确授权（`AGENTS.md` §5）。
- 每个硬件动作执行前先说明要做什么、会不会动；先做不使能的启动，再做使能，再做小幅低速运动。
- 台架或实车部署到新的独立目录，不覆盖正在使用的部署；停栈按部署脚本的有序退出方式，不直接杀进程。
- 实机结果写记录（见 §7），并在 `PROGRESS.md` 追加一行。

## 6. 提交与 PR

### 6.1 Commit

```text
<type>(<scope>): <中文描述> [<任务号>]
```

- `type`：`feat` `fix` `docs` `style` `refactor` `test` `chore` `perf` `ci` `revert`。
- `scope` 按资产：`rt-control`（`src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**`）、`interfaces`、`description`、`deploy`、`tools`、`ci`、`docs`、`governance`；也可用包名（如 `enable_manager`）。跨 scope 优先拆提交。
- `[任务号]` 写 Linear ID；无 Issue 的架构任务写 `arch-<slug>`。
- 一个提交只做一件可描述、可回滚的事；不夹带无关格式化、生成物、调试代码；禁止 `update`、`fix bug`、`wip` 之类描述。
- 提交前：`git status --short --branch`、`git diff --check`，逐行审查 `git diff` 与 `git diff --cached`（含未跟踪文件）。

### 6.2 Pull Request

- 标题沿用 commit 格式；描述按 [`pull_request_template.md`](pull_request_template.md) 填写，保留并勾选全部 `gate:*`（CI 校验）。
- 描述写明 Linear ID 和 `Closes #<GitHub Issue>`。
- 描述顶部放不超过 10 行的摘要，读者是不熟悉实现的同事：

  ```text
  **结论**：[已完成|部分完成|阻塞|待验证] 一句话：当前状态 + 对项目意味着什么
  **已完成**：最多 3 条，写效果
  **验证**：做了哪些验证及结果；未验证：没验证的关键项
  **风险/阻塞**：最多 2 条；没有写"无"
  **需要**：@谁 做什么；没有写"无"
  **细节**：Issue、记录、相关 PR 链接
  ```

  摘要不替代模板章节；没有实机证据时用"待验证"或"部分完成"。
- 实机未验证的实时或硬件相关 PR 先开草稿（draft），实机通过后再转为可合并。

### 6.3 合并要求

所有合并都要求：CI 中每个 workflow 的最新一次运行通过、本地 `tools/quality_gate.sh` 通过、无未解决讨论，使用 squash 合并。在此之上：

| PR 类型 | 评审要求 |
| --- | --- |
| 普通域内变更 | ≥ 1 approve |
| Robot Model 构建副本 | ≥ 2 approve，含 1 名核心负责人；全部消费域联合验证 |
| 公共接口 pin / source-lock | 接口所有者 + 全部消费域 approve；与上游契约同批更新 |
| `docker/**`、Compose、部署 | 平台/集成 + 受影响域 approve |
| 实时或安全边界（控制频率、CPU 隔离、capability、device） | 核心负责人 approve；附时序或隔离实测 |
| 人工封装或正式发布 | 记录 source SHA，附镜像构建、容器内启动和镜像身份 |

Agent 只有在用户针对该 PR 明确要求合并时才能执行合并（`AGENTS.md` §7）。

### 6.4 接口评审清单

修改 `rt_control_interfaces` 或公共接口 pin 时，PR 必须说明：单位/坐标系/时间戳/标识符；请求合法性与拒绝条件；终态、错误码与反馈；超时、取消、抢占、重复请求和重启语义；新旧版本兼容性与是否需要多容器原子升级；高频状态是否误用 service/action、长任务是否误用 topic/service。并列出全部生产者和消费者。

### 6.5 评审严重级别

| 级别 | 含义 | 处理 |
| --- | --- | --- |
| CRITICAL | 安全隐患、绕过硬安全链、泄露密钥 | 阻塞 |
| HIGH | 缺陷、破坏接口契约 | 合并前应修 |
| MEDIUM | 可维护性问题 | 考虑修复 |
| LOW | 风格或次要建议 | 可选 |

### 6.6 CI

| 事件 | 执行 |
| --- | --- |
| PR opened / synchronize / reopened / labeled | `rt-control-ci`：`governance`（pre-commit = `tools/quality_gate.sh` + PR 契约门禁） |
| PR 带 `full-ci` 标签 | 另跑完整 `build`（干净环境：上游依赖、补丁、IgH、全量构建与测试） |
| 每晚 23:00（北京时间；GitHub 可能推迟数小时启动）、手动触发 | 在 `main` 上跑完整 `build`；失败时由提交者第二天修复 |
| PR edited | `pr-metadata-governance`：只校验 PR 契约 |
| push(main) | 不运行 |

用户 2026-10-30 选定方案 A：PR 只等快速门禁，完整构建改为每晚。改动构建配置、包清单（`package.xml`/`CMakeLists.txt`/`deps.repos`/`patches/`/`docker/`/`.github/`/`tools/bootstrap_native_dev.sh`）的 PR 应打 `full-ci` 标签或由作者本地完成对应构建。本地通过 `pre-commit install` 调用同一门禁；未安装 hook 不构成豁免。普通源码 CI 不构建生产镜像。

### 6.7 权限

| Team | 范围 |
| --- | --- |
| `core-maintainers` | 仓库 maintain/admin；Robot Model、interfaces、部署与安全边界的最终裁决 |
| `rt-control-team` | `src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**` |
| `platform-integration` | `docker/**`、`deploy/**`、`tools/**`、`.github/**` |
| `interns` | write 权限，不得 approve 合并 |

路径级强制评审见 [`CODEOWNERS`](CODEOWNERS)。

## 7. 记录与裁决

- **开发记录**：行为变更或硬件事实变更在 `domains/rt_control/docs/areas/<area>/records/YYYY-MM-DD-<slug>.md` 新增一条（模板见 `docs/areas/TEMPLATE.md`），并更新该区 README 的事实表；纯结构/文档变更不记。合并后的记录不改，推翻结论用新记录加 `supersedes`。
- **实机时间线**：实机验证在 `PROGRESS.md` 追加一行，历史行不改。
- **阻塞与裁决**：`BLOCKED-questions.md` 只记缺少权威事实的阻塞问题和用户裁决，写入前须用户确认；解决后标 `RESOLVED` 并写明证据。
- **Issue 进度**：Linear 评论只写本次新增事实，用 §6.2 的摘要格式；PR 链接回填到 Linear 和 GitHub Issue。
- **必须暂停并请求裁决**的情形见 `AGENTS.md` §8：与已有规则或裁决冲突、需要猜测硬件或安全事实、破坏性接口变更的迁移顺序不明、需要未授权的硬件动作、构建或测试失败而无法支撑完成声明等。
