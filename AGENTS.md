# Robot monorepo AI 协作与提交契约

## 1. 适用范围

本文适用于整个仓库，定义 rt-control、perception、motion、autonomy、gateway 五域共同遵守的架构、安全、质量和 Git 底线。

`domains/<domain>/AGENTS.md` 定义该域更严格的专属规则。域规则可以细化，但不得放宽根契约。一次变更同时影响多域时，必须同时阅读所有相关域的契约。

## 2. 开始任务前

1. 阅读根 `README.md`，确认五域责任、共享资产和当前导入状态。
2. 根据目标路径查看下表，阅读对应域的 `README.md`、`AGENTS.md`、`PROGRESS.md` 和 `BLOCKED-questions.md`。
3. 阅读目标包的 manifest、构建文件、实现、配置、测试和直接消费者；不只看被点名的单个文件。
4. 执行 `git status --short --branch`，区分用户已有改动与本任务改动，并确认当前分支是 `main` 还是 `native`（决定是否适用第 4 节的封装要求）。
5. 写明任务/需求号、所属域、目标包、跨域消费者、运行阶段和风险等级。
6. 跨域接口相关变更先读 `docs/cross-domain-interfaces.md` 的冻结基线。

### 2.1 当前路径路由

| 变更路径/内容 | 必读附加契约 | 主要联合评审 |
| --- | --- | --- |
| `src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**`、rt-control 上游补丁/工具 | `domains/rt_control/AGENTS.md` | rt-control；模型/接口变更时加相关域 |
| `src/description/robot_description/**` | 根契约第 5.1 节；再阅读所有受影响消费域的契约 | Robot Model + rt-control + motion + perception |
| `src/interfaces/**` | 所有生产者/消费者所属域契约 | 接口所有者 + 全部消费域 |
| `docker/compose.yaml`、`deploy/**`、共享 DDS/发布配置 | 所有受影响域契约 | 平台/集成 + 受影响域 |
| 未建立域契约的新域目录 | 先建立 `domains/<domain>/AGENTS.md` | 域负责人 + 平台/集成 |

`tools/**`、`docs/**` 和 `.github/**` 按实际管理的对象归属，不因其位于根目录就自动视为公共资产。

## 3. 五域架构底线

- rt-control 只负责硬件适配、轨迹/速度/IO 执行、生命周期、故障处理和状态反馈；250 Hz 控制环不得跨容器。
- perception 产生感知事实与任务几何决策，不直接执行运动。
- motion 负责 Nav2、MoveIt2/cuMotion/OMPL、碰撞世界和任务级运动 Action，不决定 Demo 任务顺序。
- autonomy 是唯一任务编排、顺序、取消、恢复和任务持久化责任域。
- gateway 是机器人对外通信出口；外部任务只能进 autonomy，不得绕过其直达 motion/rt-control。
- 域是责任边界，包是编译/复用边界，容器是部署边界。三者不必一对一，但不得借部署便利破坏责任边界。

跨域变更必须先定义调用方向、数据所有者、单位/坐标系/时间戳、超时、取消、幂等、错误码、重启和版本兼容语义。禁止以共享可写目录、对方内部库或新的私有 RPC 绕过冻结契约。

## 4. 分支与封装契约

仓库使用两条长期分支，责任不同：

| 分支 | 定位 | Docker 封装 |
| --- | --- | --- |
| `main` | 稳定集成与对外交付载体 | **必须**：每个已导入域都完成 Docker 封装 |
| `native` | 敏捷开发主线，源码增量迭代 | **不要求**：允许原生构建与宿主直跑 |

### 4.1 `main` 必须是封装好的各域

进入 `main` 的每个已导入域必须同时提供：

- 可复现的镜像构建定义（`docker/<domain>/Dockerfile`），版本与上游依赖按不可变标识固定；
- `docker/compose.yaml` 中的服务定义，含该域所需的最小 device、capability、cpuset、ulimits 和停机宽限；
- 容器内启动入口，不依赖宿主源码树、宿主绝对路径或人工前置步骤。

禁止把只能在宿主原生环境跑通的域合并进 `main`。合并到 `main` 的变更必须给出镜像构建与容器内启动证据；缺证据时按未验证处理，不得声明完成。

### 4.2 `native` 为敏捷开发豁免封装

`native` 允许 `--symlink-install` 增量构建、宿主直跑和原生一键启动脚本，暂不要求容器封装。豁免范围**只限容器封装本身**。

`native` 与 `main` 完全同等的要求：实时性与调度隔离、硬安全链与软件不越界、域责任边界与依赖方向、冻结的域间接口、共享 Robot Model 与 interfaces 的所有权、密钥与敏感数据禁令、`tools/quality_gate.sh` 及域级门禁、提交与自审格式。

不得以“`native` 是开发分支”为由降低上述任何一项。

### 4.3 `native` → `main` 的提升

从 `native` 提升到 `main` 时必须补齐容器封装，并确认没有把原生专用路径、宿主绝对路径、开发者本机配置或未封装启动方式带入 `main`。原生一键脚本可以保留，但不得成为 `main` 上该域的唯一启动方式。

两条分支都禁止直接 push、force push 和改写共享历史；全部变更经 PR 合并。人的协作流程细则见 `docs/collaboration-and-commit-standards.md`。

## 5. 共享包与权威事实源

### 5.1 Robot Model

`src/description/robot_description` 属于共享 Robot Model/平台基础层，不归任一业务域私有。

- 所有消费域必须使用同一版本/哈希的发行 artifact，禁止复制 URDF/mesh 后私有修改。
- URDF/Xacro 只保存机器人固有的运动学、几何、惯量、碰撞、坐标系和物理硬限位。
- PDO/SDO、驱动插件、控制器增益、规划参数、工位、抓取/放置/拍照位与现场标定不得进基准 description。
- link/joint 名称、TF 父子关系、轴向、零位、限制或 mesh 尺寸变更属于破坏性公共变更，必须联合验证所有消费者。

### 5.2 Interfaces

`src/interfaces` 只保存 msg/srv/action/schema 定义，不放业务实现。删除、重命名或改变字段类型、单位、坐标系、终态与错误语义均是破坏性变更；没有消费者迁移和原子发布方案不得合并。

冻结的域间接口清单、QoS 剖面、时效与成功语义见 `docs/cross-domain-interfaces.md`；该文件与 `src/interfaces` 必须同批更新。

## 6. 配置、密钥和安全

- 不得猜测硬件 ID、对象字典、单位/比例/方向、CPU 拓扑、标定、工位、运动/安全阈值或现场路径。缺少权威事实时只阻塞受影响范围，记录到所属域的 `BLOCKED-questions.md`。
- 代码、镜像、Git、日志和文档不得包含 token、私钥、证书私钥、密码、代理凭据或客户敏感数据。
- 容器默认最小权限；扩大 device、capability、网络、宿主写权限或 Docker socket 必须有明确需求、威胁评审和联合签核。
- 软件诊断不得代替急停、安全继电器、STO、驱动器保护和机械限位。
- 编译、mock、仿真和离线校验不得写成“实机验证通过”。

## 7. 通用变更流程

1. 界定所属域、目标文件、直接/跨域消费者和明确不改的内容。
2. 从权威源码、冻结依赖、图纸/设备文档、已裁决问题或实测记录取证。
3. 先定义验收条件，再做保持依赖方向的最小变更。
4. 补充与风险相匹配的自动测试，并验证成功、取消、超时、失败、重启和资源收尾路径。
5. 逐行审查工作树与暂存区，排除用户无关改动、生成物、调试旁路、敏感信息和未解释大文件。
6. 更新所属域的 `PROGRESS.md` 与必要文档，如实记录已验证、未验证和剩余风险。

用户已有改动必须保留。未经明确授权，不得通过 `git checkout --`、`git reset --hard`、清理工作树、覆盖文件或改写历史处理它们。

## 8. 验证与提交底线

所有变更至少执行：

```bash
git status --short --branch
git diff --check
git diff --stat
git diff
git diff --cached --check
git diff --cached
tools/quality_gate.sh
```

还必须执行所属域 `AGENTS.md` 要求的包构建、单测、契约、mock/仿真、容器或 HIL/实机门禁。公共模型或接口变更必须扩大到全部消费者，不得只验证修改者自己的包。

目标为 `main` 的变更还必须给出镜像构建与容器内启动证据。目标为 `native` 的变更免除容器封装，但第 4.2 节列出的其他要求一项不减。

提交或推送只在用户明确要求时执行。被授权后仍必须：

- 使用可追溯的 Conventional Commit 信息和任务/需求号；
- 提交前重新审查暂存区，推送前核对 remote、branch 和提交范围；
- 禁止 `--no-verify`、强制推送、擅自 amend 他人提交、改写共享历史或把多个无关任务塞入一个提交；
- 推送后报告远端分支、提交 SHA、实际验证和未验证项。

## 9. 必须暂停并请求裁决的情形

- 新需求与根契约、域契约、冻结接口或已裁决问题冲突；
- 需要猜测硬件、标定、安全或资源分配事实；
- 破坏性修改 Robot Model/跨域接口，但消费者、迁移和发布顺序不明；
- 需要扩大容器、设备、外网、Docker socket 或宿主权限；
- 需要运行 root 脚本、修改宿主机、启停总线、写设备、使能或驱动实机，但没有对该动作的单独授权；
- 用户改动与任务重叠且无法安全合并；
- 适用的构建/测试失败，或验证环境不足以支持当前完成声明。

## 10. 交付自审格式

```text
变更范围：
- 任务/需求：
- 所属域/公共资产：
- 文件/包：
- 明确未修改：

架构审查：
- 目标分支：main/native
- 域边界与依赖方向：PASS/FAIL
- Robot Model/接口兼容性：PASS/FAIL/N/A
- 分支封装要求（main 需容器证据）：PASS/FAIL/N/A
- 安全、生命周期与失败收尾：PASS/FAIL/N/A

验证证据：
- <实际执行的命令>：PASS/FAIL

未验证与剩余风险：
- <未执行项、原因、需要谁在什么条件下完成>

Git 审查：
- 无关改动：YES/NO
- 生成物或敏感信息：YES/NO
- 是否允许提交/推送：YES/NO（依据）
```
