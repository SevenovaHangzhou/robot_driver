# robot_driver 协作与代码提交规范

> 版本：v1.0
> 适用范围：`SevenovaHangzhou/robot_driver` 仓库全部分支与全部贡献者
> 仓库边界：只保存 RT-Control 实现；其他域仅作为公共契约的外部生产者、消费者和评审方。

## 1. 与其他规范的关系

| 文档 | 管什么 |
| --- | --- |
| 本文 | 人与流程：分支、提交、PR、Issue、评审、发布、权限 |
| [AGENTS.md](AGENTS.md) | AI 与人共同遵守的架构、安全、质量底线和自审格式 |
| `src/vendor/robot_interfaces/contract/views/rt_control.md` | 随固定 SHA 导入的 RT-Control 域间契约视图 |
| `domains/rt_control/AGENTS.md` | RT-Control 更严格的专属规则 |

域规则可以细化根规则，**不得放宽**。本文与 `AGENTS.md` 冲突时，以更严格的一方为准。

## 2. 仓库组织方式

`robot_driver` 是 RT-Control 独立仓库。公共接口和 Robot Model 分别由独立
`robot_interfaces`、`robot_description` 仓库提供权威版本；公共接口通过
`deps.repos` 固定 SHA 导入，Robot Model 在本仓库保留构建副本。

```text
robot_driver/
├─ domains/rt_control/      # 实时域说明、AI 契约、进度与阻塞记录
├─ src/
│  ├─ description/          # robot_description 构建副本
│  ├─ interfaces/           # 仅 RT-Control 私有接口与公共契约 source-lock
│  └─ rt_control/           # 实时域实现包
├─ src/vendor/              # 构建时导入的固定 SHA 上游依赖（不入库）
├─ docker/                  # RT-Control 镜像与 Compose
├─ hostsetup/               # 宿主安装与验收
├─ patches/                 # 冻结上游窄补丁
├─ tools/                   # 实时域与仓库门禁工具
└─ collaboration-and-commit-standards.md
```

三个边界互相独立，不得借部署便利破坏责任边界：

- **域**是责任边界；本仓库只实现 RT-Control，其他域不得把内部代码放进来。
- **包**是编译与复用边界。
- **容器**是部署边界。

## 3. 分支模型

### 3.1 分支清单

| 分支 | 用途 | Docker 封装要求 |
| --- | --- | --- |
| `main` | RT-Control 稳定集成分支，对外交付载体 | **必须**：完成 RT-Control Docker 封装 |
| `native` | 敏捷开发主线，源码增量迭代 | **不要求**：允许原生构建运行 |
| `feature/xxx` | 新功能开发 | 跟随其基线分支 |
| `bugfix/xxx` | 普通缺陷修复 | 跟随其基线分支 |
| `hotfix/xxx` | 现场或演示前紧急修复 | 跟随其基线分支 |
| `release/vx.y` | 版本冻结与发布准备 | 必须（从 `main` 切出） |

### 3.2 `main` 与 `native` 的分工

这是本仓库最重要的分支约定，完整规则见 [AGENTS.md](AGENTS.md) 分支与封装契约一节。

**`main`**：RT-Control 必须提供可复现的镜像构建、Compose 服务定义和容器内启动入口。禁止只能在宿主原生环境跑通的实现进入 `main`。合并到 `main` 的 PR 必须提供容器构建与容器内启动证据。

**`native`**：为敏捷开发保留，允许 `--symlink-install` 增量构建、宿主直跑和原生一键脚本，暂不要求容器封装。`native` 上的实时性、安全链、接口契约和质量门禁要求**与 `main` 完全相同**，只豁免容器封装。

`native` → `main` 的提升必须补齐容器封装，不得把原生专用路径、宿主绝对路径或未封装启动方式带进 `main`。

### 3.3 分支保护

`main` 与 `native` 都开启：

- 禁止直接 push，全部通过 PR 合并；
- 必须 CI 通过；
- 必须至少一名 reviewer approve；
- 禁止 force push；
- 禁止改写共享历史。

`main` 额外要求：涉及共享 Robot Model、`src/interfaces`、Compose、部署配置或安全边界的 PR 需要核心负责人 approve。

### 3.4 分支命名

```text
feature/rt-control-native-development
feature/rt-control-contract-v0-5
bugfix/joint-states-stale-timestamp
hotfix/demo-ethercat-enable-timeout
release/v0.3
```

命名使用小写、连字符分词，包含域名前缀。禁止 `feature/test`、`feature/tmp`、`feature/wip` 这类无信息名称。

## 4. Commit 规范

### 4.1 格式

```text
<type>(<scope>): <description> [<需求号>]
```

`type` 与 `scope` 必填；需求号在有对应需求/任务号时必填。

```text
feat(rt-control): publish full 14-axis joint states [REQ-RT-003]
fix(motion): reject partial-joint FJT goals [REQ-MO-011]
docs(interfaces): freeze cross-domain interface baseline
test(tools): cover compose transport policy in repository gate
refactor(rt-control): split enable manager from bringup
```

### 4.2 type

| type | 含义 |
| --- | --- |
| `feat` | 新功能 |
| `fix` | 缺陷修复 |
| `docs` | 文档 |
| `style` | 格式，不改逻辑 |
| `refactor` | 重构，不新增功能、不修缺陷 |
| `test` | 测试代码 |
| `chore` | 构建、配置、依赖等杂项 |
| `perf` | 性能优化 |
| `ci` | CI 配置 |
| `revert` | 回滚 |

### 4.3 scope

按**域或共享资产**取值，不按文件类型：

| scope | 对应内容 |
| --- | --- |
| `rt-control` | `src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**` |
| `interfaces` | `src/interfaces/**` |
| `description` | `src/description/**`（共享 Robot Model） |
| `deploy` | `docker/compose.yaml`、`deploy/**`、systemd、release manifest |
| `tools` | `tools/**` |
| `ci` | `.github/**` |
| `docs` | 根 README、`domains/rt_control/docs/**`、RT-Control PROGRESS |

跨多个 scope 时优先拆成多个提交。确实不可拆分时使用主责 scope，并在正文列出其余受影响范围。

### 4.4 禁止的提交信息

```text
update / fix bug / 修改了一下 / 临时提交
final version / demo 能跑的版本 / wip / asdf
```

### 4.5 提交粒度

- 一个提交只做一件可描述、可回滚的事。
- 禁止把多个无关任务塞进一个提交。
- 禁止在功能提交里夹带无关格式化、生成物或调试代码。
- 用户已有的工作树改动必须保留；未经明确授权不得用 `git checkout --`、`git reset --hard`、清理工作树或覆盖文件处理它们。

## 5. 提交前门禁

### 5.1 必跑命令

```bash
git status --short --branch
git diff --check
git diff --stat
git diff
git diff --cached --check
git diff --cached
tools/quality_gate.sh
```

再加所属域 `AGENTS.md` 要求的包构建、单测、契约、mock/仿真、容器或 HIL/实机门禁。

### 5.2 pre-commit

本仓库通过 pre-commit 调用同一个门禁脚本，CI 跑的是同一份检查：

```bash
pip install pre-commit
pre-commit install
```

未安装 hook 不构成豁免。禁止使用 `--no-verify` 绕过。

### 5.3 安全检查

提交前逐项确认：

- [ ] 无 token、私钥、证书私钥、密码、代理凭据或客户敏感数据；
- [ ] 无硬件 ID、对象字典、标定值、CPU 拓扑、现场路径的猜测值；
- [ ] 无构建产物、日志、调试旁路、未解释的大文件；
- [ ] 未扩大容器 device、capability、网络、宿主写权限或 Docker socket；
- [ ] 未用软件诊断代替急停、安全继电器、STO、驱动器保护和机械限位；
- [ ] 未把编译、mock、仿真或离线校验写成“实机验证通过”。

缺少权威事实时**只阻塞受影响范围**，记录到
`domains/rt_control/BLOCKED-questions.md`，不要猜。

## 6. Pull Request 规范

### 6.1 标题

```text
[<type>][<scope>] 简要说明
```

```text
[feat][motion] add Nav2 base motion gate service
[fix][rt-control] stop base locally when cmd_vel_safe expires
[docs][interfaces] freeze cross-domain interface baseline
```

### 6.2 描述模板

仓库已提供 `.github/pull_request_template.md`，其中 `gate:*` 注释由 `tools/pr_contract_gate.py` 在 CI 中校验。**必须保留全部 `gate:*` 注释并勾选全部必选项**，否则 CI 拒绝该 PR。

模板要求填写四块内容：

1. **变更范围**：任务/需求号、所属域或共享资产、目标包、直接与跨域消费者、明确未修改的内容。
2. **架构与安全审查**：scope、architecture、interfaces、realtime-safety 四条勾选。
3. **验证证据**：逐条写“实际执行的命令：PASS/FAIL”。不接受只写“测试通过”。
4. **未验证项与剩余风险**：没有未验证项也要明确写“无”并说明判断范围；实机验证未执行必须写明。

### 6.3 合并要求

| PR 类型 | 要求 |
| --- | --- |
| 普通域内变更 | ≥1 reviewer approve；CI 通过；无 unresolved conversation |
| 共享 Robot Model（`src/description/**`） | ≥2 approve，其中 1 名核心负责人；全部消费域联合验证 |
| 公共接口 vendor/source-lock | 接口所有者 + 全部消费域 approve；同批更新上游契约及固定 SHA |
| 部署与 Compose（`docker/**`、`deploy/**`） | 平台/集成 + 全部受影响域 approve |
| 实时性或安全边界（250 Hz 环、CPU 隔离、capability、device） | 核心负责人 approve；附时序或隔离实测证据 |
| 合并到 `main` | 上述之外，必须附容器构建与容器内启动证据 |

### 6.4 评审分工路由

| 变更路径 | 必读附加契约 | 主要联合评审 |
| --- | --- | --- |
| `src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**` | `domains/rt_control/AGENTS.md` | rt-control；公共模型/接口变更时通知外部消费者 |
| `src/description/robot_description/**` | 根 `AGENTS.md` 共享资产一节 | Robot Model + rt-control；破坏性变更通知外部消费域 |
| `deps.repos` 中的 `robot_interfaces` pin、`src/interfaces/source-lock.yaml` | 独立 `robot_interfaces` 权威契约 | 接口所有者 + 全部跨域生产者/消费者 |
| `src/interfaces/rt_control_interfaces/**` | `domains/rt_control/AGENTS.md` | RT-Control 域内生产者/消费者 |
| `docker/compose.yaml`、`deploy/**`、DDS/发布配置 | `domains/rt_control/AGENTS.md` | RT-Control + 平台/集成 |

`tools/**`、`domains/rt_control/docs/**`、`.github/**` 按其实际管理的对象归属，
不因位于根目录就自动视为公共资产。根 `docs/` 必须保持为空。

### 6.5 评审严重级别

| 级别 | 含义 | 处理 |
| --- | --- | --- |
| CRITICAL | 安全隐患、数据丢失风险、绕过硬安全链、泄露密钥 | **阻塞**，合并前必须修 |
| HIGH | 缺陷或显著质量问题、破坏接口契约 | 合并前应修 |
| MEDIUM | 可维护性问题 | 考虑修复 |
| LOW | 风格或次要建议 | 可选 |

结论口径：无 CRITICAL 且无 HIGH → Approve；仅有 HIGH → 谨慎合并并记录；存在 CRITICAL → Block。

## 7. Issue 管理

任务不得只在群里或口头安排。Issue 是需求、缺陷、任务和讨论的唯一入口。

### 7.1 类型与标题

类型标签：`feature` / `bug` / `task` / `design` / `docs` / `test` / `deploy` / `question`。

标题格式 `[<域或共享资产>] 简要说明`：

```text
[motion] Nav2 导航任务 Action 增加到位误差判定
[rt-control] cmd_vel_safe 过期后本地停车时限收紧到 200 ms
[interfaces] 明确 DomainReadiness 未就绪原因语义
[deploy] Compose 增加 perception 域服务定义
```

### 7.2 内容模板

```markdown
## 背景
为什么需要这个任务。

## 目标
完成后应达到什么效果。

## 输入
依赖哪些数据、接口、模块或权威事实源。

## 输出
应产出什么（代码、接口、文档、证据）。

## 验收标准
- [ ] 标准 1
- [ ] 标准 2

## 涉及实现与共享资产
- rt-control
- description / interfaces / deploy
- 公共接口变更时列出外部生产者和消费者

## 需要的裁决或阻塞项
若依赖未裁决的硬件、标定、安全或资源事实，在此列出。
```

### 7.3 Issue 与 PR 关联

一个 PR 对应一个或多个明确 Issue，PR 描述中写 `Closes #xx` 或 `Related to #xx`。
跨域任务先在 `robot_interfaces` 开 `design` Issue 承载裁决，再由各仓库分别实现；其他域代码不得进入本 PR。

## 8. 版本与发布

### 8.1 版本号

采用语义化版本 `vMAJOR.MINOR.PATCH`：

- `MAJOR`：破坏性接口变更、共享 Robot Model 破坏性变更、域责任边界调整；
- `MINOR`：新增域、新增非破坏性接口、新增能力；
- `PATCH`：缺陷修复与文档补充。

Demo 或阶段交付附后缀：`v0.3.0-demo`、`v0.3.0-rc1`。

### 8.2 发布流程

1. 从 `main` 切 `release/vx.y`，冻结功能；
2. 更新 `CHANGELOG.md`（Added / Fixed / Changed / Known Issues）；
3. 全域镜像构建成功，整机 Compose 可拉起；
4. 执行整机验收清单与 vendored `contract/views/rt_control.md` 边界验收；
5. 记录 release manifest：RT-Control 镜像 tag、Robot Model 版本/哈希、interfaces schema 版本、标定版本、冻结上游 commit SHA；
6. 打 tag 并归档验收证据。

Tag 是版本锚点，打出后不得移动或删除。

### 8.3 发布必备证据

- RT-Control 镜像构建日志与 image digest；
- 整机 Compose 启动与健康检查记录；
- 250 Hz 实时循环时序实测；
- 接口边界验收清单勾选结果；
- 明确列出未验证项与剩余风险。

## 9. 代码风格与测试

### 9.1 通用

- 命名清晰，禁止 `a`、`tmp2`、`test3` 这类无信息命名；
- 函数聚焦，单函数 < 50 行；文件 < 800 行，典型 200～400 行；
- 嵌套不超过 4 层，优先早返回；
- 阈值、延时、限值使用命名常量，不用魔数；
- 显式处理错误，不静默吞掉；
- 系统边界必须校验输入，不信任外部数据；
- 优先不可变数据，避免隐式副作用。

### 9.2 Python

PEP 8；函数签名带类型注解；`black` + `isort` + `ruff`；测试用 `pytest`。

### 9.3 C++

现代 C++ 惯用法；遵循 C++ Core Guidelines；`clang-format`；实时路径中禁止动态分配、加锁阻塞和日志阻塞。

### 9.4 ROS 2 包

- `package.xml` 依赖完整且最小；
- 共享包不得依赖业务域实现；
- 域实现不得跨域引用对方内部库、私有配置或可写数据目录；
- launch 参数外置到版本化 YAML，不硬编码现场值。

### 9.5 测试要求

| 层次 | 要求 |
| --- | --- |
| 单元测试 | 每个包覆盖核心逻辑分支 |
| 集成测试 | 接口契约、启动流程、跨包交互 |
| 契约测试 | 域间接口的名称、类型、方向、QoS、时效、终态语义 |
| 仿真/mock | 在无实机条件下验证成功、取消、超时、失败、重启和资源收尾 |
| 实机/HIL | 实时性、总线、使能/去使能、安全链；证据必须可追溯 |

仓库工具链覆盖率门禁：门禁集（`tools/repository_gate.py`、`tools/pr_contract_gate.py`）合并覆盖率 ≥80%，由 `quality_gate.sh` 强制；其余 `tools/**` Python 脚本的覆盖率在门禁输出中打印可见但暂不设阈值（现状：`diff_legacy` 63%、`rt_control_axis_state_check` 69%、`rt_control_thread_affinity` 0%，2026-08-13 实测），提升覆盖与纳入门禁属独立任务。测试收集使用 pytest（同时收集 unittest 类与模块级测试函数，防止静默漏收）。测试必须覆盖成功、取消、超时、失败、重启和资源收尾路径，不只测 happy path。

**测试通过与实机验证是两件事**，不得混写。

## 10. CI

现有 workflow 分两阶段；新增 RT-Control 包时在保留公共门禁的前提下增加对应 job，
**不得用新功能需求削弱已有安全检查**：

| 阶段 | 内容 |
| --- | --- |
| `governance` | pre-commit 全量运行（同一份 `tools/quality_gate.sh`）；PR 契约门禁 |
| `build` | 依赖安装、`colcon build`、`colcon test`、共享 Robot Model URDF 校验、冻结上游迁移门禁 |

`main` 分支 CI 额外要求 RT-Control 镜像构建与容器内启动 smoke test。CI 未通过不得合并。

## 11. 权限与 Team

| Team | 权限范围 |
| --- | --- |
| `core-maintainers` | 仓库 maintain/admin；共享 Robot Model、interfaces、部署与安全边界的最终裁决 |
| `rt-control-team` | `src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**` |
| `platform-integration` | `docker/**`、`deploy/**`、`tools/**`、`.github/**` |
| `interns` | write 权限，不得 approve 合并 |

使用 CODEOWNERS 按路径分派评审。所有成员对 `main` 与 `native` 均无直接 push 权限。

## 12. 必须暂停并请求裁决的情形

- 新需求与根契约、域契约、冻结接口或已裁决问题冲突；
- 需要猜测硬件、标定、安全或资源分配事实；
- 破坏性修改共享 Robot Model 或跨域接口，但消费者、迁移和发布顺序不明；
- 需要扩大容器、设备、外网、Docker socket 或宿主权限；
- 需要运行 root 脚本、修改宿主机、启停总线、写设备、使能或驱动实机，而没有针对该动作的单独授权；
- 用户已有改动与当前任务重叠且无法安全合并；
- 适用的构建/测试失败，或验证环境不足以支持当前完成声明。

## 13. 禁止事项

- 直接 push `main` 或 `native`；
- `--no-verify`、force push、擅自 amend 他人提交、改写共享历史；
- 把多个无关任务塞进一个提交或一个 PR；
- 提交密钥、证书私钥、凭据或客户敏感数据；
- 提交构建产物、日志、`__pycache__`、未解释的大文件；
- 复制共享 Robot Model 后私有修改；
- 用共享可写目录、对方内部库或新私有 RPC 绕过冻结接口；
- 未经联合评审删除或重命名接口字段；
- 用软件诊断代替硬安全链；
- 把 mock、仿真或编译结果写成实机验证结论；
- 只在群里安排任务而不开 Issue。

## 14. 最终原则

**接口先冻结，责任边界不越界，证据可追溯，未验证就写未验证。**

`main` 交付容器，`native` 交付速度；两者的安全与契约底线完全一致。
