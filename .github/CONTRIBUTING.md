# robot_driver 协作流程

按需查阅。安全底线和授权见根 [`AGENTS.md`](../AGENTS.md)，架构与编码见 [`ARCHITECTURE.md`](../ARCHITECTURE.md)。

## 1. 分支

- `main` 是唯一长期分支，也是 V3 主线。禁止直接 push，全部经 PR 合并；必须 CI 门禁（governance）通过、至少一名 reviewer approve；禁止 force push 和改写共享历史。
- 工作分支：`feature/<id>-<slug>`、`bugfix/<id>-<slug>`、`hotfix/<id>-<slug>`、`release/vX.Y`。`<id>` 用 Linear ID；无 Issue 的治理任务用 `arch-` 等任务前缀。小写、连字符分词，禁止 `tmp`、`wip`、`test` 这类无信息名称。
- 涉及 Robot Model、`src/interfaces`、Compose、部署配置或安全边界的 PR 需要核心负责人 approve。

## 2. Commit

```text
<type>(<scope>): <description> [<任务号>]
```

- `type`：`feat` `fix` `docs` `style` `refactor` `test` `chore` `perf` `ci` `revert`。
- `scope` 按资产取值：`rt-control`（`src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**`）、`interfaces`、`description`、`deploy`、`tools`、`ci`、`docs`、`governance`。跨 scope 优先拆提交。
- 一个提交只做一件可描述、可回滚的事；不夹带无关格式化、生成物或调试代码。
- 禁止 `update`、`fix bug`、`wip`、`临时提交` 之类信息。

## 3. Pull Request

- 描述按 `.github/pull_request_template.md` 填写；`gate:*` 注释由 `tools/pr_contract_gate.py` 在 CI 校验，必须保留并勾选。
- PR 顶部可加不超过 10 行的摘要（结论 / 已完成 / 验证 / 风险 / 需要 / 细节），不替代模板章节。
- 一个 PR 只做一个目的；结构变更与行为变更分开提。

### 3.1 合并要求

| PR 类型 | 要求 |
| --- | --- |
| 普通域内变更 | ≥1 approve；CI 通过；无未解决讨论 |
| Robot Model 构建副本 | ≥2 approve，含 1 名核心负责人；全部消费域联合验证 |
| 公共接口 pin / source-lock | 接口所有者 + 全部消费域 approve；与上游契约同批更新 |
| `docker/**`、Compose、部署 | 平台/集成 + 受影响域 approve |
| 实时或安全边界（控制频率、CPU 隔离、capability、device） | 核心负责人 approve；附时序或隔离实测 |
| 人工封装或正式发布 | 记录 source SHA，附镜像构建、容器内启动和镜像身份 |

### 3.2 接口评审清单

修改 `rt_control_interfaces` 或公共接口 pin 时，PR 必须说明：单位/坐标系/时间戳/标识符；请求合法性与拒绝条件；
终态、错误码与反馈；超时、取消、抢占、重复请求和重启语义；新旧版本兼容性与是否需要多容器原子升级；
高频状态是否误用 service/action、长任务是否误用 topic/service。并列出全部生产者和消费者。

### 3.3 评审严重级别

| 级别 | 含义 | 处理 |
| --- | --- | --- |
| CRITICAL | 安全隐患、绕过硬安全链、泄露密钥 | 阻塞 |
| HIGH | 缺陷、破坏接口契约 | 合并前应修 |
| MEDIUM | 可维护性问题 | 考虑修复 |
| LOW | 风格或次要建议 | 可选 |

## 4. Issue

任务不得只在群里或口头安排。新 Issue 使用 [`ISSUE_TEMPLATE/task.md`](ISSUE_TEMPLATE/task.md)。
PR 描述中写 `Closes`/`Related to` 关联 Issue；跨域任务先在 `robot_interfaces` 开 design Issue 承载裁决。

## 5. CI

| 事件 | 执行 |
| --- | --- |
| PR opened / synchronize / reopened / labeled | `rt-control-ci`：`governance`（pre-commit = `tools/quality_gate.sh` + PR 契约门禁），合并只等它 |
| PR 带 `full-ci` 标签 | 另跑完整 `build`（干净环境：上游依赖、补丁、IgH、全量构建与测试） |
| 每晚 02:00（北京时间）、手动触发 | 在 `main` 上跑完整 `build`；失败时由提交者第二天修复 |
| PR edited | `pr-metadata-governance`：只校验 PR 契约 |
| push(main) | 不运行 |

用户 2026-10-30 选定方案 A：PR 只等快速门禁，完整构建改为每晚。改动构建配置、包清单
（`package.xml`/`CMakeLists.txt`/`deps.repos`/`patches/`/`docker/`/`.github/`/`tools/bootstrap_native_dev.sh`）
的 PR 应打 `full-ci` 标签或由作者本地完成对应构建。本地通过 `pre-commit install` 调用同一门禁；
未安装 hook 不构成豁免。普通源码 CI 不构建生产镜像。

## 6. 权限

| Team | 范围 |
| --- | --- |
| `core-maintainers` | 仓库 maintain/admin；Robot Model、interfaces、部署与安全边界的最终裁决 |
| `rt-control-team` | `src/rt_control/**`、`docker/rt-control/**`、`hostsetup/**` |
| `platform-integration` | `docker/**`、`deploy/**`、`tools/**`、`.github/**` |
| `interns` | write 权限，不得 approve 合并 |

路径级强制评审见 [`CODEOWNERS`](CODEOWNERS)。
