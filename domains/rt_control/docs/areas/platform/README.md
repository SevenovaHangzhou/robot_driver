# platform — 宿主、构建、CI、验证与仓库治理

**范围**：宿主、构建、CI、验证与仓库治理。本区对应一个同名职责agent。

**Owner路径**：hostsetup/**；docker/**；tools/**；.github/**；deps.repos；versions.env；domains/rt_control/testing/**；各级AGENTS.md；协作规范和记录体系。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：功能驱动、控制算法和机型事实按其owner归区；检查工具归platform不改变被检查资产的owner。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| contract/03#F3 | 全部生产/消费域必须锁定同一接口 SHA 原子升级 | [contract-20260816-01](../contract/records/2026-08-16-bq137-shared-error-readiness.md)#F3 | 有效 |
| contract/04#F3 | 最终 SHA 回填不解除各域同 SHA 原子升级与跨域 smoke 门禁 | [contract-20260817-01](../contract/records/2026-08-17-robot-interfaces-final-main-pin.md)#F3 | 有效 |
| contract/05#F1 | Native 完整构建必须安装并验证 QoS 五 profile 运行闭包 | [contract-20260817-02](../contract/records/2026-08-17-native-qos-runtime-closure.md)#F1 | 有效 |
| contract/05#F2 | Native 入口在硬件访问前 fail-closed 检查公共适配器依赖闭包 | [contract-20260817-02](../contract/records/2026-08-17-native-qos-runtime-closure.md)#F2 | 有效 |
| contract/05#F3 | Native READY 需要 live controller-manager、控制器状态与 EtherCAT OP | [contract-20260817-02](../contract/records/2026-08-17-native-qos-runtime-closure.md)#F3 | 有效 |
| contract/07#F1 | ROS Domain 由部署显式配置为十进制 `0..232`，默认 `0` | [contract-20260828-01](../contract/records/2026-08-28-configurable-ros-domain.md)#F1 | 有效；取代 BQ-128 固定 Domain 0 |
| contract/07#F2 | 同一机器实例的五域必须使用同一 Domain，Domain 不是安全隔离 | [contract-20260828-01](../contract/records/2026-08-28-configurable-ros-domain.md)#F2 | 有效 |
| contract/07#F3 | RMW 仍固定 `rmw_fastrtps_cpp`，传输仍为 Fast DDS 默认 UDP+SHM | [contract-20260828-01](../contract/records/2026-08-28-configurable-ros-domain.md)#F3 | 有效 |
| ecat-axes/TI5-ASSIGN-1 | Native 启动检查实际安装 profile、驱动分配和 IgH 缓存，仅将已知默认分配恢复为已验证的 1601/1A01；未知映射拒绝启动，正确暖态无写入/重扫。 | [ecat-axes-20260911-01](../ecat-axes/records/2026-09-11-ti5-assignment-recovery.md)#F1-F3 | PARTIAL（T3；掉电重复性未验） |
| governance/F2 | 记录格式为 TEMPLATE.md：frontmatter + 五段正文，合并后不可改，推翻用 supersedes | [governance-20260813-01](../governance/records/2026-08-13-establish-area-record-system.md)#F2 | 有效 |
| governance/F3 | PROGRESS.md 自 2026-08-13 起降级为时间线索引，历史段冻结 | [governance-20260813-01](../governance/records/2026-08-13-establish-area-record-system.md)#F3 | 有效 |
| governance/F4 | 冻结 REQ 表原件确认丢失，不重建；需求追溯坐标改用 `<记录id>#F<n>` | [governance-20260813-01](../governance/records/2026-08-13-establish-area-record-system.md)#F4 | 有效 |
| governance/F5 | 测试风险分层 T0–T4，发布门禁自动部分只到 T2 | [governance-20260813-02](../governance/records/2026-08-13-test-case-catalog-v1.md)#F1 | 有效 |
| governance/F6 | 用例 ID 规则 `TC-<分类码>-<NN>`，不复用 | [governance-20260813-02](../governance/records/2026-08-13-test-case-catalog-v1.md)#F2 | 有效 |
| governance/F7 | V0.10 最小门禁 = v010_gate run 全过 + reference 全部给出历史坐标 | [governance-20260813-02](../governance/records/2026-08-13-test-case-catalog-v1.md)#F3 | 有效 |
| governance/F8 | 根 docs 必须为空，协作规范位于仓库根，接口视图来自 vendor | [governance-20260814-01](../governance/records/2026-08-14-root-docs-and-vendor-governance.md)#F1 | 有效；取代 governance-20260813-04#F1 |
| governance/F9 | source-lock 发布身份必须与 deps.repos 的 robot_interfaces pin 一致 | [governance-20260814-01](../governance/records/2026-08-14-root-docs-and-vendor-governance.md)#F2 | 有效 |
| governance/F10 | 已归档 records 保留旧路径作为历史坐标 | [governance-20260814-01](../governance/records/2026-08-14-root-docs-and-vendor-governance.md)#F3 | 有效 |
| governance/F11 | main 是稳定源码基线，普通源码 PR 不强制 Docker；源码稳定后由人工选择 SHA 封装，发布产物仍需完整镜像/容器证据 | [governance-20260915-01](../governance/records/2026-09-15-main-manual-docker-packaging.md)#F1-F3；BQ-148 | 有效 |
| governance/F12 | 完整 CI 只由代码相关 PR 事件触发；PR 描述编辑只跑 contract governance；PR 合并后的 main push 不重复全量构建 | [governance-20260917-01](../governance/records/2026-09-17-pr-ci-trigger-deduplication.md)#F1-F3 | 有效；GitHub main 分支保护待配置 |
| governance/F13 | Robot Model CI 校验必须 source 当前构建 overlay 后再运行 xacro/check_urdf | [governance-20260919-01](../governance/records/2026-09-19-v3-description-overlay-ci.md)#F1-F2 | PASS（T0 回归；PR CI 终态待确认） |
| governance/F14 | head-fast 只提供提前反馈，所有代码 PR 仍无条件执行完整 V3 build；CI 基础镜像使用精确标签且保留公开 ROS 回退 | [governance-20260920-01](../governance/records/2026-09-20-layered-ci-cache.md)#F1-F4 | PARTIAL（基础镜像发布与变量启用待完成） |
| governance/STYLE-1 | 固定clang-format14/ruff0.11.13；仅改动范围门禁，自有C++五项警告 | [governance-20261013-01](../governance/records/2026-10-13-executable-code-style.md)#F1-F3 | UNVERIFIED（T0工具验证通过；构建待验） |
| governance/CONTRACT-1 | 三档测试、语言分工、任务指派分支授权 | [governance-20261013-02](../governance/records/2026-10-13-agent-contract-sync.md)#F1-F3 | UNVERIFIED（T0文档） |

| AREA-1 | 七区owner、历史冻结与目标分层，代码迁移另立任务 | [划分decision](../governance/records/2026-10-13-seven-agent-areas.md)#F1-F4 | UNVERIFIED（T0文档与索引） |
