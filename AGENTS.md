# robot_driver Agent 契约

本仓库是半人形拆码垛机器人的 **RT-Control 实时控制域**。开工前只需读两份文件：本文件和
[`ARCHITECTURE.md`](ARCHITECTURE.md)。架构、分层、编码与测试以 `ARCHITECTURE.md` 为准；
本文件管安全底线、授权和交付。两者冲突时，架构问题以 `ARCHITECTURE.md` 为准，安全与授权问题以本文件为准。

按需查阅：流程（分支、提交、PR、Issue、评审）见 [`.github/CONTRIBUTING.md`](.github/CONTRIBUTING.md)；
各类改动的验证命令见 [`tools/README.md`](tools/README.md)；容器、宿主与发布见
[`docker/README.md`](docker/README.md)、[`hostsetup/README.md`](hostsetup/README.md)。

## 1. 域边界

- RT-Control 只负责硬件适配、轨迹/速度/IO 执行、生命周期、故障处理和状态反馈。任务编排、感知、
  Nav2、MoveIt2、Gateway 属于外部域，本仓库不得实现或直接引用其内部库。
- 外部业务命令必须按公共契约经责任域到达 RT-Control，不得新增外部域直控硬件的旁路。实时控制环不得跨容器。
- 公共接口的权威源是独立 `robot_interfaces` 仓库，本仓库以 `deps.repos` 固定完整 SHA 导入，
  `src/interfaces/source-lock.yaml` 必须一致；禁止在本仓库复制或修改公共 schema。新跨域命令先改公共契约
  （调用方向、超时、取消、幂等、错误语义），再更新 pin。`rt_control_interfaces` 只供本域使用。
- Robot Model 的权威源是独立 `robot_description` 仓库；`src/description/robot_description` 是构建副本，
  不得形成独立模型分支。link/joint 名称、TF、轴向、零位、限位、mesh 尺寸变更是破坏性公共变更。
- 删除、重命名或改变公共字段类型、单位、坐标系、终态与错误语义均为破坏性变更；没有消费者迁移和原子发布方案不得合并。

## 2. 事实与裁决

- 不得猜测硬件 ID、对象字典、单位/比例/方向、CPU 拓扑、标定、运动/安全阈值或现场路径。
  缺少权威事实时写 `TBD`，只阻塞受影响范围。
- `domains/rt_control/BLOCKED-questions.md` 记录阻塞问题和用户裁决，**写入前必须经用户确认**（格式见该文件）。
  其中已裁决项是本域权威约束；新需求与既有裁决冲突时停止相关实现并请求裁决，不得静默覆盖。
  2026-10-29 前的旧条目已清除，追溯旧裁决查 git 历史。

## 3. 实时与安全底线

- 1 kHz 及其他实时路径禁止无界循环、阻塞 IO、文件/网络访问、动态参数查询、日志刷屏，以及每周期不受控的
  内存分配、锁竞争或容器增长。非实时回调与实时更新之间用预分配、有界的通信方式。
- 以下裁决不得被变相恢复：
  - 不新增独立 `rt_watchdog` 包；
  - 不恢复 motion/autonomy 心跳、原始 CAN 旁路监听器或另一套独立总线超时判定；
  - 不做 CANopen 节点的自动局部 NMT 恢复，故障恢复走完整 rt-control 重启；
  - 不为 CPU 绑核、总线设备、对象字典或安全阈值提供猜测性默认值。
- 软件诊断不得代替急停、安全继电器、STO、驱动器保护和机械限位。
- 执行器相关变更必须检查 configure/activate/deactivate/cleanup/error、信号退出、通信中断和部分设备未就绪，
  最终状态必须可解释且保守。

## 4. 上游与补丁

- 上游源码只通过 `deps.repos` 的完整 commit SHA 锁定，禁止把上游仓库复制进本仓库。
- `patches/` 下每个补丁只解决一个问题，保持最小差异，能对冻结提交 `git apply --check`。
  不得借补丁重写 CiA 402、controller_manager 或 ros2_control 的通用行为。
- 更新上游 SHA 时重新验证全部相关补丁、构建和行为。不得直接编辑构建目录里的上游或生成文件。

## 5. 硬件与宿主授权

代码任务不构成硬件授权。以下动作每一项都需要用户针对该动作的单独明确授权：
上电、使能、运动、写 SDO、启停总线、启动生产 Compose、运行 `hostsetup` 或任何 root 脚本、修改宿主机。
没有授权时只做静态检查。生产容器必须以安装后的 `rt_control_start` 作为 PID 1；Compose 一律通过
`tools/rt_control_compose.sh` 调用，`RT_CONTROL_CPUSET` 与 `RT_CONTROL_START_CPUSET` 必须来自目标机验证，
不设仓库默认值。不得扩大 `privileged`、device、capability、网络暴露、宿主写权限或 Docker socket。

## 6. 验证分级

验证按级升级，前一级失败不得进入后一级；每级结论只能写成该级：

| 级 | 内容 |
| --- | --- |
| 1 | 静态检查与配置/schema 校验 |
| 2 | 受影响包构建与自动化测试 |
| 3 | `use_mock_hardware:=true` 的加载、生命周期和停机 |
| 4 | 容器 config/镜像构建/无硬件启动（仅 Docker/Compose 变更或人工封装/发布时必做） |
| 5 | 断动力或不使能的现场通信 |
| 6 | 有急停、隔离区、限速、监护人和回退方案的上电低速验证 |
| 7 | 完整生产验收 |

编译、mock、仿真和离线校验不得写成"实机验证通过"。

## 7. 提交与交付

- 每次提交前：`git status --short --branch`、`git diff --check`、逐行审查 `git diff` 与 `git diff --cached`
  （含 `??` 未跟踪文件），再跑 `tools/quality_gate.sh` 和 [`tools/README.md`](tools/README.md) 中对应改动类型的命令。
- 提交、推送、开 PR 只在用户明确要求时执行。在任务系统里把 Issue 派给某个 agent，即视为授权它在自己的分支上
  提交、推送并开 PR；合并由人完成。此授权不含打 tag、上传发布物或硬件操作。
- KKOZIA-2 子 issue 的 PR 由架构师在 CI 通过后合并（2026-10-13 用户授权），`safety_policy.yaml` 除外。
- 禁止：直接推 `main`、`--no-verify`、force push、改写共享历史、擅自 amend 他人提交、把无关任务塞进一个提交；
  提交密钥、凭据、构建产物、日志、`__pycache__`、未解释的大文件；用 `git checkout --`、`git reset --hard`
  或清理工作树处理用户已有改动。
- 行为变更或硬件事实变更合并时，在 `domains/rt_control/docs/areas/<area>/records/` 新增一条记录
  （格式见该目录 `TEMPLATE.md`），实机验证还要在 `domains/rt_control/PROGRESS.md` 追加一行。纯结构/文档变更不记。
- 推送后报告远端分支、提交 SHA、实际验证和未验证项。PR 描述按 `.github/pull_request_template.md` 填写。

## 8. 必须暂停并请求裁决

- 新需求与本文件、`ARCHITECTURE.md`、冻结接口或已裁决问题冲突；
- 需要猜测硬件、标定、安全或资源分配事实；
- 破坏性修改 Robot Model 或跨域接口，但消费者、迁移和发布顺序不明；
- 需要扩大容器、设备、网络、Docker socket 或宿主权限；
- 需要第 5 节中任何未获单独授权的动作；
- 用户已有改动与任务重叠且无法安全合并；
- 适用的构建/测试失败，或验证环境不足以支持当前完成声明。

**接口先冻结，责任边界不越界，证据可追溯，未验证就写未验证。**
