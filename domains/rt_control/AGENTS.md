# rt-control 域 AI 协作与提交契约

## 1. 适用范围与目标

本文件适用于根 `AGENTS.md` 路由表中归属 rt-control 的所有资产，包括域包、容器、宿主配置、上游补丁、专属工具和验收文档。本契约只能细化根契约，不得放宽公共架构、安全和验证要求。

本域是 ROS 2 Humble 机器人实时控制域，当前包含平台共享模型/接口的首个落地版、rt-control 域实现、上游窄补丁、容器部署和目标工控机配置。任何 AI 或人工变更都必须满足：

1. 不突破域边界和既有架构裁决；
2. 不猜测硬件事实、安全参数或实时参数；
3. 变更范围最小且可追溯；
4. 提交前完成与风险匹配的自审和验证；
5. 如实报告已验证、未验证和剩余风险。

## 2. 开始任务前的必读与检查

开始编辑前，按顺序完成：

1. 阅读根 `README.md` 和 `AGENTS.md`，确认 RT-Control 仓库边界与域间公共契约；
2. 阅读 `domains/rt_control/README.md` 和本契约，确认构建、容器启动和 rt-control 布局约束；
3. 阅读 `domains/rt_control/PROGRESS.md` 中目标任务及相关历史验证记录，并阅读
   `domains/rt_control/docs/areas/` 中目标功能区 README 的冻结事实表与近期记录；
4. 在 `domains/rt_control/BLOCKED-questions.md` 搜索相关任务号、需求号、设备、接口或配置；
5. 阅读目标包的 `package.xml`、`CMakeLists.txt`、源代码、配置和直接消费者；
6. 执行 `git status --short --branch`，区分用户已有改动与本任务改动；
7. 写明本次变更涉及的任务号/需求号、包、接口、运行阶段和风险等级。

`domains/rt_control/BLOCKED-questions.md` 中的已裁决项属于本域权威约束。新需求与既有裁决冲突时必须停止相关实现，记录冲突并请求裁决，不得静默覆盖。缺少电机、驱动器、总线、CPU、标定或安全链事实时，使用 `TBD` 并只阻塞受影响范围，禁止填入“合理默认值”。

## 3. 仓库架构契约

### 3.1 目录职责

- `src/description/robot_description`：平台级机器人权威模型源，只保存机器人固有的运动学、几何、惯量、碰撞模型、坐标系和 mesh。
- `src/vendor/robot_interfaces`：由 `deps.repos` 固定 SHA 导入的公共接口与 QoS，
  `src/interfaces/source-lock.yaml` 必须记录同一身份；禁止在本仓库保存公共 schema 镜像。
- `src/interfaces/rt_control_interfaces`：RT-Control 域内接口，只保存
  `PlcIoState`、`RtEnable` 等本域 `msg/srv/action`，其他域不得依赖。
- `src/rt_control/robot_hw_ethercat`：EtherCAT 拓扑、从站配置和 ros2_control 硬件描述。
- `src/rt_control/robot_hw_canopen`：两条履带的 CANopen 总线配置。
- `src/rt_control/enable_manager`：14 轴使能、失能、故障复位状态机。
- `src/rt_control/rt_diagnostics`：实时控制域诊断归一化和状态上报。
- `src/rt_control/rt_control_bringup`：rt-control 总装、控制器配置、生命周期编排和唯一受支持的启动入口。
- `patches`：对 `deps.repos` 中冻结上游版本的最小补丁，不是上游源码副本。
- `docker`：镜像、Compose、DDS 和容器退出语义。
- `hostsetup`：目标工控机的高权限安装与验证脚本。
- `tools`：只读检查、迁移差异、配置归档和受控部署包装器。
- `domains/rt_control/docs`：rt-control 设计依据、调试记录、部署说明和实机验收证据。

不得在 rt-control 归属的路径中加入 perception、motion、autonomy 或 gateway 的业务实现。rt-control 只负责硬件适配、轨迹/速度/IO 执行、生命周期、故障处理和状态反馈；不得承担任务规划、行为树、导航决策、感知决策、抓取策略或数据库职责。

### 3.2 依赖方向

允许的主要依赖方向是：

```text
robot_description ───────────────┐
robot_*_interfaces + QoS（vendor）┼─> rt_control 功能包 ─> rt_control_bringup
rt_control_interfaces（域内）────┘
```

- 功能包不得反向依赖 `rt_control_bringup`；
- `robot_description`、公共接口 vendor 和 `rt_control_interfaces` 不得依赖任何域实现包；
- 功能包之间新增依赖前，必须证明不能通过已有 ros2_control/ROS 接口解耦；
- rt-control 不接收 BT 节点、工位、箱子、抓取序列等任务层语义；
- 新的跨域命令必须先在公共契约明确责任域、调用方向、超时、取消、幂等和错误语义，
  再更新本仓库的 `robot_interfaces` 固定 SHA；不得放入 `rt_control_interfaces`。

### 3.3 `robot_description` 权威事实源

`src/description/robot_description` 是本机器人模型的单一权威源码，逻辑所有者是平台/系统架构，而不是某个业务域。当前由 rt-control 运行 `robot_state_publisher` 只是运行时部署决定，不改变模型所有权。

必须遵守：

- 各容器消费同一 Git 提交或同一版本发布产物中的 `robot_description`，不得复制一份后独立修改；
- 下游域需要修改模型时，必须在本权威包提交变更并通知所有消费者验证；
- 容器镜像或发布清单应能追溯 `robot_description` 的 Git SHA/版本；同一机器人实例上运行的相关域必须使用相同模型版本；
- URDF/Xacro 中只保存机器人固有事实。EtherCAT PDO/SDO、CANopen、ros2_control 插件、控制器增益、规划参数、工位、抓取/放置位姿、相机拍照位和 Demo 配置不得写入该包；
- 总线和 ros2_control 拼装保留在 `robot_hw_*` 与 `rt_control_bringup`；规划语义放在 motion；任务语义放在 autonomy；标定结果应进入独立的版本化 calibration/config 资产，而不是篡改基础模型；
- link/joint 名称、TF 父子关系、关节方向、零位、限制或 mesh 尺寸变化属于破坏性模型变更，必须联合验证 rt-control、motion、perception 和 TF 消费者；
- 未经实测、图纸或已确认标定数据支持，不得修改惯量、关节限制、原点、轴向或碰撞几何。

### 3.4 接口契约

修改公共 `robot_*_interfaces` 契约或本仓库 vendor pin 时，必须先有同批公共契约变更，并检查所有跨域生产者
和消费者。修改 `rt_control_interfaces` 时，只检查本域生产者和消费者，且不得扩大为跨域依赖。
接口评审至少覆盖：

- 单位、坐标系、时间戳和标识符是否明确；
- request/goal 的合法性约束以及 reject 条件；
- response/result 与 feedback 的终态、错误码和诊断信息；
- 超时、取消、抢占、重复请求和节点重启后的语义；
- 新旧版本兼容性以及是否需要同步升级多个容器；
- 高频状态是否错误地设计成 service/action，长任务是否错误地设计成 topic/service。

删除、重命名、改变字段类型/单位/语义均视为破坏性变更。没有消费者迁移方案和联合验证，不得提交破坏性接口变更。

### 3.5 上游源码与补丁

- 上游源码只通过 `deps.repos` 的完整 commit SHA 锁定，禁止将完整上游仓库复制进本仓库；
- 每个补丁只解决一个清晰问题，保持最小差异，并能对冻结提交执行 `git apply --check`；
- 更新上游 SHA 时必须重新验证全部相关补丁、构建、行为和迁移差异；
- 不得借补丁重写上游 CiA 402、controller_manager 或 ros2_control 的通用行为；
- 修改补丁时同步更新适用顺序、设计依据、验证证据和必要的回退说明；
- 不得直接编辑构建目录中由上游导入或生成的文件。

### 3.6 实时与安全边界

250 Hz 或其他实时路径中禁止引入无界循环、阻塞 IO、文件/网络访问、动态参数查询、日志刷屏，以及每周期内不受控的内存分配、锁竞争或容器增长。非实时回调与实时更新之间使用预分配、边界明确且已验证的通信方式。

以下裁决不得被变相恢复：

- 不新增独立 `rt_watchdog` 包；`src/rt_control/rt_watchdog` 即使作为本地空目录存在，也不得加入源码；
- 不恢复 motion/autonomy 心跳、原始 CAN 旁路监听器或另一套独立总线超时判定；
- 不做 CANopen 节点的自动局部 NMT 恢复；故障恢复遵循已裁决的完整 rt-control 重启流程；
- 不用软件诊断替代急停、安全链、驱动器保护和机械限位；
- 不为 CPU 绑核、EtherCAT/CAN 设备、驱动器对象字典或安全阈值提供猜测性的默认值。

生命周期、取消、停机和故障路径与成功路径同等重要。任何执行器相关变更都必须检查 configure/activate/deactivate/cleanup/error、进程信号退出、通信中断和部分设备未就绪场景，最终状态必须是可解释且保守的。

### 3.7 容器与宿主机边界

- 生产容器必须通过安装后的 `rt_control_start` 作为 PID 1；不得用 `ros2 run` 包裹它，也不得直接启动 launch 文件绕过失能等待；
- Compose 一律通过 `tools/rt_control_compose.sh` 调用；`RT_CONTROL_CPUSET` 必须来自目标机验证，不得设置仓库默认值；
- 不得增加 `privileged: true`、扩大设备映射、capability、主机目录写权限或 DDS 暴露范围，除非有明确需求和风险评审；
- 镜像依赖必须锁定且可追溯；不得在仓库、镜像环境变量、构建日志或文档中写入代理凭据、令牌或私有地址；
- `hostsetup` 脚本会修改 GRUB、systemd、内核模块、网络和总线。没有用户对目标主机及动作的明确授权，只能静态检查，禁止执行安装、启停、重启或写设备操作；
- 实机命令不得由“实现代码”任务自动推导为已授权。上电、使能、运动、写 SDO、重启总线和启动生产 Compose 都需要单独明确授权。

## 4. 标准变更流程

每个任务按以下顺序执行：

1. **界定范围**：列出任务/需求号、目标文件、直接消费者和明确不改的内容；
2. **查明事实**：从源码、冻结依赖、设备文档、既有裁决或实测记录取证；
3. **处理阻塞项**：事实缺失或裁决冲突时更新 `domains/rt_control/BLOCKED-questions.md`，不得自行猜测；
4. **设计最小变更**：保持现有依赖方向和运行入口，先定义验收条件；
5. **实现与测试**：只改任务所需文件，优先补自动化测试或检查脚本；
6. **差异自审**：逐行审查工作树和暂存区，排除生成物、调试代码和无关改动；
7. **按风险验证**：执行第 5 节适用的静态、构建、mock、容器或实机门禁；
8. **记录证据**：在 `domains/rt_control/docs/areas/` 对应功能区新增一条开发记录
   （格式必须使用 `docs/areas/TEMPLATE.md`，归属与记录规则见 `docs/areas/README.md`），
   同步维护该区 README 的冻结事实表，并在 `domains/rt_control/PROGRESS.md` 时间线
   追加一行索引；写清命令、结果、未执行项和原因；
9. **交付**：以“变更、架构影响、验证、未验证/风险”的格式汇报。

用户已有改动必须保留。除非用户明确要求，不得通过 `git checkout --`、`git reset --hard`、清理工作树、覆盖文件或改写提交历史处理它们。

## 5. 验证门禁

### 5.1 所有变更的最低门禁

日常增量开发（内环）优先使用 `tools/run_scoped_tests.sh`：按改动范围自动裁剪到
repository_gate / quality_gate / `colcon test --packages-above <受影响包>`，命中
patches、deps.repos、versions.env、docker、CI 等全局影响路径时会提示改跑全量。
增量漏掉的跨包影响由 PR 的 CI 全量兜底；发布门禁走 `tools/release_test_runner.py`，
不得以增量结果替代。

提交或上传前至少执行：

```bash
git status --short --branch
git diff --check
git diff --stat
git diff
git diff --cached --check
git diff --cached
tools/quality_gate.sh
```

`git diff` 不显示未跟踪文件；必须逐项审查 `git status` 中的 `??`，并在提交前通过暂存区差异再次检查其完整内容。

同时确认：

- 变更只覆盖本任务，所有新文件均为预期文件；
- `build/`、`install/`、`log/`、`.colcon/`、`__pycache__/`、`*.pyc` 和临时归档未进入提交；
- 没有密钥、令牌、密码、证书私钥、代理凭据、个人路径或现场敏感数据；
- 没有注释掉的旧实现、调试旁路、无限重试、静默错误或伪造成功状态；
- 新增失败路径能给出稳定错误码/诊断，资源和生命周期能正确收尾；
- 文档声称的每项测试确实执行且结果一致；未执行的验证被明确列出。

### 5.2 ROS 包和 C++

先 source ROS 2 Humble，再按最小受影响闭包构建；公共头文件、接口或共享配置变化时扩大到所有下游包：

```bash
source /opt/ros/humble/setup.bash
colcon list --base-paths src
colcon build --symlink-install --packages-select <affected-packages>
colcon test --packages-select <affected-packages>
colcon test-result --verbose
```

若仓库当前没有相应自动化测试，`colcon test` 的“无测试”不能替代验证；必须增加针对性检查，或在交付中明确测试缺口。C++ 变更还要人工检查实时路径的分配、锁、阻塞、线程安全、单位/符号、边界、生命周期和错误传播。

### 5.3 `robot_description`

至少完成模型展开、URDF 校验、包构建和污染检查：

```bash
source /opt/ros/humble/setup.bash
xacro src/description/robot_description/urdf/robot.urdf.xacro > /tmp/robot-description-review.urdf
check_urdf /tmp/robot-description-review.urdf
colcon build --symlink-install --packages-select robot_description
python3 tools/repository_gate.py
```

结构性变更还必须检查 TF 树、关节限制、碰撞模型、mesh 路径以及所有相关容器使用的模型版本。禁止仅凭 RViz “看起来正常”判定通过。

### 5.4 ROS 接口

修改接口时先导入 `deps.repos`，再至少构建公共 vendor 包、域内包及全部 RT-Control 消费者，
并对仓库全局搜索旧字段/旧名称：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  robot_rt_control_interfaces robot_system_interfaces robot_interfaces_qos rt_control_interfaces \
  control_api_adapter plc_node enable_manager rt_control_bringup
rg -n "<changed-interface-or-field>" src docker tools domains README.md AGENTS.md
```

交付记录必须列出生产者、消费者、兼容性结论及是否要求多个容器原子升级。

### 5.5 EtherCAT、CANopen 与控制策略

相关变更按适用范围执行：

```bash
bash tools/check_ecat_sync_shutdown_policy.sh
python3 tools/diff_legacy.py --legacy-root /home/kkozia/robot_driver --target-root .
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_hw_ethercat robot_hw_canopen enable_manager rt_diagnostics rt_control_bringup
```

只有在 `/home/kkozia/robot_driver` 确认处于冻结基线提交时，迁移差异结果才有效。CANopen 生成物必须从权威 EDS 和 `bus.yml` 重建，不得手改 `.bin/.dcf`。`canopen_sdo_archive.sh` 只能用于明确授权的只读归档；不得把“读取归档”扩大成 SDO 写入。

补丁要在 `deps.repos` 指定的上游提交上执行 `git apply --check`，并构建/测试受影响的上游包。优先用隔离临时工作区或完整 Docker 构建验证，禁止污染权威源码或遗留基线。

### 5.6 Launch、Docker、脚本和宿主配置

- 改动 shell 脚本：对所有修改的脚本执行 `bash -n`；可用时执行 `shellcheck`；
- 改动 Python/launch：执行 `python3 -m py_compile <changed-files>`，并运行适用的 launch/mock 检查；
- 改动 Compose：设置经验证的临时 `RT_CONTROL_CPUSET`，通过 `tools/rt_control_compose.sh config` 检查展开结果；不得为通过校验而把该值写成仓库默认；
- 改动 Dockerfile、`deps.repos`、`versions.env` 或补丁：需要完整镜像构建，或明确说明为何当前环境不能执行；
- 改动 systemd unit：先用副本或离线方式执行 `systemd-analyze verify`；
- 改动 `hostsetup`：只做语法和静态审查，除非用户明确授权在已确认目标机执行。

### 5.7 Mock 与实机验证分级

验证按以下顺序升级，前一级失败不得进入后一级：

1. 静态检查与配置/schema 校验；
2. 受影响包构建和自动化测试；
3. `use_mock_hardware:=true` 的加载、生命周期和停机验证；
4. 容器 config/镜像构建/无硬件启动验证；
5. 断动力或不使能的现场通信验证；
6. 有急停、隔离区、限速、监护人和回退方案的上电低速验证；
7. 完整生产验收。

AI 不得把 mock、编译通过或离线配置检查描述成“实机验证通过”。

## 6. 提交、推送和上传规则

完成代码不等于自动获得提交或推送权限。只有用户明确要求时，AI 才能执行 `git commit`、`git push`、创建 PR、打 tag 或上传发布物。

在被明确要求提交/推送时，必须先：

1. 完成第 5 节全部适用门禁；
2. 重新审查 `git diff --cached`，确认暂存区没有用户无关改动；
3. 更新 `domains/rt_control/docs/areas/` 对应功能区记录与 `domains/rt_control/PROGRESS.md` 时间线及必要文档，确保结果可追溯；
4. 使用带任务号/需求号的提交信息，例如：

   ```text
   fix(T-014): pin CANopen modes by node [REQ-CAN-004, REQ-CAN-006]
   ```

5. 在推送前核对目标 remote、branch 和提交范围；
6. 推送后报告远端分支和提交 SHA。

禁止使用 `--no-verify`、强制推送、擅自 amend 他人提交、改写共享历史或把多个无关任务压进同一提交。测试失败、架构不合规、事实未确认或安全风险未闭环时，不得以“先上传再修”为理由绕过门禁。

## 7. AI 自审输出格式

每次交付或提交前，AI 必须按以下结构自审；发现问题时先报告问题，不得只给“已完成”：

```text
变更范围：
- 任务/需求：
- 文件/包：
- 明确未修改：

架构审查：
- 域边界与依赖方向：PASS/FAIL
- robot_description/接口兼容性：PASS/FAIL/N/A
- 实时与安全约束：PASS/FAIL/N/A
- 生命周期、取消与停机：PASS/FAIL/N/A

验证证据：
- <实际执行的命令>：PASS/FAIL

未验证与剩余风险：
- <未执行项、原因、需要谁在什么条件下完成>

Git 审查：
- 无关改动：YES/NO
- 生成物或敏感信息：YES/NO
- 是否允许提交/推送：YES/NO（依据）
```

代码审查结论按严重程度排列：`BLOCKER`、`HIGH`、`MEDIUM`、`LOW`，并标注文件和行号。若没有发现问题，也要明确写“未发现阻断提交的问题”，但仍需列出测试缺口和剩余风险。

## 8. 必须暂停并请求确认的情形

出现以下任一情形，停止相关写入或执行，向用户提供证据和最小问题：

- 新需求与 `domains/rt_control/BLOCKED-questions.md`、冻结配置或已验收行为冲突；
- 需要猜测硬件标识、对象字典、单位、比例、方向、安全阈值或 CPU 拓扑；
- 需要破坏性修改 `robot_description` 或跨域接口，但消费者和升级顺序不明确；
- 需要扩大容器权限、设备访问、网络暴露或宿主机写权限；
- 需要运行 root 脚本、修改宿主机、启停总线、写 SDO、使能或驱动实机；
- 现有用户改动与任务文件重叠且无法安全合并；
- 适用的构建/测试失败，或验证环境不足以支持宣称的完成状态；
- 即将提交的内容无法追溯到明确任务、需求或裁决。

本契约的目的不是阻止迭代，而是确保每次迭代都能回答：谁拥有事实、谁消费接口、失败时如何收尾、用什么证据证明没有破坏机器人。
