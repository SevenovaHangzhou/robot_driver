# rt-control 域

This directory owns the development contract and operating notes for the ROS 2
Humble real-time control domain in the dedicated `robot_driver` repository. The runtime
source and deployment assets remain in their technical top-level directories;
this directory is their single domain-level documentation and governance home.

## 新同事从这里开始

- [Docker 部署与性能验证](docs/docker-deployment-performance-summary.md)：当前工控机上的镜像、功能、重要配置、性能结论和通信风险。
- [开发进度与联调准入](docs/integration-readiness-summary.md)：当前可用接口、已完成能力、联调边界以及域间契约/安全诊断等剩余任务。
- [一键启动](docs/one-command-start.md)：当前工控机上启动、自动使能、查看状态/日志和有序停止的最短说明。
- [原生开发与运行](docs/native-development-workflow.md)：在目标机文件夹中增量编译、启动、显式使能和停止，不必每次重构镜像。
- [PLC / BMS 同容器集成](docs/plc-bms-integration.md)：精简接口、确认后的 IO 映射、保留位写入、中央配置和发布边界。
- [PLC / BMS 与一键启动实机验收](docs/plc-bms-commissioning-20260728.md)：关停修复、三路输出逐点开关、无命令使能保持、末态总线和剩余人工观察项。
- [PLC / BMS 合并与实机交接](docs/plc-bms-merge-hardware-handoff.md)：T-020 基线、提交/镜像锁流程、目标机验证顺序和证据清单。
- [接手知识图谱](docs/onboarding-knowledge-map.md)：域边界、包依赖、启动/执行/故障/关停链、按任务找代码和推荐阅读顺序。
- [新机部署与运行手册](docs/deployment-operations-runbook.md)：新机准入、镜像交付、宿主配置、Mock、生产启动、日常使用、故障恢复和回退。
- [后续修改计划](docs/follow-up-modification-plan.md)：主接触器急停掉电恢复入口和 `/wheel/odom`/TF 所有权调整；两项均已进入源码，生产镜像与联合/实机验收尚未完成。
- [XMC SW 5.11 固定 PDO 映射](docs/xmc-updown-sw511-fixed-pdo.md)：Updown 从站身份、19/24-byte 固定 PDO、启动 SDO、单位换算、XML 差异和实机门禁。
- [XMC 首次整组使能记录](docs/xmc-updown-enable-commissioning-20260727.md)：14 轴 reset/enable/disable 实机结果，以及 WC 增长和联合退出段错误阻塞项。
- [CANopen 有序清理与 EtherCAT 同步容忍复测](docs/canopen-shutdown-sync-tolerance-commissioning-20260727.md)：`0x10F1:02=250` 首镜像门禁、三轮联合退出、BQ-119/BQ-122 闭环和剩余风险。
- [PROGRESS.md](PROGRESS.md)：已经验证和仍未验证的范围，不能把构建或 Mock 结果当成实机验收。
- [BLOCKED-questions.md](BLOCKED-questions.md)：已裁决和仍开放的硬件、接口与安全事实；按关键词或 BQ 编号搜索，并注意后续裁决可能取代早期记录。

Domain records:

- [AGENTS.md](AGENTS.md): rt-control architecture, safety, validation, and AI rules.
- [PROGRESS.md](PROGRESS.md): implementation and verification history.
- [BLOCKED-questions.md](BLOCKED-questions.md): adjudicated and unresolved domain facts.
- [docs/](docs/): onboarding, deployment, design evidence, commissioning notes, and host records.

The source layout follows the frozen rt_control implementation specification:

- `src/interfaces/robot_control_interfaces`、`robot_system_interfaces`: 公共契约 0.5.0 的本地构建镜像。
- `src/interfaces/rt_control_interfaces`: 仅供本域使用的 `PlcIoState`、`RtEnable` 等私有接口。
- `src/description/robot_description`: kinematics-only robot description.
- `src/rt_control`: EtherCAT/CANopen configuration, bringup, enable/fault handling, and diagnostics packages.
- `docker/rt-control` and `docker/compose.yaml`: rt-control image and current Compose deployment.
- `tools`: migration and commissioning tools.
- `hostsetup`: target-host setup assets.

The legacy baseline `/home/kkozia/robot_driver@6bc94cd` is read-only and is not vendored in this repository. External source checkouts are declared in `deps.repos`.

Build the in-repository packages with:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

目标工控机的日常开发使用独立工作区 `/home/ar/rt-control-dev`，由
[`bootstrap_native_dev.sh`](../../tools/bootstrap_native_dev.sh) 管理冻结依赖和增量构建，
由 [`rt_control_native.sh`](../../tools/rt_control_native.sh) 管理真实硬件进程。原生运行与
Docker 运行互斥；Docker 继续作为里程碑发布、回归和交付载体。详见
[原生开发与运行](docs/native-development-workflow.md)。

Build or expand the rt-control image configuration only after exporting the
target host's validated isolated CPU set:

```bash
export RT_CONTROL_CPUSET=<validated-isolated-cpu-list>
tools/rt_control_compose.sh config
tools/rt_control_compose.sh build rt-control
```

Starting the production service activates the real CANopen hardware path and
may put those drives into an operational/energized state even before
`/rt/enable`; it also accesses real EtherCAT. Follow the staged authorization,
inspection, startup, and shutdown procedure in the
[deployment and operations runbook](docs/deployment-operations-runbook.md);
do not treat `up` as a hardware-free smoke test.

The wrapper reads the pinned IgH version and full commit from `versions.env`.
In a Git checkout it tags the image with the current repository commit. For an
audited non-Git release export it requires both `RT_CONTROL_PROJECT_ROOT` and an
explicit full `RT_CONTROL_IMAGE_TAG`; the current-IPC launcher supplies the
locked values and does not infer them from the directory contents. The dependency identity
is also stored in the image labels and
`/usr/local/share/rt-control/dependency-versions.env`. The wrapper intentionally
has no default CPU set.

The supported container command is the installed `rt_control_start` signal
gate. It waits up to 30 seconds for `/rt/disable` before forwarding an orderly
shutdown to ROS. The image executes the installed file directly so it remains
PID 1; do not wrap it in `ros2 run` when overriding the container command.
Starting `ros2 launch rt_control_bringup rt_control.launch.py` directly also
bypasses that clean-shutdown guarantee.

For hardware-free controller loading checks, use the isolated Mock recipe in
the [deployment and operations runbook](docs/deployment-operations-runbook.md).
It keeps the production `rt_control_start` shutdown gate, selects
`use_mock_hardware:=true`, uses a separate ROS domain, and does not map the
EtherCAT device. The production default remains `false`. If GitHub access alone
needs the host proxy during an image build, set `RT_CONTROL_BUILD_PROXY` to a
host-reachable proxy URL for that invocation; the value is scoped to the
source-import layer and is not stored as a runtime environment variable.

## Development quality gate

Install the local hook dependencies once on an Ubuntu 22.04 development host:

```bash
sudo apt-get install python3-coverage python3-pip python3-yaml shellcheck
python3 -m pip install --user pre-commit==3.7.1
python3 -m pre_commit install
```

Run the same repository gate manually at any time:

```bash
tools/quality_gate.sh
```

The hook checks repository architecture, dependency pins, forbidden generated
artifacts, YAML/XML/Python syntax, shell syntax, policy tests with at least 80%
coverage, and the frozen EtherCAT shutdown policy. GitHub Actions reruns the
same gate with ShellCheck mandatory, validates completion of the pull request
contract, and then builds and tests the ROS workspace. Local hook success does
not authorize hardware access, a commit, or a push; both the root
[`AGENTS.md`](../../AGENTS.md) and this domain's [AGENTS.md](AGENTS.md) still
apply.

After this workflow first succeeds, a repository administrator must configure
the protected `main` branch to require the `governance` and `build` status
checks and to disallow bypasses. Workflow files cannot enable GitHub branch
protection by themselves.
