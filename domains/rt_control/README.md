# rt-control 域

This directory owns the development contract and operating notes for the ROS 2
Humble real-time control domain in the dedicated `robot_driver` repository. The runtime
source and deployment assets remain in their technical top-level directories;
this directory is their single domain-level documentation and governance home.

## 新同事从这里开始

- [一键启动](docs/one-command-start.md)：当前工控机上启动、自动使能、查看状态/日志和有序停止的最短说明。
- [原生开发与运行](docs/native-development-workflow.md)：在目标机文件夹中增量编译、启动、显式使能和停止，不必每次重构镜像。
- [接手知识图谱](docs/onboarding-knowledge-map.md)：域边界、包依赖、启动/执行/故障/关停链、按任务找代码和推荐阅读顺序。
- [新机部署与运行手册](docs/deployment-operations-runbook.md)：新机准入、镜像交付、宿主配置、Mock、生产启动、日常使用、故障恢复和回退。
- [Docker 部署与性能验证](docs/docker-deployment-performance-summary.md)：当前工控机上的镜像、功能、重要配置、性能结论和通信风险。
- [开发进度与联调准入](docs/integration-readiness-summary.md)：联调边界与剩余任务；接口清单以 vendored `robot_interfaces/contract/views/rt_control.md` 为准。
- [PROGRESS.md](PROGRESS.md)：验证时间线索引；不能把构建或 Mock 结果当成实机验收。
- [BLOCKED-questions.md](BLOCKED-questions.md)：已裁决和仍开放的硬件、接口与安全事实；注意后续裁决可能取代早期记录。

专题与历史文档（PLC/BMS、XMC、CANopen 验收记录、后续修改计划等）不再在此逐一罗列，
按功能区索引见 [docs/areas/](docs/areas/README.md) 各区"历史锚点"；
发布前测试体系与用例目录见 [testing/](testing/README.md)。

Domain records:

- [AGENTS.md](AGENTS.md): rt-control architecture, safety, validation, and AI rules.
- [PROGRESS.md](PROGRESS.md): implementation and verification history.
- [BLOCKED-questions.md](BLOCKED-questions.md): adjudicated and unresolved domain facts.
- [docs/](docs/): onboarding, deployment, design evidence, commissioning notes, and host records.
- [docs/areas/](docs/areas/README.md): per-functional-area development records and frozen facts.
- [testing/](testing/README.md): pre-release test tiers, categories, and case catalog.

The source layout follows the frozen rt_control implementation specification:

- `src/vendor/robot_interfaces`: 从固定 SHA 导入的公共接口、契约视图与命名 QoS 包。
- `src/interfaces/source-lock.yaml`: 与 `deps.repos` 一致的公共契约身份元数据。
- `src/interfaces/rt_control_interfaces`: 仅供本域使用的 `PlcIoState`、`RtEnable` 等私有接口。
- `src/description/robot_description`: kinematics-only robot description.
- `src/rt_control`: EtherCAT/CANopen configuration, bringup, enable/fault handling, and diagnostics packages.
- `docker/rt-control` and `docker/compose.yaml`: rt-control image and current Compose deployment.
- `tools`: migration and commissioning tools.
- `hostsetup`: target-host setup assets.

For ELECTRI-94, `driver_variant.yaml` is the RT-Control source of truth for
drive profiles, bus addresses, effective SI conversions, controller roles, and
lifecycle projections. The independently versioned Robot Model remains the
authority for logical joint names and types: the manifest must not add or
remove a whole-body logical joint on its own. Any motor-count change that
changes that logical topology must be made in the authoritative
`robot_description` repository and atomically update this repository's pinned
build copy/version together with every affected consumer.

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
contract, builds and tests the ROS workspace, and builds the production image
before exercising its signal-gated entrypoint with an isolated no-device Mock.
Local hook success does not authorize hardware access, a commit, or a push; both the root
[`AGENTS.md`](../../AGENTS.md) and this domain's [AGENTS.md](AGENTS.md) still
apply.

After this workflow first succeeds, a repository administrator must configure
the protected `main` branch to require the `governance`, `build`, and
`container` status checks and to disallow bypasses. Workflow files cannot
enable GitHub branch protection by themselves.
