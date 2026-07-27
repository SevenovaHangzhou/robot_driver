# PLC / BMS 合并、发布与实机验证交接

更新时间：2026-07-28
接手对象：已完成 T-020 的后续 AI / rt-control 负责人

## 1. 当前状态

> 执行进展（2026-07-28）：首次候选 `e4fed685…` 在目标机完成 BMS/PLC 读取，但暴露关停期间 IO 节点 respawn，
> 100 秒后 exit 137，已拒绝。修复提交为 `d415c0c2c75917a9545a4a2f87487718de8622a2`，新镜像 ID 为
> `sha256:01bd550b068fccb9158b007067e55c30eed7d7d7253ef9179dfdf6d9be9a11c2`；带 IO Mock 已在 2 秒内干净退出，
> 本提交把阶段 B 锁更新到该修复身份。本节以下“未提交工作区”等文字保留为交接时点的历史记录，不代表当前 Git
> 状态。后续目标机重复停止、三路 PLC 输出逐点 ON/OFF 和完整一键 start→READY→stop 均已通过，操作锁为
> `7bc8f16e903ef7174f60ebe9613f6eeed3a96185`。最终证据见
> [PLC / BMS 与一键启动实机验收](plc-bms-commissioning-20260728.md)。

工作区基线是：

```text
d2c146fb7cf6bd56832fdaed3384fd887a9de405
feat(T-020): publish navigation and vision TF tree
```

T-020 已经提交；PLC/BMS 改动全部位于该提交之上的**未提交工作区**。本轮没有执行 `git add`、`git commit`、
`git push`、Docker 正式镜像构建、目标机部署或硬件访问。

接手后先执行：

```bash
cd /home/kkozia/robot
git rev-parse HEAD
git status --short
git diff --check
```

预期 HEAD 为上面的 `d2c146f...`，并能看到本交接列出的修改和新增文件。不要执行 `git reset --hard`、
`git checkout -- .`、`git clean`，也不要重新整包 merge 同事远端分支；当前工作区已经是基于 T-020 的选择性集成结果。

## 2. 已完成的选择性集成

### BMS

- 仅保留 `bms_node`，没有合入 `can_bus_guard`。
- 只接收标准 CAN 数据帧 `0x3FC`。
- byte 0-1：大端无符号数 × 0.1 V。
- byte 4：SOC 百分数，发布时换算为 0.0-1.0。
- 唯一话题：`/bms/battery_state`，类型 `sensor_msgs/msg/BatteryState`，周期 5 秒。
- 只承诺 `voltage` 和 `percentage`；其他浮点字段为 `NaN`。
- 有效帧超过 3 秒未更新时，电压/SOC 均发布 `NaN`，`present=false`。
- 节点只读 `can1`，不配置接口、位率或链路状态。

### PLC

固定映射：

| 地址 | 语义 |
| --- | --- |
| `%MW200 bit0` | 左臂电磁阀命令，1=开 |
| `%MW200 bit1` | 右臂电磁阀命令，1=开 |
| `%MW200 bit2` | 共用真空泵命令，1=开 |
| `%MW201 bit0` | 远程控制允许，必须保持 1 |
| `%MW210 bit0` / `IX0.6` | 左臂真空已建立，1=true |
| `%MW210 bit1` / `IX0.7` | 右臂真空已建立，1=true |
| `%MW211 bit0-bit2` | 三路实际输出状态 |
| `%MW212` | IO 报警字 |

ROS 接口只保留：

- `/plc/io_state`：`robot_interfaces/msg/PlcIoState`，0.5 秒一次。
- `/plc/left_solenoid`：`std_srvs/srv/SetBool`。
- `/plc/right_solenoid`：`std_srvs/srv/SetBool`。
- `/plc/vacuum_pump`：`std_srvs/srv/SetBool`。

已经删除 `/plc/command`、三路 Bool command topic、`/plc/inputs`、`/plc/outputs` 和 JSON `/plc/status`。

每个服务都执行：读取 `%MW200` → 只更新目标 bit → 写回 → 最多等待 1 秒，要求 `%MW200` 命令位与
`%MW211` 实际位同时符合目标。失败返回 `success=false`，不做补偿写。启动、断连、重连和节点退出不会强制清零
`%MW200`。

连接、重连、服务调用和周期轮询都会检查 `%MW201 bit0`。发现为 0 时写入 `原值 | 0x0001` 并回读确认，且要求
回读全字与目标值一致；非目标 bit 变化同样判为失败。修复失败会阻止服务写输出，但仍尽可能发布 PLC 只读状态。

详细接口说明见 [plc-bms-integration.md](plc-bms-integration.md)。

## 3. 与 T-020 合并时必须保留的内容

当前改动已经建立在 T-020 提交之上，正常情况不需要解决历史冲突，只需审查并提交工作区。以下文件同时承载 T-020
或既有部署策略，提交前重点复核：

| 文件 | 必须同时保留 |
| --- | --- |
| `docker/rt-control/Dockerfile` | T-020 的 `robot_state_publisher` 可执行文件检查；新增 `bms_node`、`plc_node` 构建清单 |
| `rt_control.launch.py` | T-020 的 RSP 50 Hz；新增 PLC/BMS 条件节点；原 controller 启动顺序不变 |
| `docker/compose.yaml` | 单一 `rt-control` 服务、原 PID 1、CPU/设备门禁；新增 `NET_RAW` 和两个启动环境变量 |
| `one-command-start.md` | T-020 TF 发布边界；新增 PLC/BMS 发布边界和接口 |
| `repository_gate.py` | 原容器边界；能力白名单扩展为 `SYS_NICE`、`IPC_LOCK`、`NET_RAW` |

禁止重新引入：

- `can_bus_guard` 包或 BMS 对它的依赖；
- 第二个 `rt-io` 容器、第二套 Compose、`rt_io_start`；
- PLC 批量 command topic 或单路 Bool command topic；
- BMS 电流、容量、告警、JSON 和重复数值话题；
- PLC heartbeat `%MW202` 的未使用代码；
- 在容器内配置 `can1` 所需的 `NET_ADMIN`；
- 退出时自动关闭泵或电磁阀的行为。

## 4. 主要文件

新增实现：

- `src/rt_control/bms_node/`
- `src/rt_control/plc_node/`
- `src/interfaces/robot_interfaces/msg/PlcIoState.msg`
- `src/rt_control/rt_control_bringup/config/rt_io.yaml`
- `tools/tests/test_rt_io_integration.py`
- `domains/rt_control/docs/plc-bms-integration.md`

主要修改：

- `src/rt_control/rt_control_bringup/launch/rt_control.launch.py`
- `src/rt_control/rt_control_bringup/package.xml`
- `docker/rt-control/Dockerfile`
- `docker/compose.yaml`
- `tools/rt_control_ipc.sh`
- `tools/check_rt_control_ipc_launcher_policy.sh`
- `tools/repository_gate.py`
- `tools/tests/test_repository_gate.py`
- `domains/rt_control/BLOCKED-questions.md`
- `domains/rt_control/PROGRESS.md`

## 5. 已完成验证

已完成且通过：

- `robot_interfaces`、`bms_node`、`plc_node`、`rt_control_bringup` 受影响包构建；
- BMS/PLC 共 47 项测试，0 failure；
- `ament_flake8`，17 个相关 Python 文件无问题；
- `tools/quality_gate.sh`：26 项仓库测试通过，分支覆盖率 84%；
- `RT_CONTROL_CPUSET=14 tools/rt_control_compose.sh config`；
- `ros2 launch rt_control_bringup rt_control.launch.py --show-args`；
- `git diff --check`。

复跑建议：

```bash
source /opt/ros/humble/setup.bash
source /home/kkozia/robot/install/setup.bash

colcon --log-base .colcon/rt_io_log build \
  --base-paths src/interfaces src/rt_control \
  --build-base .colcon/rt_io_build \
  --install-base .colcon/rt_io_install \
  --packages-select robot_interfaces bms_node plc_node rt_control_bringup

source /home/kkozia/robot/.colcon/rt_io_install/setup.bash
colcon --log-base .colcon/rt_io_test_log test \
  --build-base .colcon/rt_io_build \
  --install-base .colcon/rt_io_install \
  --packages-select robot_interfaces bms_node plc_node

colcon --log-base .colcon/rt_io_test_result test-result \
  --test-result-base .colcon/rt_io_build --verbose

tools/quality_gate.sh
RT_CONTROL_CPUSET=14 tools/rt_control_compose.sh config >/dev/null
git diff --check
```

注意：在只包含仓库本体的局部 `--base-paths` 下使用 `--packages-up-to rt_control_bringup`，会尝试重编既有
`robot_hw_canopen`，若 vendor `lely_core_libraries` 未加载会在无关依赖阶段失败。局部复核使用上面的
`--packages-select`；正式 Docker 构建会按 `deps.repos` 导入完整 vendor 依赖。

本机没有 ShellCheck，质量门禁只报告本地 notice；提交前应在安装 ShellCheck 的环境或 CI 再跑一次。

## 6. 建议的两阶段提交与不可变镜像流程

现有 `tools/rt_control_ipc.sh` 仍锁定旧 runtime `4fc8414...`，这是有意保留的发布门禁。不要在脏工作区直接运行
Compose build：包装器会使用 HEAD 作为镜像 tag，脏源码会造成“tag 指向干净提交、镜像内容却不同”的身份错误。

### 阶段 A：功能提交与镜像

1. 审查当前 diff，确认 T-020 内容没有被回退。
2. 按仓库授权完成一次 PLC/BMS 功能提交；记录完整 `IO_SHA`。
3. 确认 `git status --short` 为空。
4. 运行完整 Docker 构建：

```bash
export RT_CONTROL_CPUSET=14
tools/rt_control_compose.sh build rt-control
docker image inspect "rt-control:$(git rev-parse HEAD)" --format '{{.Id}}'
```

5. 记录镜像 ID，完成容器内包、接口、CMD/PID 1、capability 和 Compose 展开检查。

### 阶段 B：发布锁提交

镜像通过后，再做一个单独的发布锁提交：

- `tools/rt_control_ipc.sh`：
  - `runtime_sha="<IO_SHA>"`
  - `runtime_image="rt-control:<IO_SHA>"`
  - `runtime_image_id="<实际镜像 ID>"`
  - `runtime_root="/home/ar/rt-control-releases/<IO_SHA>/robot"`
- `tools/check_rt_control_ipc_launcher_policy.sh`：同步更新 SHA 和镜像 ID 的精确断言。
- `PROGRESS.md` / 发布文档：记录构建和未验证边界。

然后再次运行 `tools/quality_gate.sh`。运行 release 使用阶段 A 的 `IO_SHA`；阶段 B 只是锁定并指向该不可变运行副本。

## 7. 目标机部署前门禁

遵循 [deployment-operations-runbook.md](deployment-operations-runbook.md)，并至少确认：

- 目标机、实时内核、CPU14、EtherCAT 和现有 CANopen 门禁仍符合当前锁；
- `can0` 序列号仍为 `004D00675230500720333159`；
- `can1` 序列号仍为 `003000265230500720333159`；
- `can1` 已由宿主配置为 UP、500 kbit/s、ERROR-ACTIVE；容器不得修改它；
- `candump can1` 能看到 `0x3FC`；
- PLC 地址仍为 `192.168.1.88:502`，目标机网口仍为 `enp4s0`；
- PLC/电气人员确认 `%MW200/201/210/211/212` 和左右映射未变化；
- 明确告知现场：PLC 节点启动后可能自动把 `%MW201 bit0` 从 0 写成 1；
- 同 CPU14 是临时风险接受，不得据此关闭实时调度/性能验证项。

生产一键命令会自动调用 `/rt/enable`。第一次仅验证 PLC/BMS 时，不要直接运行一键启动；先由现场负责人批准一个
“不自动使能 14 轴”的 commissioning 步骤。不得为了方便修改生产 Compose 的 CMD/entrypoint 或绕过最终 PID 1
关停链。只有 PLC/BMS 单项验证完成、运动区域清空、急停/STO 可用并获得现场确认后，才执行完整一键 start→READY→stop。

## 8. 实机验证顺序与验收标准

### A. BMS 被动验证

1. 保存 `can1` 链路详情、序列号和一段包含 `0x3FC` 的原始帧。
2. 启动 BMS 节点后检查唯一话题：

```bash
ros2 topic type /bms/battery_state
ros2 topic info /bms/battery_state --verbose
ros2 topic echo /bms/battery_state
timeout 35 ros2 topic hz /bms/battery_state
```

验收：

- 类型为 `sensor_msgs/msg/BatteryState`；
- 只有 `bms_node` 发布，不存在旧 BMS 重复话题；
- 频率约 0.2 Hz；
- 电压与 BMS/HMI 对照一致，SOC 为 0.0-1.0；
- 如获准做失帧测试，下一次发布应为电压/SOC `NaN`、`present=false`；不得从容器执行 `ip link set`。

### B. PLC 只读与远程允许验证

```bash
ros2 topic type /plc/io_state
ros2 topic echo /plc/io_state
timeout 10 ros2 topic hz /plc/io_state
ros2 service list -t | grep '^/plc/'
```

验收：

- `connected=true`、`data_fresh=true`，频率约 2 Hz；
- 只有三个 `SetBool` 服务，没有 `/plc/command` 和三个 `/command` topic；
- `%MW210 bit0/bit1` 与左右真空传感器现场状态一致；
- `%MW211 bit0-bit2` 与实际左右电磁阀/泵输出一致；
- `%MW212` 与 PLC 工程工具报警字一致；
- 若 PLC 人员获准把 `%MW201 bit0` 清零，节点应只恢复 bit0，并保持该字其他 bit 不变；失败时服务必须拒绝写输出。

### C. 三路输出逐点验证

必须由 PLC/电气和现场安全负责人确认后进行。每次先保存 `%MW200`、`%MW211` 和 `/plc/io_state` 基线，然后一次只测
一个服务：

```bash
ros2 service call /plc/left_solenoid std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/left_solenoid std_srvs/srv/SetBool "{data: false}"

ros2 service call /plc/right_solenoid std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/right_solenoid std_srvs/srv/SetBool "{data: false}"

ros2 service call /plc/vacuum_pump std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/vacuum_pump std_srvs/srv/SetBool "{data: false}"
```

每一步验收：

- service 返回 `success=true`；
- `%MW200` 和 `%MW211` 只有目标 bit 变化；
- `%MW200` 其他 15 bit 完全保持；
- 左右实际设备没有互换，泵确为共用泵；
- 建立真空后，对应 `%MW210` bit 与 ROS 左/右 vacuum 字段变为 true；
- 关闭后状态按 PLC/气路设计恢复。

不要在未建立安全工况时测试“输出保持跨重启”。源码明确不会在退出时清零输出，因此如果停机时输出保持存在危险，
应由安全 PLC/电气回路和现场操作流程处理，不能临时在 ROS 节点里加入自动全关。

### D. 同容器与一键启动验证

镜像/release 锁更新后，检查：

- Compose 仍只有 `rt-control` 一个服务；
- PID 1 仍是安装后的 `rt_control_start`；
- capability 精确为 `SYS_NICE`、`IPC_LOCK`、`NET_RAW`，没有 privileged/`NET_ADMIN`；
- PLC/BMS 与控制栈都在同一 CPU14 cpuset；
- 一键脚本启动前验证 `can1` 和 `0x3FC`；
- 启动后必须看到 PLC fresh、BMS 非 NaN，才调用 `/rt/enable`；
- 完整 start→READY→stop 后，运动控制仍按既有顺序失能/退出，容器 exit 0，EtherCAT 回 Idle/Inactive、16 从站 PREOP；
- 停止过程中没有第二个 `robot-rt-io-1`，也没有 PLC 输出自动清零。

## 9. 必须保存的实机证据

- 功能提交 SHA、锁提交 SHA、镜像 tag、镜像 ID、release 路径；
- 构建日志、`tools/quality_gate.sh` 输出和 Compose 展开文件；
- `docker inspect` 的 PID 1、cpuset、capability、device、environment；
- `can1` 序列号/位率/错误计数和原始 `0x3FC` 片段；
- BMS 电压/SOC 与 HMI 对照；
- PLC 每次服务前后的 `%MW200/%MW201/%MW210/%MW211/%MW212`；
- 三个 service 的完整响应和 `/plc/io_state`；
- 一键 start、READY、stop、最终总线状态和容器退出码；
- CPU14 调度/负载观测，明确注明这是临时同核数据，不是生产实时性关闭证据。

任一映射相反、非目标 bit 变化、PLC 回读不一致、BMS 数值不合理、节点重复发布、容器权限扩大、PID 1 改变或关停链
退化，都应停止联调并回退到旧不可变 release，不得现场热改源码后继续。

## 10. 完成定义

只有以下条件全部满足，才可以把本项从“源码集成完成”更新为“目标机可用于联调”：

- 工作区改动经过审查并形成干净功能提交；
- 新镜像从该提交构建，镜像 ID 已记录；
- release 与 launcher/policy 锁完成第二阶段提交；
- 目标机 BMS、PLC 只读和三路输出逐点验证通过；
- 同容器一键 start→READY→stop 通过；
- 全部证据进入域文档/PROGRESS，未把临时 CPU14 共核运行写成生产实时性验收完成。

2026-07-28 执行结果：上述发布、读取、输出逐点和一键启停的软件/寄存器门禁已满足，当前目标机可进入工程联调。
独立 BMS HMI 目视对照及左右实体/共用泵气路的人工对应关系没有由本次 SSH 证据覆盖，保留为现场联调观察项，不得
写成已完成的机械、电气或工艺验收。CPU14 共核仍是临时风险接受，不构成生产实时性关闭证据。
