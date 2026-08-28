# rt-control 原生开发与运行

本文只适用于目标工控机 `ar-Default-string`。目标是让日常源码修改通过
`--symlink-install` 增量生效，不必为每次改动重构、导出和上传 Docker 镜像；Docker
仍是里程碑发布、可追溯回归和同事交付的正式载体。

## 最短使用说明

首次拿到源码后，由 rt-control 负责人执行一次：

```bash
cd /home/ar/rt-control-dev/robot

git branch --show-current
git rev-parse --short HEAD
./tools/bootstrap_native_dev.sh prepare
./tools/bootstrap_native_dev.sh install-deps
./tools/bootstrap_native_dev.sh build
./tools/bootstrap_native_dev.sh doctor
```

日常只修改一个包时，不再制作镜像：

```bash
cd /home/ar/rt-control-dev/robot

./tools/bootstrap_native_dev.sh build \
  --packages-select <包名>
```

真实硬件普通启动不自动使能：

```bash
export RT_CONTROL_ROS_DOMAIN_ID=<本机器实例统一的 0..232>
./tools/rt_control_native.sh start
```

确认现场允许上电后，再单独使能：

```bash
./tools/rt_control_native.sh enable
```

急停或主接触器断电后，现场重新上电且确认允许执行器上电时，使用恢复一键脚本：

```bash
./tools/rt_control_native_oneclick.sh
```

它等价于 `./tools/rt_control_native.sh recover-power-loss`，会要求输入
`RECOVER_RT_CONTROL_NATIVE`，随后按固定顺序执行：尽力停止旧原生会话、等待 EtherCAT
回到 Idle/Inactive、启动新的原生控制栈、等待 enable-manager 进入 `IDLE` 或
`fault_requires_reset`、确认 controller update 线程已被 pin 到 CPU14、只调用一次
`/rt/reset_fault`、确认 14 个 EtherCAT 轴处于失能状态、在 `/rt/enable` 前再次确认
controller update 线程仍在 CPU14，最后确认控制器、14 轴和 EtherCAT 主站均已进入运行态。

联调结束必须有序停止：

```bash
./tools/rt_control_native.sh stop
```

如果明确需要一次完成启动和使能，可用
`./tools/rt_control_native.sh start-and-enable`，但它不会自动 fault reset，不适合作为
急停/主接触器掉电后的恢复入口。不要同时运行原生栈和 `robot-rt-control-1` Docker 容器；
包装器发现容器仍在运行时会直接拒绝启动。以上包装器会自行加载 ROS 和原生 overlay，
不需要先在终端 source rt-control 的 `install/setup.bash`。

## 1. 固定目录

```text
/home/ar/rt-control-dev/
├── robot/        # 本仓库 Git 工作树
├── src/vendor/   # deps.repos 锁定的四个上游仓库
├── build/        # colcon 构建输出
├── install/      # 原生运行 overlay
└── log/          # colcon 与运行日志
```

仓库内部不保存上游完整源码或构建产物。准备脚本不会对现有 vendor 仓库执行
`reset`、`checkout` 或 `clean`；发现依赖不完整、HEAD 不等于冻结 SHA，或补丁状态
无法解释时会停止并要求人工处理。

不带包选择参数的完整 `build` 会清理各包 CMake cache，构建全部运行包依赖闭包，并在
结束后逐包验证安装前缀及 `robot_interfaces_qos` 的五个命名 profile。带
`--packages-select` 的命令只用于明确范围的增量开发，不构成可部署运行闭包证据；公共接口、
QoS 或适配器依赖变化后必须再执行一次完整 `build`。

## 2. 首次准备

```bash
cd /home/ar/rt-control-dev/robot

./tools/bootstrap_native_dev.sh prepare
./tools/bootstrap_native_dev.sh install-deps
./tools/bootstrap_native_dev.sh build
./tools/bootstrap_native_dev.sh doctor
```

只有 `install-deps` 会通过 `rosdep` 请求 sudo 安装宿主依赖。`prepare` 导入
`deps.repos` 中的冻结版本并幂等应用已批准补丁；`build` 使用 merge install 和
symlink install，输出都在仓库外。

## 3. 日常增量编译

只改一个或少数包时：

```bash
cd /home/ar/rt-control-dev/robot

./tools/bootstrap_native_dev.sh build \
  --packages-select <本次修改的包>
```

修改公共接口、Robot Model、bringup 或上游补丁时，应扩大到直接消费者，必要时执行
不带包选择参数的完整 rt-control 构建。不要手工修改 `build/`、`install/` 或 vendor
工作树中的生成物。

## 4. ROS/DDS 约定

按 BQ-141，ROS Domain 是部署输入而不再固定为 0。Native 优先级是：

```text
--ros-domain-id N
RT_CONTROL_ROS_DOMAIN_ID
ROS_DOMAIN_ID
默认 0
```

所有值必须是十进制 `0..232`。推荐在联调终端显式设置一次：

```bash
export RT_CONTROL_ROS_DOMAIN_ID=12
./tools/rt_control_native.sh doctor
./tools/rt_control_native.sh start
```

单次命令可用 `./tools/rt_control_native.sh --ros-domain-id 12 <command>` 覆盖环境。
包装器对子进程继续显式设置：

```text
ROS_DOMAIN_ID=<已验证的部署值>
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

原生子进程不设置 `FASTRTPS_DEFAULT_PROFILES_FILE`、`FASTDDS_DEFAULT_PROFILES_FILE` 或
`CYCLONEDDS_URI`，因此 Fast DDS 保留默认 UDP 与共享内存传输。导航、雷达、感知和运控继续
source 各自的 ROS/workspace，但必须从同一部署清单取得与 RT-Control 相同的 Domain。

原生包装器只把这些变量传给自己的子进程，不会修改调用终端。同网段、同 Domain 的
ROS 2 participant 会互相发现；Domain 不是网络或安全隔离。现场网络必须受控，节点名、
topic 所有权和 QoS 仍需按公共契约管理。

## 5. 真实硬件启动

先做只读门禁：

```bash
cd /home/ar/rt-control-dev/robot
./tools/rt_control_native.sh doctor
./tools/rt_control_native.sh status
```

普通启动只启动控制栈，不自动复位故障，也不调用 `/rt/enable`：

```bash
./tools/rt_control_native.sh start
```

启动脚本的 CPU 时序是固定的：

1. 启动前要求 CPU14 包含在 `isolated` 和 `nohz_full` 集合内，并扫描非白名单
   `SCHED_FIFO/SCHED_RR` 线程；若发现它们实际运行在 CPU14，或 tight affinity 包含
   CPU14，则拒绝启动。
2. 外层 `rt_control_start` 运行在 housekeeping CPUs：
   `0,2,4,6,16-27`。这些 housekeeping CPUs 还必须与当前 `isolated`/`nohz_full`
   集合不重叠；当前多隔离核配置中的 CPU8/10/12 不归 native rt-control 普通线程使用。
3. 接触总线前，脚本验证公共接口、`robot_interfaces_qos` Python API、适配器、BMS、PLC、
   诊断与 bringup 均已安装；缺少任一运行依赖时 fail-closed。随后在最多 150 秒内要求
   `/rt/enable`、`/control/set_enabled` 可发现，并实际调用
   `/controller_manager/list_controllers`，避免旧 DDS discovery 造成假 READY。
4. 控制器必须达到 disabled 启动契约，EtherCAT 必须为 Operation/Active；两项均通过后才
   报告 READY。随后脚本查找 `ros2_control_node` 内部实时线程；只有“恰好一个
   `SCHED_FIFO` 且 `rt_priority=80`”的线程会被视为 controller update 线程。
5. 该线程被 `sched_setaffinity` 到 CPU14；同一进程内 DDS、service、CANopen/Lely 和普通回调线程保留在
   housekeeping CPUs。
6. 设置后必须验证 update 线程 affinity 和当前 `PSR` 均为 14，否则启动失败，且不会使能执行器。

Native 包装器不是持久化 daemon。目标机的 SSH session cgroup 会在连接断开后回收该会话
中的后台进程，即使进程使用了 `nohup`/`setsid`；远程联调必须保持启动终端连接。需要跨
会话托管时，应另行实现并评审受控 systemd service，不能把远程 shell 后台任务当作发布态。

在已运行的原生控制栈上显式使能：

```bash
./tools/rt_control_native.sh enable
```

每次通过原生脚本调用 `/rt/enable` 前都会重新执行上述线程级 pin 和验证。这样即使
`ros2_control_node` 崩溃重启、TID 变化，脚本也不会在未重新 pin 的情况下使能执行器。

需要一次完成启动和使能时：

```bash
./tools/rt_control_native.sh start-and-enable
```

主接触器断电、硬件急停或驱动器故障复位后的恢复启动：

```bash
./tools/rt_control_native_oneclick.sh
```

该脚本是傻瓜式入口，内部调用 `recover-power-loss`。它会：

1. 要求现场输入 `RECOVER_RT_CONTROL_NATIVE`。
2. 如果旧原生会话仍存在，先尽力调用 `/rt/disable`，再请求 `rt_control_start` 有序退出。
3. 等待 EtherCAT 主站确认 `Idle/Inactive`，避免上一轮控制栈残留。
4. 启动新的原生控制栈。
5. 等待控制栈暴露 `/rt/enable`，并完成 controller update 线程 CPU14 pin 门禁。
6. 等待 enable-manager 进入 `IDLE` 或明确的 `fault_requires_reset`。
7. 调用一次全组 `/rt/reset_fault`。
8. 检查 14 个 EtherCAT 轴处于失能状态。
9. 在 `/rt/enable` 前再次执行并验证 controller update 线程 CPU14 pin。
10. 调用一次 `/rt/enable`。
11. 检查 JTC、JSB、diff-drive、enable-manager、14 轴状态和 EtherCAT OP 状态。

如果任一步失败，脚本会停止本次恢复会话并报错；它不会发送 FJT、`/cmd_vel_safe` 或 PLC 输出。

与 Docker 路径（[one-command-start.md](one-command-start.md)）相比，原生序列多出第 5、9 两步
controller update 线程 CPU14 pin 门禁——这是原生运行的有意差异，不是文档漂移。

以上命令都保留真实硬件确认口令。原生脚本锁定当前工控机、实时内核、CPU14、
EtherCAT MAC，以及 ZLG PCIe-9140I 的 `10b5:9140` PCI 身份、`zpcican` 驱动和端口
`dev_id`；L0 固定为 CANopen `can0`、L1 固定为 BMS `can1`，两路均设置为
500 kbit/s、txqueuelen 128。硬件身份、端口数量或保留接口名任一不符即拒绝启动。
它也会拒绝与正在运行的 `robot-rt-control-1` 容器重叠。IgH EtherCAT 主站必须通过
`/etc/modprobe.d/ec_master.conf` 配置为 `options ec_master run_on_cpu=14`；该模块参数需要
重启或重载 `ec_master` 后才对 `EtherCAT-OP` 线程生效。

查看状态和日志：

```bash
./tools/rt_control_native.sh status
./tools/rt_control_native.sh logs
```

有序停止：

```bash
./tools/rt_control_native.sh stop
```

停止顺序为调用 `/rt/disable`，再向安装后的 `rt_control_start` 发送 TERM，并等待其完成
controller、EtherCAT 和 CANopen 有序退出。脚本不会用 SIGKILL 掩盖超时。

## 6. Docker 发布边界

原生方式适合高频开发，但会直接依赖宿主 ROS/apt/IgH 环境，复现性和回滚隔离弱于
发布镜像。准备联调里程碑或交付同事前仍应执行仓库门禁、干净镜像构建、Mock、
GitHub Release 归档发布和分阶段实机验证；不得把一次原生运行结果直接声明为发布镜像验收。

截至 2026-07-31，目标工控机 `/home/ar/rt-control-dev/robot` 已切到
`feature/rt-control-native-development` 的 `a022b1052e9b76480e8e37e4dac8913a81eba392`：
`prepare` 已幂等导入并验证冻结 vendor/补丁，`install-deps` 报告 rosdep 依赖满足，
完整 `build` 完成 24 个包，`doctor` 通过。宿主 IgH identity 已补齐到
`/usr/local/share/rt-control/dependency-versions.env`，与 `versions.env` 的
`stable-1.6 / 2f7f884f1c7d377c02a7d627eb06512126a0e50e` 一致。

后续 CPU14 线程级 pin 门禁和 IgH `run_on_cpu=14` 变更需要在目标机重新同步、构建、重启或重载
IgH 后再做上电验证；已部署 Docker 版本和 Compose 配置不由原生开发文档修改。
