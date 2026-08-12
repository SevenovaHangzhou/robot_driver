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
├── src/vendor/   # deps.repos 锁定的三个上游仓库
├── build/        # colcon 构建输出
├── install/      # 原生运行 overlay
└── log/          # colcon 与运行日志
```

仓库内部不保存上游完整源码或构建产物。准备脚本不会对现有 vendor 仓库执行
`reset`、`checkout` 或 `clean`；发现依赖不完整、HEAD 不等于冻结 SHA，或补丁状态
无法解释时会停止并要求人工处理。

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

原生脚本显式使用：

```text
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

原生子进程不设置 `FASTRTPS_DEFAULT_PROFILES_FILE`、`FASTDDS_DEFAULT_PROFILES_FILE` 或
`CYCLONEDDS_URI`，因此 Fast DDS 保留默认 UDP 与共享内存传输。导航、雷达和感知继续
source 各自的 ROS/workspace；它们默认也在 Domain 0 时无需 source rt-control overlay
或额外 DDS 脚本。

原生包装器只把这些变量传给自己的子进程，不会修改调用终端。Domain 0 会让同网段、
同 Domain 的 ROS 2 participant 互相发现；现场网络必须受控，节点名、topic 所有权和
QoS 仍需按公共契约管理。

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

1. 启动前扫描非白名单 `SCHED_FIFO/SCHED_RR` 线程；若发现它们实际运行在 CPU14，或 tight
   affinity 包含 CPU14，则拒绝启动。
2. 外层 `rt_control_start` 运行在 housekeeping CPUs：
   `0,2,4,6,8,10,12,16-27`。
3. 控制栈同时暴露 `/rt/enable` 和 `/control/set_enabled` 服务后，脚本查找 `ros2_control_node` 内部实时线程；只有“恰好一个
   `SCHED_FIFO` 且 `rt_priority=80`”的线程会被视为 controller update 线程。
4. 该线程被 `sched_setaffinity` 到 CPU14；同一进程内 DDS、service、CANopen/Lely 和普通回调线程保留在
   housekeeping CPUs。
5. 设置后必须验证 update 线程 affinity 和当前 `PSR` 均为 14，否则启动失败，且不会使能执行器。

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

以上命令都保留真实硬件确认口令。原生脚本锁定当前工控机、实时内核、CPU14、
EtherCAT MAC、两只 CANable 序列号和 500 kbit/s 参数；任一事实不符即拒绝启动。
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
