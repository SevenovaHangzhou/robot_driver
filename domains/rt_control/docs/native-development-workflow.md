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

联调结束必须有序停止：

```bash
./tools/rt_control_native.sh stop
```

如果明确需要一次完成启动和使能，可用
`./tools/rt_control_native.sh start-and-enable`。不要同时运行原生栈和
`robot-rt-control-1` Docker 容器；包装器发现容器仍在运行时会直接拒绝启动。以上包装器
会自行加载 ROS 和原生 overlay，不需要先在终端 source rt-control 的 `install/setup.bash`。

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

在已运行的原生控制栈上显式使能：

```bash
./tools/rt_control_native.sh enable
```

需要一次完成启动和使能时：

```bash
./tools/rt_control_native.sh start-and-enable
```

以上命令都保留真实硬件确认口令。原生脚本锁定当前工控机、实时内核、CPU14、
EtherCAT MAC、两只 CANable 序列号和 500 kbit/s 参数；任一事实不符即拒绝启动。
它也会拒绝与正在运行的 `robot-rt-control-1` 容器重叠。

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
锁定镜像。准备联调里程碑或交付同事前仍应执行仓库门禁、干净镜像构建、Mock、发布锁
更新和分阶段实机验证；不得把一次原生运行结果直接声明为发布镜像验收。

当前交付只把非 Docker 原生源码放到目标机；尚未安装原生依赖、导入 vendor、构建、
启动或使能原生栈。已部署 Docker 版本和 Compose 配置不由本交付修改。
