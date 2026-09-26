---
id: release-deploy-20260924-01
area: release-deploy
title: 退役旧履带启动与 CANopen 驱动链
date: 2026-09-24
type: corrective
trigger: 用户要求从 ELECTRI-127 工作树完整退役旧履带控制链
commits: []
env: both
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [release-deploy-20260909-01, release-deploy-20260913-01]
---

## 背景

V3 已不安装旧 `rt_control.launch.py` 和旧控制器配置，但源树还保留 diff-drive
履带启动、Leadshine CANopen bus/EDS/variant 生成链、旧机操作脚本及相应测试。
这些资产不应被误认为 V3 舵轮运行入口。

## 改动

删除旧履带 launch、控制器配置、CANopen 履带 bus/EDS/variant、生成规则、旧原生/IPC
包装器及仅针对它们的测试；宿主 `can0.service`、旧履带 USB 序列号固定命名与
`0x702/0x703` 心跳验收也移除，BMS `can1` 路径保留。从共享测试中移除
已退役入口的断言。保留
`robot_hw_canopen/SwerveEncoderSystem`、舵轮编码器 draft 校验、V3 模块启动、
共享 Robot Model 和公共接口。冻结 `ros2_canopen` 补丁由原 0001..0006
重整为 `0001-shared-canopen-lifecycle.patch`、独立的 0002 Lely 队列补丁和
`0006-expose-rpdo-receive-hook.patch`；
新补丁只保留主站/代理/基类回调退出、释放顺序、RPDO 接收时间及线程命名，
不再修改 CiA402 电机或按节点 2/3 执行 EMCY 整组停车。Docker 与 Native
补丁顺序同步更新。旧现场文档保留为历史证据并标注不可用。

## 验证

- `tools/quality_gate.sh`：PASS，207 项工具测试通过、12 项跳过；ShellCheck 本机缺失。
- 使用仓库外临时安装目录构建 `robot_description`：PASS；source 该安装目录后执行
  `python3 -m pytest src/rt_control/rt_control_bringup/test src/rt_control/robot_hw_canopen/test src/rt_control/robot_hw_ethercat/test/test_x503_topology.py -q`：98 项通过。
- 在仓库外使用冻结 `ros2_canopen@fef50e54b1c94c50e908e2c5d0b8888eed907e8d`
  的全新工作树，顺序对新 0001、0002、0006 执行 `git apply --check` 并应用：PASS。
  两份共享补丁的累计源码 diff 与已通过 150 项测试的隔离候选字节一致。
  `colcon build --packages-up-to robot_hw_canopen`：8 包通过；另构建
  `canopen_master_driver`：PASS。`colcon test --packages-select canopen_core
  canopen_master_driver canopen_proxy_driver canopen_ros2_control robot_hw_canopen`：
  150 项通过、0 错误/失败/跳过。
- `docker build --build-arg IGH_VERSION=stable-1.6 --build-arg IGH_COMMIT=2f7f884f1c7d377c02a7d627eb06512126a0e50e -t rt-control:electri127-retire-local -f docker/rt-control/Dockerfile .`：PASS，39 包构建通过，测试镜像 ID 为
  `sha256:94e24dd2bf1f7d99351bfc6b9e739ed8eb16564abc361efceca9ecd65cceef`。
  `docker run --rm --network none`（不映射设备）执行 `rt_control_module.launch.py`
  的 `arms_only` 和 `chassis_only` `validation_only:=true`：PASS，前者 CANopen
  `not_required`，后者 swerve controller/4 encoder 仅静态 draft，Node ID 仍 TBD；
  镜像中旧履带 launch/config/bus/variant 均不存在，舵轮编码器插件与 draft 存在。
- `git diff --check`：PASS。V3 machine-profile 与舵轮编码器 draft 校验：PASS
  （8 个 profile/scope/option 组合、4 个编码器节点）。
- `robot_hw_canopen` 在未导入冻结依赖的默认 shell 中配置失败（缺
  `canopen_ros2_controlConfig.cmake`）；新补丁链的隔离构建/测试通过，不将此
  环境问题误记为代码编译失败。
- `bash -n hostsetup/can-install.sh hostsetup/verify-host.sh hostsetup/rt-control-can-names.sh`：PASS；
  `python3 -m pytest tools/tests/test_rt_io_integration.py -q`：16 项通过。
- `systemd-analyze verify` 对仓库外两份同内容 unit 副本（仅将 naming service 的
  ExecStart 指向本工作树可执行脚本）返回 0；原 unit 直接校验在本机因
  `/usr/local/sbin/rt-control-can-names` 未安装而失败。未执行宿主安装，
  目标机 unit 与 BMS 连接仍未验证。
- `tools/quality_gate.sh` 的 CANopen 门禁确认共享补丁保留回调退出先于驱动释放、
  RPDO 时间钩子存在，且没有旧 CiA402/履带节点修改。
- 未执行 V3 真实 runtime、设备映射或任何实机/总线动作。静态 validation 容器不等于
  ros2_control Mock 生命周期，更不能声称停车或运动性能验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 旧履带控制源码、bus 生成链、宿主 `can0` 服务和旧机包装器已从当前工作树移除；`ros2_canopen` 的旧履带 CiA402/EMCY 补丁已剥离，V3 舵轮编码器包仍保持 draft、不可直接用于实机运行。
- F2: 历史文档和旧发布物的运行记录不代表本工作树可用入口；部署需要独立重建、锁定和验证。

## 遗留

原 0001/0005 含共享生命周期修复且 0003/0006 依赖其上下文，因此本次用新
共享补丁替换旧链并在冻结上游离线构建/测试通过。真实 CANopen 设备失联、回调退出竞态、系统服务安装及 V3 控制器 Mock 生命周期
仍未验证；本地 Docker 静态 validation 已通过，但测试镜像不是经 source SHA
和现场条件锁定的部署候选。用户确认旧 `can0` 为履带专属，已移除其宿主配置；V3 舵轮编码器新
CAN 身份仍为 TBD。BMS `can1` 保留已有 USB 身份配置，实际目标硬件身份和
系统服务验收不在本次静态清理范围。未完成适用发布验证之前不得部署。
