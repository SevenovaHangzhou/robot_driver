---
id: canopen-chassis-20260917-01
area: canopen-chassis
title: LPMS-NAV3 CAN 外置 IMU 重新迁入 main 源码基线
date: 2026-09-17
type: feature
trigger: ELECTRI-105，用户要求基于当前 main 重新放入独立驱动源码
commits: [feat/electri-105-lpms-nav3-main]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [ELECTRI-105, ELECTRI-120, ELECTRI-122, BQ-150]
---

## 背景

ELECTRI-105 的源码此前存在无 Git 历史的独立 staging，旧机于 2026-08-28 在独立
CANable 接口完成静止测试，没有形成 robot_driver 功能分支。本次从最新
`origin/main@74afc8dcfd3e2c62291ef8d22452458e9a834f2c` 创建
`feat/electri-105-lpms-nav3-main` 重新集成。主责是 CANopen 传感器协议适配，因此记录归属
canopen-chassis，而不是导航融合或宿主接口配置。

## 改动

- 新增 `src/rt_control/lpms_nav3_can`，保留默认 16-bit 四 TPDO、零填充 heartbeat、SI 换算、
  四帧组装、超时诊断、接收 socket 重连；NOTICE 记录原始源码 SHA-256。
- 删除 NMT Start 编码/发送路径，不保留 SDO、bitrate、接口-up、systemd/udev 或目标机绝对路径。
  节点是该 IMU 的专用传感器适配，不是履带/舵轮 CANopen 主站的旁路监视器。
- interface/frame 默认为空，Node ID 默认 0；不完整配置拒启动，can0/can1 没有共享接口 override。
  launch 默认 `validation_only=true`，不创建节点；生产 rt_control launch 与 alfa_v3 manifest 不变。
- 默认输出私有 `~/data`、`~/mag`、`~/diagnostics`，使用命名 fast_state/diagnostic QoS。
  不把旧 `/imu/data` 和 SensorDataQoS 自动变成新的公共契约，不发布 TF 或启动导航融合。
- native/CI 构建测试列表与 bringup 安装依赖纳入新包；新增接入合同和无硬件节点/launch 测试。
- 节点拆成可测试 library 与薄 main：同进程 intra-process 验证诊断，进程级验证参数拒绝、
  缺接口日志、SIGINT 退出和静态 launch。

## 验证

- 接入合同 TDD：目标包未迁入时 3 failed；迁入后 `pytest tools/tests/test_lpms_nav3_integration.py`
  为 3 passed。
- Native `colcon build/test --packages-select lpms_nav3_can`（`ENABLE_COVERAGE=ON`）：5 个测试
  目标、30 条汇总记录（16 个 GTest + 9 个 pytest + 5 个目标），0 errors/failures/skipped。
- 首轮 Python executor context 错配已修复；之后本机跨进程 DDS 没有发现节点，localhost 隔离
  也未成功。诊断验证改为同进程 intra-process；不宣称跨进程 DDS 发布已通过。
- 纯协议 ASan+UBSan：13 tests 通过。gcov 行覆盖：decoder 97.83%、ROS conversion 100%、
  socket protocol 100%；node 47.58%，CAN 连接成功/接收/断流/热插拔路径缺口保留。
- 受影响 machine-profile/launch/validator 与接入合同：67 passed；Python compile、bash syntax、
  `git diff --check` 通过。
- `tools/quality_gate.sh`：285 passed、13 skipped，策略覆盖率 83%；本机没有 ShellCheck，
  保留 CI 检查。未执行受影响闭包的远端 CI，不能用本地结果宣称远端通过。
- 未启动真实或虚拟 CAN、未写设备、未 reset/enable/运动；未执行 Docker、目标部署或当前
  基线实机验收。2026-08-28 的独立 can2/500 kbit/s/Node 1 结果仅为历史依据。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: ELECTRI-105 已有当前 main 基线的版本化源码路径；本 PR 只集成源码，不进入生产启动。
- F2: 新驱动只接收精确匹配的标准 CAN 数据帧，不拥有 NMT/SDO、接口配置或控制环生命周期。
- F3: 默认 draft 静态 launch 不启动 CAN 节点，三代机共线拓扑和跨域 IMU 接口未因此获准。
- F4: 时间戳为组装完成后的 ROS 发布时间，非设备采样时间；四帧基于宿主到达组装，不证明
  同一设备样本。坐标/外参、四元数有效性与协方差标定仍是正式融合前置条件。

## 遗留

与 ELECTRI-120/122 确认三代机最终器件、共线完整仲裁 ID/波特率、唯一接口配置所有者、
Node ID、PDO/heartbeat、拥塞/丢帧/重启策略。由 Robot Model owner 冻结安装 TF，并完成
动态正负轴、磁场和协方差标定。随后在隔离台架完成真实 CAN→ROS 跨进程发布、四帧同步质量、
断流/热插拔及整机生命周期；当前节点 CAN 连接路径的覆盖率不能从纯解码覆盖率推导。
