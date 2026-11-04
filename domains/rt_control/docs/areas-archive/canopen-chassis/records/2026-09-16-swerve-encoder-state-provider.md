---
id: canopen-chassis-20260916-01
area: canopen-chassis
title: 三代机四路 CANopen 舵角 state-only provider
date: 2026-09-16
type: feature
trigger: ELECTRI-132
commits: [work/electri-117-swerve-module-binding]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [ELECTRI-117, ELECTRI-132, BQ-144, BQ-145, BQ-148]
---

## 背景

三代机 swerve controller 已冻结四路外置舵角 `position/feedback_age_ms`
输入合同，但 `robot_hw_canopen` 原来只拥有两路履带 Cia402 actuator，
没有 state-only 编码器 provider。旧独立驱动的阻塞 SocketCAN 循环不能进入
共享 ros2_control 实时路径。

## 改动

- 新增 `SwerveEncoderFeedback` 固定四槽核心。每个节点只接受
  `0x6004:00`，按每转计数、齿圈/小齿轮齿数、方向和安装偏移换算输出轴角。
  RPDO 回调写入原始值与单调接收时刻；控制线程使用固定三次 seqlock 快照，
  不加锁、不等待、不分配。
- 新增 `SwerveEncoderSystem` hardware plugin，继承冻结 ros2_canopen 的
  `CanopenSystem`，复用同一个 Lely DeviceContainer/master 生命周期。
  插件恰好接受四个 sensor，仅导出每路 `position` 和
  `feedback_age_ms`，不导出任何 command interface。
- 新增 ros2_canopen `0006-expose-rpdo-receive-hook.patch`。补丁只增加
  protected RPDO hook 与 `steady_clock::now()` 接收时刻；默认 generic
  RPDO 存储行为不变。native patch/verify 链和后续人工 Docker 封装链均按
  0001..0006 顺序应用。
- 新增 `alfa_v3_swerve_encoders.draft.yaml` 及构建期 validator。
  固定 4 ms master SYNC、ProxyDriver、`0x6004:00/32-bit unsigned`、
  四个 FL/FR/RL/RR 逻辑资源名和状态接口；实际 CAN interface、master/node ID、
  EDS、0x6501/0x6502、齿数、方向与零偏全部保持 `TBD`。
- alfa_v3 machine manifest 引用该 unverified profile；controller draft 使用
  四个逻辑资源名，但 `calibration_verified=false`，runtime gate 不开放。
  原 alfa_v1 履带 descriptor/bus 和生产启动不变。

## 验证

- 核心按 RED/GREEN 实现：空实现首先产生未定义符号链接失败；实现后 5 个 gtest
  覆盖配置拒绝、节点/对象路由、轴角换算、真实年龄、静止值刷新及初始/未来时间无效。
- provider RED 首先准确失败于未实现构造、配置、接口导出和 RPDO hook；
  实现后 4 个 gtest 通过，覆盖只读接口、错误 HardwareInfo 拒绝、
  RPDO 更新与错误对象隔离。
- 从冻结 `ros2_canopen@fef50e54b1c94c50e908e2c5d0b8888eed907e8d`
  新克隆依次 apply-check/apply 0001..0006，`--packages-up-to
  canopen_ros2_control` 构建 7 个包通过。
- `robot_hw_canopen` 隔离构建通过，CTest 汇总 64 tests、
  0 errors、0 failures、0 skipped；安装空间存在 pluginlib 索引和 draft profile。
- CANopen 与 machine-profile 配置 pytest：112 passed。
- 纯核心 ASan+UBSan 5 tests 通过；发布用例目录 33 cases valid。
- 补丁顺序/回调 seam 与 Docker 人工封装规则定向测试：5 passed。
- `tools/quality_gate.sh`：282 passed、13 skipped，策略覆盖率 83%；
  本机缺少 ShellCheck，保留 CI 检查。
- `git apply --check` 在持久冻结 0001–0005 工作树上确认 0006 可应用，
  `bash -n tools/bootstrap_native_dev.sh` 与 `git diff --check` 通过。

以上均未打开真实 CAN、未加载实际 EDS、未启动 hardware lifecycle。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 外置舵角 provider 复用 ros2_canopen/Lely master，不新建旁路 SocketCAN
  控制循环；4 ms SYNC 由 master `sync_period` 负责。
- F2: 每路只导出输出轴 `position` 和真实 `feedback_age_ms`，静止值重复
  TPDO 也刷新接收年龄；当前起点是 Lely RPDO 回调的宿主单调时间，不是内核 CAN
  硬时间戳。首次数据前导出 NaN/+inf，控制器按无效反馈停车。
- F3: provider 没有 command interface、自动 rezero、局部 NMT 恢复或
  电机编码器续跑逻辑，符合 BQ-144/BQ-145。
- F4: 当前只完成软件骨架和 draft 模块接入；硬件字段和标定未确认，
  `chassis_only --require-runtime-ready` 必须继续拒绝。

## 遗留

需要实际 BRT 型号/EDS、CAN interface、master/node ID、0x6004/0x6501/0x6502
读回、TPDO 映射、精确反馈齿数、方向和安装零偏。随后生成独立 authority
bus/DCF，测量 CAN 接收到 Lely 回调的额外延迟，并验证 4 ms SYNC 负载、
每路数据年龄、心跳、断流、跳变、多圈保持和
有序停机，并与 ELECTRI-127 的 Kinco 8 轴完成 chassis_only 联合验证。
