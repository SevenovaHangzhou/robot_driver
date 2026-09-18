---
id: io-power-20260919-01
area: io-power
title: 四路超声波切换为标准 Range 接口
date: 2026-09-19
type: feature
trigger: "用户确认 A22 使用 3.5 m 量程、60 度角度、E084F 同时测量，并要求 PR #38 发布标准 Range"
commits: ["feature/rt-control-modbus-ultrasonic-driver"]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: ["#37", "io-power-20260917-01"]
---

## 背景

PR #38 首版使用仅含 `float32 range` 的 RT-Control 私有消息，不能携带测量时间、
传感器坐标系、辐射类型、视场角和量程边界。用户确认四个 A22 按 3.5 m、60 度工作，
E084F 采用四路同时测量，且 `0xFFFD` 无目标按正常应用状态处理。

## 改动

- 删除私有 `rt_control_interfaces/msg/UltrasonicRange`，四个既有话题改为标准
  `sensor_msgs/msg/Range`，并移除驱动对私有接口包的依赖。
- 每个消息填充批次完成时间、通道 frame、`ULTRASOUND`、`1.0471975512 rad`、
  `0.01 m`、`3.5 m` 和测距值。一次 FC03 连续读取 `0x0106..0x0109`，四条消息共享
  同一个响应完成时间。
- 增加四个 `frame_ids` 参数；默认名称仅标识 E08 通道，等待装车外参测量后替换并接入
  `base_link` TF。
- 将 `0xFFFD` 映射为 `+Inf` 与 `OK/no_target`；干扰、探头超时、校验错误继续通过
  NaN 和 WARN/ERROR 诊断区分。
- 参数校验固定 3.5 m 和 60 度，并检查四个 frame 非空、相对且唯一。
- 按用户要求将 PR #38 的三个原始提交和标准 Range 改动 rebase 到
  `v3@f5b639fbb927a44d3e2e6657492adcea44c71ced`。冲突处理保留 V3 的
  `JointControlModeResult`、V3 依赖和全部已有开发记录。
- V3 不安装旧 `rt_control.launch.py`，因此新增可安装的独立
  `rt_control_ultrasonic.launch.py`，只启动采集节点；Mock 模式不创建硬件进程，
  不改双臂运行入口。CI、原生构建工具和 Dockerfile 构建包选择同步到新包名。

## 验证

- TDD RED：新增标准 Range 字段测试后，构建按预期因缺少 `ultrasonic_range.hpp` 失败。
- 隔离构建 `rt_control_interfaces`、`modbus_tcp_rtu485`：PASS，2 packages。
- 隔离 `colcon test`：PASS，2 tests / 0 errors / 0 failures / 0 skipped；覆盖完整 Range
  元数据、正常距离、`0xFFFD`、协议错误码、FC03 四寄存器请求、超时和节点生命周期。
- 单独重跑 `node_lifecycle`：PASS；3.5 m、60 度约束与非法端点均能在启动时拒绝。
- `test_rt_io_integration.py`：PASS，18 passed；私有接口资产清单已移除旧测距消息。
- `tools/quality_gate.sh`：PASS，286 passed / 13 skipped，门禁覆盖率 84%；本机缺
  `shellcheck`，由 CI 强制检查。

V3 rebase 后重新验证：

- 隔离构建 `modbus_tcp_rtu485`、`rt_control_bringup`：PASS，2 packages。
- 驱动 `colcon test`：PASS，2 tests；组合命令在进入 bringup 时因隔离前缀缺少机械臂
  runtime 依赖标记中止，不记为组合测试通过。
- 对已构建 bringup 直接执行 CTest：PASS，6/6 suites，包括原有 V3 机械臂、原位使能、
  machine profile 和新增超声波入口测试。
- 独立入口与 V3 launch 配置 pytest：PASS，16 passed。
- 安装后执行独立 launch 的 `use_mock_hardware:=true`：PASS，未启动硬件进程。
- V3 `tools/quality_gate.sh`：PASS，292 passed / 13 skipped，门禁覆盖率 83%；
  Dockerfile 仅调整改名包的构建选择，未构建镜像或执行发布部署。

尚未在真实 E084F/A22 上复测标准 Range 话题、时间戳、frame 或长期运行，未测量四个
探头相对 `base_link` 的外参。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 四路测距话题的类型为 `sensor_msgs/msg/Range`，每批数据携带同一响应完成时间以及
  各通道 frame、超声类型、60 度视场角、0.01 m 最小量程和 3.5 m 最大量程。
- F2: A22 原始值 `0xFFFD` 表示量程内无目标，发布 `+Inf`，诊断为 `OK/no_target`。
- F3: 默认 `ultrasonic_channel_1_link..4_link` 只代表 E08 通道，不代表前后左右；装车后
  必须用实测外参替换并提供到 `base_link` 的 TF。
- F4: E084F 通过一次 FC03 连续读取四个通道，按已确认配置触发四路同时测量；当前代码
  不实现四路逐个发射的轮询调度。
- F5: V3 超声波由 `ros2 launch rt_control_bringup rt_control_ultrasonic.launch.py`
  独立启动；双臂 runtime 不隐式启动该采集节点。

## 遗留

需要在台架或装车环境复测标准 Range 发布，冻结四个完整 A22 型号、通道方向与外参，
并完成长期通信、断线恢复、场景覆盖和低速减速/停车联动验证。
