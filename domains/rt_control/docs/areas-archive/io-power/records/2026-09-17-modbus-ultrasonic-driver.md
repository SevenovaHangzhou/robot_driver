---
id: io-power-20260917-01
area: io-power
title: 通用 Modbus RTU485 包与四路超声波驱动
date: 2026-09-17
type: feature
trigger: "GitHub #37；现场用户要求保留 LED 配置，将包重命名并接入已连通的 E08 四路 A22"
commits: ["feature/rt-control-modbus-ultrasonic-driver"]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: ["#37", "io-power-20260916-01"]
---

## 背景

现场链路为工控机经 RS485-ETH-M04 HC2 连接 DYP-E084F-V2.0，再连接四个 A22。
用户已用 `mbpoll` 证明 `192.168.1.12:504`、unit 1、FC03、寄存器
`0x0106..0x0109` 可读，要求在保留未提交 LED 现场配置的前提下，将 LED 专用包改为
通用 RTU485 包并增加超声波信息采集。

## 改动

包由 `modbus_tcp_rtu485_led` 重命名为 `modbus_tcp_rtu485`，保留
`led_strip_node`；用户已有 `led_strip.yaml` 的端口 `[502,502,502,502]`、地址
`[1,2,3,4]` 原样迁移。共享传输层增加 FC03 可变长度响应、异常帧和寄存器解析。

新增 `ultrasonic_node` 和 `ultrasonic.yaml`，默认读取
`192.168.1.12:504`、unit 1、`0x0106..0x0109`，周期 300 ms、事务截止 500 ms。
节点发布四路仅含 `float32 range` 的 `rt_control_interfaces/UltrasonicRange`、一份原始 `UInt16MultiArray` 和
`DiagnosticArray`；解析 `FFFD/FFFE/FFFF/EEEE`。通信失败只发布错误诊断，不把陈旧
数据伪装成当前测量。bringup 增加 `start_ultrasonic` 与 `ultrasonic_config`，Mock 默认
不启动任何 Modbus 硬件节点。

## 验证

环境：ROS 2 Humble；构建、安装、日志均位于 `/tmp/modbus-rtu485-*`。

- 功能包隔离构建：PASS，`modbus_tcp_rtu485` 两个 C++ 节点编译通过。
- 使用 `/home/ar/rt-control-dev/install` 的目标机既有上游依赖构建
  `modbus_tcp_rtu485` 与 `rt_control_bringup`：PASS，2 包。
- 本地 socketpair 协议测试：PASS；覆盖 FC16、FC03、实际四路响应形状、分片、异常、非法头、断连、恢复、截止时间及超声波状态码。沙箱禁止 socket IO，按既有模式在沙箱外运行同一测试二进制。
- 驱动包测试：PASS，2 tests / 0 errors / 0 failures / 0 skipped。
- `test_preop_snapshot_launch.py`：PASS，8 项；LED 与超声波在 Mock 下默认关闭、真实配置下默认开启。
- 自定义测距接口：PASS；`ros2 interface show rt_control_interfaces/msg/UltrasonicRange`
  仅输出 `float32 range`。
- bringup 非实机 pytest（排除完整 Mock 合同）：PASS，147 项。
- T2 只读实机：PASS。启动新 `ultrasonic_node` 后，`/ultrasonic/raw` 收到
  `[21,56,20,17]`；第一路 `Range` 收到约 `0.023 m`；一次诊断采样四路均为 `OK`；
  SIGINT 正常退出。不同话题样本来自不同轮询周期，数值不要求逐帧相同。
- `tools/quality_gate.sh`：PASS，286 passed / 13 skipped，门禁覆盖率 84%；本机缺
  `shellcheck`，由 CI 强制检查。
- 首次从空临时前缀构建完整 `rt_control_bringup` 闭包：BLOCKED，当前源码工作区未导入
  `canopen_ros2_control`；改用目标机既有已安装上游依赖后两包构建通过。组合
  `colcon test` 的 bringup 编排仍要求全部工作区依赖出现在同一临时 install，故使用完整
  bringup pytest 验证源码。未运行完整整机 Mock 合同测试。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `modbus_tcp_rtu485` 同包保留独立 LED 写节点并新增 E08 只读节点；同步网络 IO 均在普通 ROS 回调中，不进入 ros2_control 实时环。
- F2: E08 现场端点是 `192.168.1.12:504`、unit 1，FC03 连续读取 `0x0106..0x0109` 已完成 T2 只读通信和 ROS 发布验证。
- F3: `ultrasonic_1..4` 仅是接口标识；物理前后左右关系、安装位姿和 TF 未确认，不得由通道号推导方向。
- F4: `0xFFFD/0xFFFE/0xFFFF/0xEEEE` 分别归一化为无目标、同频干扰、探头超时和校验错误；原始值同时保留供诊断。

## 遗留

需冻结四个 A22 的物理位置、frame 名称和 TF 后再接入面向其他域的稳定接口。尚未执行
长时间轮询、网络断连恢复实机测试、整机 Mock、Docker 或发布封装。LED 的实机写入与
退出保持行为仍沿用上一记录的待验结论。
