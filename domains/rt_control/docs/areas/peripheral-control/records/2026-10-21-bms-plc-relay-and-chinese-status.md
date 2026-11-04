---
id: peripheral-control-20261021-01
area: peripheral-control
title: 双电池PLC继电器服务与中文状态故障
date: 2026-10-21
type: feature
trigger: 用户要求双电池只通过PLC继电器开关，不控制BMS MOS，并发布中文状态与故障
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [/tmp/bms-plc-analysis-build, /tmp/bms-plc-analysis-test-log]
supersedes: [io-power-20261009-01]
related: [io-power-20261006-01, BQ-152]
---

## 背景

旧实现包含 D9 放电 MOS 与自动双电池并入状态机。用户本次明确要求取消 MOS 控制，
主、副电池只通过同一数字量 PLC 模块上的两路不同继电器地址开闭，并用话题输出两块
电池的中文状态和中文故障。

## 改动

- `bms_node` 删除 D9 收发及自动并入状态机；CAN 路径只查询
  `0x90/0x91/0x92/0x93/0x94/0x95/0x98`。
- 电池通信接口切换为专用 USB-CAN 的 SocketCAN `can1`，波特率 250 kbit/s，并以 USB
  序列号 `004D00675230500720333159` 防止绑定到错误适配器。
- 开关不经过 `bms_node` 转发，调用方直接调用 `/plc/battery/primary`、
  `/plc/battery/secondary` 的 `rt_control_interfaces/SetDigitalOutput`；PLC 明确确认并回读
  目标线圈状态后才报告成功。
- `plc_io_modbus.digital` 新增两路电池继电器 DO 配置，共用既有
  `digital.module` host/port/unit_id；两地址必须不同，也不得与已启用的真空输出冲突。
- 电池继电器参数归属 `plc_io_modbus/config/plc_io_modbus.yaml`，PLC 节点直接加载自己的
  配置文件；`bms_node.yaml` 仅包含 BMS 采集参数。
- 两块电池分别保留 `BatteryState`，并新增中文状态和故障话题。中文状态覆盖电量、
  电压、电流、温度、充放电和 MOS 状态；中文故障覆盖 `0x98` 全部定义位及故障码。
- MOS 开闭原始值未由协议定义，YAML 默认 `-1/-1`，只显示原始值而不猜测开闭。
- `0x95` 收到帧序号 0 时开启新扫描，避免把不同轮次的单体电压拼成完整扫描。
- SocketCAN 不再安装 14 条内核精确过滤器；专用 `can1` 的帧由既有用户态白名单校验
  扩展帧标志、RTR/ERR、DLC、数据 ID、主机地址和电池地址。现场原内核过滤路径可发送
  查询但没有向解码器交付响应，改为用户态过滤后恢复主、副电池实时状态。

## 验证

- 隔离构建 `bms_node plc_io_modbus`：通过。
- 修复后 `bms_node` 构建通过；29 tests，0 errors，0 failures，0 skipped。
- `python3 -m pytest -q tools/tests/test_rt_io_integration.py`：16 passed。
- 验证 `bms_node` 不再包含 PLC 客户端或 `SetBool` 服务。
- 实机 `can1` 为 250 kbit/s、ERROR-ACTIVE，CAN 错误计数为 0；捕获到地址 `0x01/0x02`
  的 `0x90..0x98` 响应。修复版中文话题分别解码出主电池 49.5 V/90.8% 和副电池
  49.9 V/65.2%，并分别报告“总压过高一级告警”和“无故障”。
- 未写 PLC 继电器，未执行实机开闭。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: BMS CAN 路径只读，不发送 D9/DA；调用方直接使用 PLC 继电器服务，不经过 BMS。
- F2: 两路电池继电器共用 digital 模块通信端点，但 DO 地址必须不同；当前配置为主电池
  DO `0`、副电池 DO `1`，`battery_relays.configured=true`。
- F3: PLC 写只有在请求被接受、结果为 CONFIRMED、回读有效且等于目标值时才向调用方
  报告成功。
- F4: 两块电池分别发布标准数值状态、中文状态和中文故障，不合并两块电池测量值。

## 遗留

现场仍须确认继电器常开/常闭、失电状态、额定直流开断能力、辅助回读与供电路径。MOS
原始值、多字节端序和电流符号仍须厂家资料或同帧实测确认。
