---
id: io-power-20260930-01
area: io-power
title: 金凤凰 BMS V1.1 C++ 驱动迁移
date: 2026-09-30
type: feature
trigger: 用户要求按《电池-金凤凰.pdf》替换旧 BMS 节点并迁移到 C++
commits: []
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-121, BQ-124]
---

## 背景

旧 Python 节点只被动解析标准帧 `0x3FC`，与金凤凰 V1.1 的 250 kbit/s、29 位扩展帧和
默认查询应答模式不兼容。本次按用户提供的 8 页协议替换实现，不保留旧电池双协议路径。

## 改动

- `bms_node` 改为 C++17，删除 Python 节点、脚本、`0x3FC` 解析和 Python 测试。
- 每 200 ms 在 `can1` 发送 `0x18900140` 八字节零查询，只接受 `0x18904001` 八字节扩展数据帧。
- 包内 `config/bms_node.yaml` 固定 `golden_phoenix_v1_1`、250000 bit/s、BMS 地址 1、主机地址 64；
  owner-local 配置已随包安装；最新模块化 main 尚未组合 BMS runtime，本变更不恢复已退役 launch。
- 保持 `/battery_state`、5 s 发布和 3 s freshness 契约，只输出电压、SOC 与 present。
- can0 保持 500 kbit/s；BMS can1 的宿主配置和验收预期改为 250 kbit/s。
- PDF 未定义字节序；解析器支持 big/little，并以 SOC 原始范围做保守 auto 判定，歧义帧不更新状态。

## 验证

- 隔离 `colcon build`：`robot_interfaces_qos` 与 `bms_node` 编译、链接和安装通过。
- `colcon test`：15 项通过，覆盖大小端、auto 歧义、ID、扩展帧、freshness 和线程退出。
- `bash -n`：通过；相关仓库 pytest：26 项通过；`tools/quality_gate.sh`：207 项通过、12 项跳过。
- 未访问 `can1`，未发送真实 CAN 帧，未启动 PLC/EtherCAT/CANopen，未使能或运动。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `bms_node` 只支持金凤凰 V1.1；查询 ID 为 `0x18900140`，响应 ID 为 `0x18904001`，均为 DLC 8 扩展数据帧。
- F2: BMS 使用 `can1` 250 kbit/s；CANopen `can0` 继续使用 500 kbit/s。
- F3: ROS 契约保持 `/battery_state` 5 s 发布，超过 3 s 无有效响应则电压/SOC 为 NaN、present=false。
- F4: 协议未定义多字节字节序，真实设备抓包确认前保留 auto 的歧义拒绝策略，不把离线测试写成实机结论。

## 遗留

- 在真实金凤凰 BMS 上确认查询响应、实际字节序、电流方向及 HMI 电压/SOC 对照。
- 项目锁定的 `robot_interfaces` 提交 `9aa2693d...` 当前无法从远端检出；本次隔离构建使用远端当前 QoS
  API，只证明源代码与该 API 兼容，不构成冻结依赖可复现构建。
