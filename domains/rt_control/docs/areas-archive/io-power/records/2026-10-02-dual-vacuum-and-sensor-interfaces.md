---
id: io-power-20261002-01
area: io-power
title: 双侧真空与结构化传感器接口
date: 2026-10-02
type: feature
trigger: ELECTRI-152，RTC-01～10 软件实现
commits:
  - feature/electri-152-rt-interface-implementation
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [RTC-01, RTC-02, RTC-03, RTC-05, RTC-06, RTC-07, RTC-09, RTC-10]
---

## 背景

> 本记录不代表完整 Spec 验收通过；事务原子性、逐路时效和真实执行故障测试等软件缺口见
> [Draft 复核](../../contract/records/2026-10-03-interface-dependent-draft.md)。

旧分立 IO 仅有单泵／单压力路径，LED 使用含义混淆的 ColorRGBA，双 E08 任一失败会阻止
整批发布。本次按 V3 需求实现独立资源和结构化有效性，不填实物地址。

## 改动

- `plc_io_modbus` 支持共用泵、左右阀和左右压力三类独立资源；私有写服务表达确认、未执行和
  结果未知。V3 地址、寄存器和传感器身份均保持 `-1/TBD`，`vacuum_system.configured=false`。
- 真空阈值固定为单个有效样本 `<= -60 kPa` 建立、`>= -1 kPa` 释放；阀 Action 不等待压力。
  普通停泵要求两阀已释放、两侧压力满足释放、无活动或未知写结果。
- LED 改为 `/led/set_rgbw` 服务，非法值拒绝，成功仅证明 Modbus 写确认；退出不改色。
- 两台 E08 分别读取／发布，失败一台不跳过另一台；逐路状态记录真实读取完成时间，2.5 s
  过期，无目标 `+Inf` 与协议异常 `NaN` 保持区分。
- 红外只按开关量发布结构化状态；通道和电平未确认，配置为 `configured=false`。
- LPMS 将标准 IMU 发布到 `/imu/data` 并发布结构化质量状态；磁场仍为 RT 工程话题。
- BMS 保持只读电池状态，未增加换电或 MOS 写命令。

## 验证

- 真空核心 13 项测试覆盖阈值边界、泵前置、不同侧并行、双侧忙预检、部分成功、未知结果、
  取消、不去重和停泵保护，PASS。
- `plc_io_modbus`、`modbus_tcp_rtu485`、`lpms_nav3_can` 隔离构建 PASS。
- Modbus 传输／协议与无设备生命周期测试 PASS；LPMS 25 项包测试 PASS；旧 PLC 包构建与测试 PASS。
- 未执行 PLC／LED 写入、CAN 访问、目标机或实机测试。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 双侧真空软件不再复制单泵／共享压力为左右独立事实；缺物理映射时整体 fail closed。
- F2: E08 模块故障隔离在软件实现，Range 数据年龄超过 2.5 s 为过期。
- F3: LED 公共命令使用完整 RGBW 服务，通信确认不表示实际发光。
- F4: 红外是开关量观测；BMS 本轮只读；IMU 失效由 Navigation 决定降级／停止。

## 遗留

真空地址／压力标定、红外数量／电平、超声波外参、IMU 共线配置与外参、LED 实际写入、
BMS 字节序和换电方案均待实物信息或获准测试，集中见 BQ-152。
