---
id: io-power-20260914-01
area: io-power
title: 分立 IO 模块真空桥接
date: 2026-09-14
type: feature
trigger: 用户提出的分立 DI/DO 与 AI 模块接入需求
commits:
  - feature/rt-control-plc-io-modbus
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

# 分立 IO 模块真空桥接

## 背景

旧 `plc_node` 面向一体 PLC 寄存器。当前硬件改为独立数字量和模拟量
Modbus TCP 模块，需要在不改变公共真空接口的前提下替换硬件适配层。

## 改动

- 新增 `plc_io_modbus`：FC02 读取激光 DI，FC01/FC05 读写泵继电器 DO，
  FC04 读取 ZSE30A 模拟量并线性换算 kPa。
- 配置使用直接地址 `di_address: 0` 和 `do_address: 0`；吸牢阈值为
  `-80.0 kPa`，释放阈值为 `0.0 kPa`。
- `PlcIoState` 增加压力有效性、压力值和释放状态。单回路硬件下，
  `left_solenoid_on`/`right_solenoid_on` 与唯一的泵继电器读回保持一致。
- `VacuumGrip.GRIP` 开泵后等待有效压力达到阈值；`RELEASE` 关泵后
  等待压力回到释放阈值，并支持超时和取消。

## 验证

- `tools/quality_gate.sh`：PASS，277 passed，13 skipped，门禁覆盖率 83%。
- `python3 -m pytest src/rt_control/control_api_adapter/test/test_vacuum_adapter.py
  tools/tests/test_rt_io_integration.py tools/tests/test_rt_control_native.py -q`：PASS，
  92 passed。
- 隔离临时目录中 `colcon build --packages-up-to plc_io_modbus`：PASS，
  `rt_control_interfaces` 与 `plc_io_modbus` 编译安装成功。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `plc_io_modbus` 只提供 RT-Control 内部 `/plc/io_state`、传感器 Topic
  和单个 `/plc/vacuum_pump` 继电器服务；公共入口仍由 `control_api_adapter` 提供。
- F2: FC01/FC05 读回只证明 IO 线圈状态；工件吸牢只由有效压力
  `<= -80.0 kPa` 判定。左右公共通道映射同一条物理真空回路。
- F3: `GRIP` 写继电器 ON 后等待吸牢；`RELEASE` 写 OFF 后等待
  有效压力 `>= 0.0 kPa`。取消或超时只停止等待，不额外反转继电器。

## 遗留

- 未执行 Docker 构建、容器内启动、目标机部署或实机吸附/释放闭环。
- 整个 `rt_control_bringup` 隔离构建因当前环境缺少 `lely_core_libraries`
  而未完成；CI 需在导入冻结 vendor 依赖后完成全闭包构建。
- `-80.0 kPa` 吸牢阈值和 `0.0 kPa` 释放阈值需在目标机上验证噪声余量与回差需求。
- 当前左右通道无独立物理阀，不提供独立左右控制。
