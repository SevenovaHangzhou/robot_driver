---
id: realtime-host-20260904-01
area: realtime-host
title: 当前工控机真实硬件身份锁迁移
date: 2026-09-04
type: fix
trigger: ELECTRI-112
commits: [fix/electri-112-ipc-identity]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - ELECTRI-112
supersedes: []
related: [ELECTRI-94, realtime-host-20260821-01]
---

## 背景

ELECTRI-94 合入 `main@06e4a1224dee731d2726645e8cefe69dc08ea7b7` 后，目标机在全新
版本化 Native workspace 完成构建和只读现场预检；真实 `rt_control_native.sh doctor`
随后按设计拒绝旧身份，报错 `must run as ar`。操作者明确批准实际
`user` / `localhost` / `6.8.1-1057-realtime` 为新的目标身份，因此必须在主线修复身份锁，
不得在目标机绕过或临时修改 launcher。

## 改动

- `tools/rt_control_native.sh` 与 `tools/rt_control_ipc.sh` 的只读身份常量从
  `ar` / `ar-Default-string` / `5.15.0-1032-realtime` 更新为
  `user` / `localhost` / `6.8.1-1057-realtime`。
- Docker IPC 发布根随运行账号从 `/home/ar/rt-control-releases/...` 更新为
  `/home/user/rt-control-releases/...`；ROS Domain 仍是显式部署输入，没有固定为现场使用的 7。
- 更新身份合同测试、IPC launcher policy 与当前态文档；旧目标、旧内核和旧 IP 的历史记录
  不作批量改写。
- CPU14、EtherCAT MAC、18 responder、IgH、PCIe CAN、确认口令、单主站以及
  reset/enable/disable 顺序均未修改。

## 验证

- TDD RED：在 `main@06e4a122` 上运行新身份 focused test，结果为 1 failed；失败精确来自
  两套 launcher 仍缺少新身份常量。
- TDD GREEN：`python3 -m pytest -q tools/tests/test_rt_control_native.py` 为 61 passed。
- `bash -n tools/rt_control_native.sh tools/rt_control_ipc.sh tools/check_rt_control_ipc_launcher_policy.sh`
  通过；`bash tools/check_rt_control_ipc_launcher_policy.sh` 通过。
- 本分支 `tools/quality_gate.sh` 为 212 passed、门禁覆盖率 83%；本机缺少 ShellCheck，
  该项必须由 GitHub governance CI 补齐。
- 目标机只读预检：单一 Master0、18 responder 全 PREOP、Idle/Inactive；Lost frames 基线
  390 在观测窗内零增量；PCIe CAN 为 `10b5:9140` / `zpcican`，L0..L3 的 `dev_id=0..3`，
  can0/can1 均为 500 kbit/s、ERROR-ACTIVE、txqueuelen 128，且无 Socket owner。
- exact 旧 main 的干净 Native workspace 已完成 29 包构建、五个 QoS profile 检查、
  bootstrap doctor 和 211 项质量门禁；真实运行 doctor 仅在旧用户身份锁处失败，未启动控制栈。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前真实硬件 launcher 的批准身份为用户 `user`、hostname `localhost`、内核
  `6.8.1-1057-realtime`。
- F2: 当前版本化 Native 验证根为
  `/home/user/rt-control-main/releases/<short-main-sha>`，源码位于其 `robot_driver/` 子目录；
  发布 SHA 变化时必须新建目录，不覆盖旧 release。
- F3: ROS Domain 仍由部署显式提供；2026-09-04 本机联合验证使用 Domain 7，不构成仓库默认值。
- F4: 旧 `ar@192.168.0.40` 及 `5.15.0-1032-realtime` 记录继续作为历史审计证据，
  不再定义当前 launcher 身份。

## 遗留

- 本变更合入后，必须从新 `main` 创建新的版本化 release，在目标机重新完成真实 wrapper
  doctor 后，才可按单独现场授权进入一次短时 Native enable/disable/stop。
- 本轮明确不运行 Docker。`rt_control_ipc.sh` 的身份与账号路径已对齐，但 PCIe CAN Docker
  迁移、镜像构建和现场运行均未验证，不得把本记录写成 Docker 准入结论。
