---
id: contract-20260903-01
area: contract
title: Qt 操作员故障控制台首版
date: 2026-09-03
type: feature
trigger: 用户口头需求（2026-09-03，操作员需直观看到故障因果与执行受控复位/使能/停机）
commits: [feature/rt-control-operator-console]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-070, BQ-141, BQ-142, contract-20260828-01, release-deploy-20260903-01]
---

## 背景

现场故障当前散落在日志、状态字和标准诊断中，操作员无法快速区分第一故障与后续连锁故障，
也无法核对厂商手册含义。本变更在不新增公共 schema、不中断 250 Hz 控制环的前提下提供独立 Qt 界面。

## 改动

新增 `rt_control_operator_ui`：消费 `/diagnostics`、公共 `/control/safety_state` 和
`/battery_state`，显示 Domain、总线、CPU14、电池、第一故障、时间顺序故障列表与手册详情；
相同故障按指纹合并计数，历史恢复只标记不删除。Reset/Enable/软件停机按钮均为单请求、无自动重试，
软件停机明确标注非安全级。`rt_diagnostics` 为 EtherCAT 轴补充 joint、ring position、vendor、
AL 原值与 statusword 原值/十六进制键，没有新增 topic 或公共接口。

故障目录只收录已核对的 ZeroErr V1.9 `0xA000` 以及钛虎 C1 1.1.1
`0x2310..0x8611`，每条记录保存手册页码与源 PDF SHA-256；PDF 本体未进入仓库。

## 验证

- `colcon build --symlink-install --packages-select rt_diagnostics rt_control_operator_ui`：PASS。
- `colcon test --packages-select rt_diagnostics rt_control_operator_ui`：PASS，46 aggregate tests，
  对应 36 个 focused pytest cases。
- `QT_QPA_PLATFORM=offscreen python3 -m pytest ...`：PASS，36 focused tests。
- `QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=232 timeout --kill-after=2s 3s ...`：PASS，
  退出码 124、无 traceback，SIGTERM 后 Qt/ROS 线程有界收尾。

未执行：实机故障注入、按钮服务调用、0x603F 自动采集与现场显示验收；其中 0x603F 通道受
BQ-142 阻塞。本记录不授权使能或运动。

## 结论与冻结事实

- F1: 操作员 UI 复用标准 `/diagnostics`、公共 `SafetyState` 与 `/battery_state`，不新增
  enable-manager 状态 topic/message，也不进入实时路径。
- F2: 厂商错误解释必须带手册版本、页码和源文件 SHA；未知码保留原值并禁止猜测或自动复位。
- F3: Reset、Enable、软件停机每次点击至多发起一个请求，超时/失败均不自动重试；软件停机不替代
  物理急停、安全继电器或 STO。
- F4: 故障日志允许人工滚动，但同一指纹只更新次数和首末时间，恢复后历史不消失。

## 遗留

BQ-142 的两份匹配型号 ESI 已确认 `0x603F` 可映射到 TxPDO，但 14 轴 PDO 变更仍需用户明确
裁决；还需在隔离 Domain 的无硬件集成环境验证真实 DDS/QoS，并在单独授权下完成一次现场只读
显示和按钮失败路径验收。
