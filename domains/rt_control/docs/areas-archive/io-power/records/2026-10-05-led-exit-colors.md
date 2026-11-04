---
id: io-power-20261005-01
area: io-power
title: LED 正常与异常退出颜色策略
date: 2026-10-05
type: corrective
trigger: 用户要求正常退出无色、异常退出红色
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no, led: attempted-no-ack }
verified: PARTIAL
evidence: []
supersedes: ["io-power-20260916-01#F2"]
related: ["io-power-20261002-01"]
---

## 改动

- `led_strip_node` 在 executor 正常返回时向六个控制器分别写入 RGBW 全零。
- 未捕获异常从 executor 退出时，在返回失败前向六个控制器分别尽力写入红色
  `255,0,0,0`；单路失败不阻止其余控制器。
- `exit_color_enabled` 默认为 `true`；可显式关闭供外部颜色责任方接管。
- 六路退出写使用独立的 `exit_response_timeout_ms=100`，避免未接入站号按普通业务超时
  累计阻塞退出；所有六个站号仍依次尝试。
- 离线生命周期测试显式关闭退出写，仅覆盖 SIGINT/SIGTERM 进程收尾；颜色纯函数断言
  正常为 `0,0,0,0`、异常为 `255,0,0,0`，实机结果由现场人员验证。

## 验证

- 修改后的锁定依赖构建与包内离线测试通过。
- 开发期间曾向当前 HC0/502 发送一次红色服务命令和正常退出清零请求，均未收到 RTU
  应答，不作为实机通过证据；按用户要求不再由开发侧执行实机验证。

## 安全边界

节点自身无法在 SIGKILL、进程内存损坏、宿主断电或网关/RS485 链路失效后发送 Modbus
命令。这些故障下如需保证红灯，必须由独立进程或硬件看门狗拥有检测和写灯职责；本改动
不声称提供该保证。

## 结论与冻结事实

- F1: 可执行的正常退出路径写无色；被 `main` 捕获的致命异常路径尽力写红色。两种路径均
  对六个控制器逐路执行，且不把 Modbus 确认表述为实际发光反馈。

## 遗留

- 需获准后实机验证六路正常退出写、异常退出写、部分控制器离线时其余控制器继续写入，
  以及总收尾时延。
