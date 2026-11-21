---
id: hw-config-20261122-02
area: hw-config
title: 研华转角编码器 0x7320 清除（0x2690=10）后按外置编码器重算转向零点
date: 2026-11-22
type: commissioning
trigger: 用户 2026-11-22 清除四台转角驱动 0x7320 后选择用外置编码器换算零点，并目视确认四轮在 ±3° 内（ELECTRI-233）
commits: [bugfix/advantech-steering-zero]
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [hw-config-20261105-01, hw-config-20261116-02, chassis-control-20261017-01]
---

## 背景

研华 Alfa_V3-Advantech-01 四台转角驱动（ring 1/3/5/7）同时报 `0x603F=0x7320`（`0x2601` bit2，磁电编码器内部错误）。
现场对 `0x2690` 写 10 后故障清除；用户说明该操作复位故障状态并清除多圈计数。清除前后 `0x6064` 的差正好是
65536 的整数倍（−8、−37、0、−105 圈），只保留单圈值，原 `steering_zero_counts` 失效。
按旧零点换算的清除前读数（22°、−40°、−135°、385°）也与外置编码器不一致，模块 3 清除前已是单圈值。

## 改动

`units/Alfa_V3-Advantech-01/swerve_calibration.yaml` 的 `steering_zero_counts`：

| 模块 | 外置编码器角 | `0x6064` | 新零点 | 旧零点 |
| --- | --- | --- | --- | --- |
| module_1 | −1.26° | 6245 | −25747 | 46212 |
| module_2 | −1.31° | 44927 | 11433 | −3394086 |
| module_3 | +1.63° | 12830 | 54374 | −3430317 |
| module_4 | −3.04° | 24429 | −53061 | 2967074 |

换算：外置角 `q = remainder(direction·raw·2π·27/(65536·108) − installation_offset, 8π)`；
`steering_zero_counts = 0x6064 − q·(−1·65536·140/2π)`。外置编码器方向、安装偏置、140:1 与轮组坐标均未改。

## 验证

- T2 只读：can3 被动接收节点 11–14 TPDO1（`6004`），EtherCAT SDO 读 `0x6064`/`0x603F`；工控机重启后复读计数不变、无故障。
- 用户目视四轮朝向在 ±3° 内。
- 未验证：使能后电机与外置编码器差值、低速转向方向与角度；目视无法区分 0° 与 180°，依赖原安装偏置正确。

## 结论与冻结事实

- F1: 研华转角驱动对 `0x2690` 写 10 复位故障状态并清除多圈计数（用户 2026-11-22）；之后必须重算 `steering_zero_counts`。
- F2: 研华 2026-11-22 转向零点为上表新值。

## 遗留

- 四台同时 0x7320 的根因未查明。
- 使能低速核对（ELECTRI-233 T3/T4）。
