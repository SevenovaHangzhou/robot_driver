---
id: hw-config-20261116-01
area: hw-config
title: 研华底盘行走轮方向取反移入 Kinco 驱动器（607E=1），软件 direction 改为 +1
date: 2026-11-16
type: commissioning
trigger: 用户 2026-11-16 要求改驱动器配置反向，选方案 A（取反移到驱动器，软件 direction 改为 +1）
commits: [feature/drive-wheel-polarity]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: [Advantech ~/Robot/rt-control/records-v3/drive-polarity-20261116T070322Z]
supersedes: []
related: [hw-config-20261105-01, chassis-control-20261017-01]
---

## 背景

2026-10-16 实机修正把四个行走轴的反向放在软件 `direction: -1`。用户要求改为由驱动器自身反向。
Kinco FD 对象 `607E:00`（速度位置方向控制，RWS，0=逆时针为正、1=顺时针为正，手册 V3-DOC-2026-014 §8.3）
在驱动器内部翻转运行极性。

## 改动

- 研华 Alfa_V3-Advantech-01 行走驱动器 ring 0/2/4/6：`607E:00` 0 → 1，逐台读回一致，`0x2FF0:01=1` 存储控制参数。
  转向驱动器 ring 1/3/5/7 未改（仍为 0）。
- 写前守卫：主站 Idle、8 台 PREOP、身份 6820200/4604997、`6041=0x0231` 未使能、`603F=0` 无故障。
  基线、写入与写后读回保存在 evidence 目录；恢复脚本 `restore.sh`（写回 0 并存储）。
- `swerve_chassis.yaml`：四个行走轴 `direction` -1 → +1。转向轴保持 -1。
  驱动器与软件的乘积不变，预期车辆运动与修改前一致。

## 验证

- 写后读回：ring 0/2/4/6 `607E=1`，ring 1/3/5/7 `607E=0`；8 台仍 PREOP、无故障。
- 离线：`integ_composer` 与 `hw_ethercat` Python 测试 250 项通过；3 项失败在 main 上同样失败（本地缺安装 share/手臂插件），与本次无关。
- 未验证：驱动器断电重启后 `607E` 保持；`607E` 对 6064/606C 反馈符号的作用（写入时行走位置接近 0，静态无法判断）；
  使能后点动方向与里程计符号。本次未使能、未运动。

## 结论与冻结事实

- F1: 研华 Alfa_V3-Advantech-01 行走驱动器 ring 0/2/4/6 保存的 `607E:00=1`；转向驱动器为 0。
- F2: 该机行走轴软件 `direction=+1` 只在驱动器 `607E=1` 时成立；更换或恢复出厂的行走驱动器必须先写 `607E=1`，否则该轮反向。

## 遗留

- 断电重启后回读 `607E`；使能后单轮低速点动确认方向，需另行授权。
- 启动前检查未读 `607E`；是否加入 preflight 守卫待定。
