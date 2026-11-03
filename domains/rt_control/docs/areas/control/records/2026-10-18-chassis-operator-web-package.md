---
id: chassis-control-20261018-03
area: chassis-control
title: 底盘网页操作台进入仓库（rt_control_operator_web）
date: 2026-10-18
type: feature
trigger: ELECTRI-127；用户 2026-10-18 决定把研华现场使用的底盘网页整理成包提交
commits: [feature/electri-127-operator-web]
env: native
risk: T2
writes: {reset: yes, enable: yes, motion: yes, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [chassis-control-20261018-02, contract-20260925-01]
---

## 背景

底盘网页一直以 ELECTRI-109 部署包形式存在于工控机（`tasks/ELECTRI-127-operator-web-v3-*`），
未进入 main。研华新架构压测期间在该部署包上增加了能力声明对接、故障复位、转向参数与现场调参。
本记录把现场版本整理为 `src/rt_control/rt_control_operator_web`。

## 内容

- 来源：研华工控机现场部署版本（2026-10-18 用户压测所用）。
- 与部署包的差异：
  - 只按 `/rt/capabilities` 对接，删除按组件名匹配的旧回退；
  - 现场调参路径由部署产物推导（`artifact.yaml` 的 `machine_unit`、`controllers.yaml` 的默认值），
    与 `rt_control_bringup.operator_tuning` 使用同一目录 `/var/lib/rt-control/tuning`，不再写死工控机路径；
  - 部署包的离线 wheels、`.venv` 与清单校验不进仓库，依赖改为 `python3-aiohttp`（Ubuntu 22.04）；
  - 入口 `rt_control_operator_web` 保留部署包启动检查：显式 ROS 域、仅本机 DDS、显式非实时 CPU、令牌文件 0600。
- 包加入 `tools/rt_control_layers.yaml`（integration）、CI 的依赖/构建/测试列表与 `bootstrap_native_dev.sh` 运行时列表。

## 验证

- 包测试 125 通过（协议、遥控、配置、服务端、现场调参文件、启动检查；含新增转向参数令牌复核与控制权检查）。
- `colcon build` + ctest 6/6 通过；`tools/quality_gate.sh` PASS。
- 本机冒烟：安装后的入口启动，页面返回 200，令牌登录后在无运行栈时显示“等待底盘运行栈启动”、使能与转向参数按钮禁用，无页面错误。
- 研华台架：同一功能的部署包版本已由用户压测使用；本包版本尚未在台架部署。

## 未验证与剩余风险

- 本包版本未在研华台架实机部署；部署时需安装 `python3-aiohttp`（系统包，3.8.1），部署包使用的是 aiohttp 3.14.5。
- 删除旧回退后，旧固定插件部署（不发布 `/rt/capabilities`）不能再用本网页。
