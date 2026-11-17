---
id: integration-20261116-02
area: integration
title: 操作网页迁至独立仓库 robot_operator_web，本仓库删除 integ_operator_web
date: 2026-11-16
type: removal
trigger: ELECTRI-209/210；用户 2026-11-16 决定把网页拆成独立仓库，并选择先删除、再迁移 .40 的本地补丁（方案 A）
commits: [refactor/remove-operator-web]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [integration-20261116-01]
---

## 背景

`integ_operator_web`（状态、底盘、机械臂与头部页面）已迁到组织仓库
[SevenovaHangzhou/robot_operator_web](https://github.com/SevenovaHangzhou/robot_operator_web)，按日志站的方式独立部署。
迁移保留了 #159 起的提交历史，导入时内容与本仓库 2129bc6 逐字节一致。

## 改动

- 删除 `src/rt_control/integ_operator_web/`，以及 CI 构建/测试列表、`tools/bootstrap_native_dev.sh` 中的包名。
- 删除 `tools/arch_rules.yaml` 中网页序列预设的 NAME-3 例外（文件已不在本仓库）。
- 域 README、integration 区 README 与网页设计稿指向新仓库。

## 新仓库的约定（摘要，以新仓库 README 为准）

- 网页仍属 rt_control 域。可使用公共接口，以及新仓库 README 中列出的 RT-Control 内部接口（`controller_manager`、控制器参数等）。
- `compat.yaml` 固定一个 robot_driver 提交，`deps.repos` 固定同一个 robot_interfaces；
  模型来自 robot_driver `src/description/source-lock.yaml`，由目标机已安装的 robot_description 提供。
- 部署：`deploy/install.sh <提交> <robot_driver setup.bash>` 叠加构建，`robot-operator-web.service` 用户服务。
  研华 Alfa_V3-Advantech-01 于 2026-11-16 已改用新仓库，旧的 `rt-operator-web(-v3).service` 已删除。
- 网页只在持有底盘控制权时才是 `/rt/base/cmd_vel` 的发布者（ELECTRI-222）。

## 遗留

- 台架 .40 上运行的网页仍来自 `head-arm-independent-20261116` 发布：本地候选提交 `de2ae668`，未推送。
  其补丁修改了 `integ_operator_web` 的 `head/ros_bridge.py`、`launcher.py`、`node.py`（`--head-instance-file`、`--view-model-file`）。
  这部分须改到 robot_operator_web 提交；launch 部分仍在本仓库。在此之前 .40 不换用新仓库网页。
- 本记录不改变运行中的部署；.40 当前进程不受影响。

## 结论与冻结事实

- F1: 操作网页的源码与发布只在 robot_operator_web；本仓库不再构建或测试它。
