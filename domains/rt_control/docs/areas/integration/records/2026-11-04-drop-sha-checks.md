---
id: integration-20261104-03
area: integration
title: 删除 SHA-256 校验（制品、CANopen 配置包、IgH 补丁、门禁与冻结用例）
date: 2026-11-04
type: behavior
trigger: 用户 2026-11-04“其他 SHA-256 我也不想留了”，四类全删
commits: [refactor/arch-drop-sha-checks]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 改动

- 组合制品：`artifact.yaml` 的 `files` 改为文件名列表（schema 3，旧制品按“版本不支持”拒绝并提示重新生成）；启动只核对选择与文件集合，不再拒绝被改过内容的文件。
- CANopen 配置包：不再写 `source.sha256`；预检直接比对包内 `bus.yml` 与制品中的总线文本。
- IgH：安装元数据由三条补丁 SHA-256 改为一行 `IGH_PATCHES=<按应用顺序的补丁文件名>`；`verify-host.sh`、bootstrap doctor、Dockerfile 同步。
- 力传感器台架构建标记改为“依赖提交 + 补丁文件名”文本。
- 分层门禁频率例外只要求文件存在；冻结用例 manifest 的输出改为文件名列表，删除历史输入哈希；BRT EDS 原件不再按哈希锁定（仍校验原件 + diff = 兼容件）。

## 影响

- 已安装 IgH 的主机，`dependency-versions.env` 仍是旧格式，`verify-host.sh` 会失败，直到重新执行 `igh-install.sh` 或补写 `IGH_PATCHES` 行（需要 root，另行授权）。
- 台架上已有制品需要重新生成。

## 验证

本机：`tools/tests` 191 项、hw_canopen/integ_composer/integ_bringup 290 项测试通过；三个 shell 脚本 `bash -n` 通过。未在主机上执行安装或校验脚本。

## 遗留

- rolling 公开接口里的轴集合/限位摘要（robot_interfaces 与 ctrl_rolling_trajectory，待同事拆分后）；文档出处与网页/审计标识另开 PR；robot_description 上游。
