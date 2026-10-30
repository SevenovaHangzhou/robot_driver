---
id: release-deploy-20260912-02
area: release-deploy
title: ELECTRI-118 同步最新 main 并解决五处冲突
date: 2026-09-12
type: corrective
trigger: 用户要求先同步最新 main 并检查冲突
commits: []
env: none
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [ELECTRI-118, BQ-147, ecat-axes-20260912-11]
---

## 背景

当前开发基线为 308dbfca75c2fb5fec53241a6e6ead37dcd4f8fa，包含尚未提交的三代机
配置、PP、舵轮与原位使能改动。最新 origin/main 为
34acbae4e58ff69d762d993f84c3ad483b23c58d，比基线多 28 个提交。
为保留此前实机验证环境，在新分支 kozia/electri-118-main-sync 中以最新 main 为基线同步。

同步过程中，共享仓库的 origin 被其他操作切换为内网仓库，其 main/本地 origin/main
变为 308dbfc；此次没有修改该远端设置。再次显式查询用户最初指定的 GitHub URL，
main 仍为 34acbae。因此所有同步与保留性检查使用该固定 SHA，不使用已变化的 origin/main。

## 改动

完整保存原工作树的 100 个修改/新增文件（其中 80 个原为未跟踪文件），用独立索引
生成带完整 blob 身份的补丁，再 git apply --3way --index 到新工作区。原工作树与索引
未改；新文件逐字节核对后保留。没有创建 commit、push、PR 或更新工控机部署。

实际产生五个文件、六个冲突区块，处理如下：

| 文件 | 冲突内容 | 处理 |
| --- | --- | --- |
| docker/rt-control/Dockerfile | 双方在第七个 EtherCAT 补丁后追加不同补丁 | 保留 main 0008..0012，追加本轮周期钩子为 0013 |
| tools/bootstrap_native_dev.sh | 补丁应用和预期树验证两处列表冲突 | 两处与 Docker 顺序一致，0001..0013 |
| domains/rt_control/BLOCKED-questions.md | main 的 X503B 与本轮 PP 均占用 BQ-143 | 保留 main BQ-143，本轮 PP 改 BQ-147，并更新本轮引用 |
| domains/rt_control/PROGRESS.md | 两边在时间线尾部追加事件 | 保留双方所有事件 |
| domains/rt_control/docs/areas/ecat-axes/README.md | 事实表 04#F1 与记录列表重复 | 保留 main 的 04，本轮事实顺延至 05..15，合并列表和交叉引用 |

功能实现文件没有文本冲突。保留 main 的 1 kHz 默认旧机配置、启动实时/连续交接
修正、X503B 代码及 CI；三代机独立原位入口仍为用户指定并已在旧基线实测的 250 Hz。

## 验证

- 原工作树 status/index/内容与备份一致，100 文件全部导入，80 个新增文件导入时
  逐字节一致；冲突文件 index stage 1/2/3 和标记均已消除。
- 在独立干净 vendor workspace 导入四个原 pin，执行更新后的 bootstrap prepare，
  EtherCAT 0001..0013、CANopen 0001..0005、controllers 0001..0003 应用成功。
- 配置、launch、stationary、hardware composition 测试共 124 项通过。
- tools/quality_gate.sh：266 passed、13 skipped，受门禁代码覆盖率 83%；
  ShellCheck 本机未安装。跳过项依赖未配置的外部源码环境，不计作验证通过。
- git diff --check、暂存区检查、shell 语法及冲突标记检查通过；此次没有全量 ROS
  编译、最终 Docker 构建、CI 或新基线实机回归，不沿用旧基线 6.66 s 使能作为同步后证据。
- 原始快照树 3664055adf5339c8e95ee24e23e1ff497f00cf91；备份 tar SHA256
  648f517ba635753b5091422c6e6a1785469c1bc29c54bfd1bd6524bd8c91066a；补丁 SHA256
  560156e5a7743ca06c41faa578d839e5c90943f27955a44498a4e83fb6fa21ff。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 同步工作区已基于 main@34acbae，五处文件冲突全部解决，保留双方改动。
- F2: 本轮 PP 周期钩子补丁现在为 0013，PP 标定事项现在为 BQ-147；main 的补丁
  编号和 BQ-143 不被替换。
- F3: 同步和配置验证完成不等于 main 发布准入完成；新工作区尚未提交或推送。

## 遗留

在新工作区继续全量构建与相关回归，补齐最终容器启动/退出和 CI 证据。新 main 的
时间基准、连续周期交接和主机参数与三代机 250 Hz 原位入口须联合验证。
当前工控机与原开发工作区保持此前通过实机测试的版本。
当前同步分支的 upstream 随共享 origin 设置指向内网旧 main；提交/推送前必须明确
目标远端，不从“ahead 28”推断本轮创建了 28 个新提交。
