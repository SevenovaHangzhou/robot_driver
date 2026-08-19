---
id: release-deploy-20260819-02
area: release-deploy
title: ELECTRI-102 rolling 安装态依赖闭包与无硬件镜像验证
date: 2026-08-19
type: corrective
trigger: ELECTRI-102 完成性审计 / release-deploy-20260819-01
commits: [185a343b08be877cc96941e6187a102c270847ff]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [/tmp/e102-compose-config.yaml, /tmp/e102-container-image-inspect.log, /tmp/e102-container-package-smoke.log, /tmp/e102-container-controllers.log, /tmp/e102-container-full-graph.log, /tmp/e102-container-final-state.log, /tmp/e102-container-mock.log]
supersedes: []
related: [ELECTRI-102, contract-20260819-02, release-deploy-20260819-01]
---

## 背景

源码 overlay 构建通过不能单独证明发布镜像和目标机 Native 入口已安装 rolling 的完整运行
依赖。完成性审计发现 Docker、Native bootstrap 和 Native 启动前门禁虽然能通过传递依赖得到
部分包，却没有把 `robot_motion_interfaces`、`rolling_trajectory_controller` 与两个 rolling
QoS profile 明确列入安装后闭包；未来依赖图变化可能使软件测试绿而发布物缺包。

## 改动

- Docker runtime package 集合显式加入 `robot_motion_interfaces` 和
  `rolling_trajectory_controller`，并在 vendor 导入后检查 Motion 接口包存在。
- Native bootstrap 的 runtime package 集合、vendor layout 检查和安装后 QoS smoke 同步加入
  rolling 依赖。
- Native 生产入口在任何硬件访问之前，fail-closed 检查 Motion 接口、rolling controller 和
  `rolling_command`／`rolling_state` Python profile。
- 对上述三条路径增加结构门禁；测试提交与实现提交分开，便于独立复核 RED／GREEN。

## 验证

- 两个相关 policy test 文件共 63/63 PASS；shell 语法与 `git diff --check` PASS。
- 使用目标配置中已批准的 CPU 集合展开 Compose，确认候选 tag 为
  `rt-control:185a343b08be877cc96941e6187a102c270847ff`；只执行 `build`，未执行生产
  `up`。镜像完整构建 PASS，image ID 为
  `sha256:664f02f97a79910229d6c345cf1803b011541324fbfa265b1859378710c8fc8f`。
- `--network none`、不映射设备的安装态 smoke 能解析 Motion、RT、System、QoS 四个公共包和
  rolling controller；两个 rolling QoS Python factory 可实例化。
- 同一镜像以 `use_mock_hardware:=true`、ROS Domain 223、`network=none`、`ipc=private` 启动。
  当前审计宿主的 cgroup 实测可用 CPU 是 `0-23`，所以本机 smoke 使用 `0-23`；它不替代目标
  compose 的 CPU 集合，也没有把本机值写入仓库默认。
- 运行态为 `whole_body_jtc=inactive`、`rolling_trajectory_controller=inactive`、
  `enable_manager=active`；五个 rolling 端点、FJT Action 和 `/joint_states` 可见，禁止的 JTC
  topic 命令入口不可见。容器 inspect 为 `devices=[]`，停止后 exit 0、OOM false，日志无
  ERROR/FATAL；shutdown gate 返回 `already_disabled` 后有序关闭。

第一次本机 `docker run` 直接沿用目标 CPU 集合时，Docker 因本机没有 CPU 24～27，在创建
容器进程前拒绝请求；没有容器、ROS 进程或设备访问发生。读取本机 cgroup 后使用实际集合复验
通过，此环境差异不属于控制器缺陷。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: Docker 与 Native 发布路径必须显式安装并检查 Motion interface、rolling controller 和
  rolling command/state QoS，不能依赖偶然的传递依赖。
- F2: Native 入口必须在任何硬件访问前完成上述运行依赖闭包检查并 fail closed。
- F3: 实现提交 `185a343b08be877cc96941e6187a102c270847ff` 的候选镜像已在零设备、
  零网络的 mock 容器中证明 installed runtime closure 和 mandatory-inactive writer 图。

## 遗留

本记录只证明本机无硬件发布物 smoke。目标机 CPU affinity、DDS 尾延迟、STRICT 切换耗时、
真实总线与机械运动仍按 BQ-138～140 和现场授权另行验证；不得从本记录外推生产使能能力。
