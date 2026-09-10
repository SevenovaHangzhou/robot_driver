---
id: ecat-axes-20260910-02
area: ecat-axes
title: ELECTRI-97 将1 kHz启动配置补齐到Docker并保持125 Hz公共状态契约
date: 2026-09-10
type: fix
trigger: 用户批准将实机验证过的启动优化提交PR并合并main
commits: [fix/electri-97-startup-realtime]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [/home/kkozia/research_reviews/electri97-main-pr-20260910]
supersedes: []
related: [ELECTRI-97, ecat-axes-20260910-01]
---

## 背景

Native 两次失能启动在 6.821/6.381 秒到达全部配置从站 OP/WKC 完整。
进入 main 还需要可复现容器构建及安装入口启动证据；本记录补齐该交付路径，
不将容器 Mock 结果外推为实机 1 kHz 运动或冷启动认证。

## 改动

- 将原实验目录中的 ICube 0011 原样提升到补丁主目录，Native/Docker 均应用
  0001..0011。Docker 默认开启连续交接、统一 shift 0 和显式 send interval。
- Compose 必须提供已验证的单个 RT CPU，FIFO 优先级为与控制器一致的 80；
  IPC 包装器使用已有核配置传参。没有增加 device、capability 或宿主权限。
- controller_manager 和 EtherCAT startup/SYNC0 均为 1000 Hz；Ti5/Updown
  0x60C2=1 ms。JSB 显式请求 125 Hz，保持冻结 R-OUT-03 的频率和 14 关节集合。
  原 250 Hz 控制器曾把配置的 100 Hz 量化为 125 Hz；1 kHz 下需显式保留125。
- 迁移门禁仅增加已批准的 1 ms 插补和 Ti5 已校验 PDO 保留差异；其他字段仍逐项比较。
- 保持 DC 等待上限 5 s、接受条件 10 us 和初始 offset 阈值 1 ms。
  100 us 候选仅归档，不进入默认构建；定时激活和额外探针均不在正常启动路径。

## 验证

`tools/quality_gate.sh` 搭配明确的冻结源码输入和 ShellCheck：279 passed、0 skipped，
门禁工具覆盖率 83%。首次公共状态参数断言仍为旧100，修正契约测试后通过。
配置/迁移 scoped 测试31项通过；最终相关配置检查46项通过，2项源码环境缺失时跳过，
已由上述完整源码门禁覆盖。冻结 legacy 6bc94cd 的925个语义值比较100%通过。

标准 `docker build -f docker/rt-control/Dockerfile` 成功构建运行依赖闭包，无源码注入，
IgH与ROS依赖使用仓库固定SHA。Compose通过包装器展开，核值来自已有IPC核配置。
镜像通过安装后的 rt_control_start 启动，无设备映射、network=none、Mock hardware，
PLC/BMS关闭。接口探针读回 controller_manager=1000 Hz，14关节 /joint_states
实测125.0005 Hz；JSB、内部JSB、diff_drive、enable_manager为active，whole_body_jtc
为inactive。/rt/disable返回already_disabled；SIGTERM后记录控制器quiesce及
EtherCAT硬件inactive，退出0，无UNCLEAN_SHUTDOWN。验证容器已删除，日志保留。

本次只补齐main软件/容器交付，不在工控机执行新的启动、运动或部署。
无公共接口schema、Robot Model、坐标/比例或PDO条目变化。实机提速证据见前一记录，
本次显式125 Hz公共发布和Docker入口仅做Mock验证。用户已批准提交/PR/main合并。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: Native和Docker均包含完整启动补丁链及1 kHz周期；Docker连续交接须有经验证的RT核。
- F2: /joint_states继续满足R-OUT-03的125 Hz和14 EtherCAT机械轴契约。
- F3: 容器安装入口、控制器生命周期及有序停机T1通过；不代表实机Docker发布切换已验收。

## 遗留

双X503停机0x001A、长时间抖动、冷启动/PDO分配掉电保持性、使能运动和跟随误差仍未闭环。
0x10F1:02仍为250计数，1 ms周期下nominal容忍时间约250 ms，不能沿用旧1秒描述。
当前IPC Docker包装器还有已记录的PCIe CAN迁移/发布门禁，本PR不宣称已部署容器。
公共契约注释里的旧250 Hz实现说明由其所有者后续更新，本PR保持频率、schema和pin不变。
