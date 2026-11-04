---
id: ecat-axes-20260908-01
area: ecat-axes
title: ELECTRI-97 应用时间统一为单调时钟并分阶段复测
date: 2026-09-08
type: fix
trigger: ELECTRI-97 / 用户授权继续二代机启动 DC 排查
commits: [fix/electri-97-startup-realtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PARTIAL
evidence: [/home/kkozia/research_reviews/electri97-gen2-20260908/timebase]
supersedes: []
related: [realtime-host-20260908-01, realtime-host-20260908-02]
---

## 背景

已修正 Native 运行线程绑定顺序及 IgH hrtimer 构建后，首次全 OP 仍需数十秒。
ICube 初始化使用 CLOCK_MONOTONIC，周期 update/writeData 使用 CLOCK_REALTIME；
两者又共用减去 2000 年纪元的转换，导致 monotonic uptime 的无符号下溢。

## 改动

新增上游补丁 `0008-use-monotonic-application-time.patch`：四条 application_time 路径
均采用 CLOCK_MONOTONIC，以 `seconds * 1e9 + nanoseconds` 转换，不减日历纪元。
Native bootstrap 与 Docker 补丁链均注册此补丁。控制周期保持 4 ms，SYNC0 shift、
PDO、application_time 调用位置、同步 API 顺序及 IgH offset 算法在本组实验不变。

## 验证

实际宏编译回归先复现零 uptime 得到 `17500059273709551616`，修正后通过零值、
纳秒进位及大于 32 位秒数测试。单调时间补丁四项回归、目标 ethercat_interface 的
五项 PDO 测试通过；本阶段本地质量门禁 227 项通过。

目标 Native 包经正常 colcon 重编译，运行进程 maps 确认加载新库：
`ab7a4b776321c7b61b2e2368ddd9cdaf96e48e965aff3cdf14622e4dafc36e0b`。
运行时 application_time 与主机 monotonic 相差约 2.916 ms，符合 4 ms 更新周期。
ROS 消息头的时间仍来自 ROS 节点时钟；没有修改主机墙钟。

| 轮次 | 激活至首次全 OP/WKC | DC 5秒等待警告 | UNMATCHED | TIMED OUT | lost 增量 | 稳定后8节点 DC 最大绝对值 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 56.284146242 s | 6 | 1 | 0 | 0 | 570 ns |
| 2 | 30.524277986 s | 3 | 1 | 0 | 0 | 959 ns |
| 3 | 36.004148734 s | 4 | 1 | 0 | 0 | 2003 ns |

三轮均通过 Native READY 和后续 15 秒 OP/WKC 采样，但两台 X503 每轮均出现
AL 0x001A 瞬态后恢复。停机仍有 Ti5 故障锁存及 UNCLEAN_SHUTDOWN。
原源码、库和构建入口备份位于目标 `rt-control-backups/electri97-timebase-20260908`。

本次实机操作经用户会话授权，采用 Native start/stop 确认入口；本组无 reset、enable、
运动或独立 PLC 命令，PLC 节点随标准启动运行。本记录不授权使能或运动。

## 结论与冻结事实

- F1: EtherCAT application_time 的初始化与运行必须使用同一连续时基；本实现固定为
  CLOCK_MONOTONIC 原始 uptime 纳秒。CLI 的日历显示不能解释为真实日期。
- F2: 统一时基已修正确定的转换错误，但本组三次实测不能支持启动耗时或 X503 瞬态已解决。

## 遗留

后续分别验证初始偏移读数配对和启动收发线程的实时调度。三次暖启动不是可靠性或
p95 验收；未执行运动、长时间负载试验、Docker 镜像构建或容器启动验收。
