---
id: realtime-host-20260908-02
area: realtime-host
title: ELECTRI-97 DC 偏移配对与启动收发线程调度修正
date: 2026-09-08
type: investigation
trigger: ELECTRI-97 / 用户授权继续 SSH 实机启动对照
commits: [fix/electri-97-startup-realtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PARTIAL
evidence: [/home/kkozia/research_reviews/electri97-gen2-20260908/offset-paired, /home/kkozia/research_reviews/electri97-gen2-20260908/startup-scope]
supersedes: []
related: [realtime-host-20260908-01, ecat-axes-20260908-01, ELECTRI-97]
---

## 背景

统一 application_time 后仍有启动等待。IgH 初始偏移计算把此前读到的 0x0910 与处理
响应时的最新 application_time 比较，再用 jiffies 补偿；处理延迟与应用更新时间不一致
时会引入偏差。另一个实测遗漏是，初始化收发在 ros2_control_node 主线程执行；正常
FIFO80 更新线程直到硬件激活完成后才创建，提前等待/绑定它不能覆盖初始化循环。

## 改动

- IgH `0002-dc-offset-use-sent-application-time.patch` 为每个已发送报文保存 application_time，
  0x0910 响应按同一报文的时间计算 32/64 位偏移，去掉处理时刻/jiffies 补偿。
- Host、Native 与 Docker 构建链记录并校验 `IGH_DC_OFFSET_PATCH_SHA256`，Native 同时
  核对已加载模块 Build ID。IgH frozen commit、PDO 保留补丁及 hrtimer 构建保持不变。
- ICube `0009-scope-startup-realtime-scheduling.patch` 在主站激活、OP 等待和 preload
  周期内临时设置调用线程 CPU/FIFO，成功和异常路径恢复原调度及亲和性。
- Native 从现有已核实的配置传入 `RT_CONTROL_ECAT_STARTUP_CPU=14`、
  `RT_CONTROL_ECAT_STARTUP_PRIORITY=80`；更新线程选择器排除进程主线程，避免把
  启动阶段的临时 FIFO80 错认成正常更新线程。两个环境变量均未提供时保持上游行为；
  只提供一个或提供无效值则拒绝激活。Docker 仅注册编译补丁，本次未配置或验证其启动环境。
- 每一组只增加上述一个行为改动。控制/SYNC0 周期始终 4 ms，初始偏移修正触发值
  1 ms、DC 接受差值 10 us、等待上限 5 s 及 PDO 内容均未修改。

## 验证

偏移算法的实际 C 函数编译回归：读数与发送时间本来一致、最新应用时间推进 4 ms、
处理间隔 6 ms 时，旧函数把 35 ms 偏移错误改为 33 ms；配对修正后保持 35 ms。
同时覆盖 32/64 位、正负偏移、容差内及 32 位回绕。临时调度的实际 C++ 头文件通过
成功恢复、异常恢复、FIFO 配置失败回滚、参数缺失/越界测试；线程选择回归复现并修正
主线程误识别。本地质量门禁 233 项通过，门禁工具覆盖率 83%；ShellCheck 未安装。

目标机器内核 `6.8.1-1057-realtime` 上，IgH userspace/modules 编译成功；安装过程备份
三个旧模块、依赖身份文件和启动入口，失败时恢复。新 ec_master SHA256：
`3720b496e782bb4b1bd7ed103bc6626a030e27f8c3e71cf1c54557c4f432859c`；
已加载 Build ID：`5cd310baa88d9ebccba4a8026204ef9284e62ec1`。
驱动通过正常 colcon 重编译、21 项原有参数测试及实际头文件回归，新库 SHA256：
`2cbaa7fc6f92a93ba5190ecb2b893aa94b1f3d202b6db955d50fe590c4d12103`。

| 阶段 | 轮次 | 激活至首次全 OP/WKC | DC wait 警告 | UNMATCHED | TIMED OUT | lost 增量 | 稳定后8节点 DC 最大绝对值 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 偏移配对 | 1 | 46.444146034 s | 5 | 0 | 0 | 0 | 1598 ns |
| 偏移配对 | 2 | 18.404148114 s | 1 | 1 | 0 | 0 | 2119 ns |
| 偏移配对 | 3 | 16.164149935 s | 0 | 1 | 0 | 0 | 839 ns |
| 再加启动临时 RT | 1 | 57.284028356 s | 6 | 1 | 0 | 0 | 1237 ns |
| 再加启动临时 RT | 2 | 16.644029325 s | 1 | 1 | 0 | 0 | 420 ns |
| 再加启动临时 RT | 3 | 14.164027228 s | 0 | 1 | 0 | 0 | 349 ns |

上述各轮均通过 Native READY 及后续 15 秒完整 OP/WKC 采样，两台力传感器每轮仍各有
AL 0x001A 后恢复。Hub 13 的 PREOP 是已知期望状态，不计为运行异常。额外只读 DC
采样仅在临时 RT 第2轮开启，比较耗时时必须保留这一观测条件差异。

实测启动临时 RT 之前，PID/TID 196109 以 TS 在 CPU16 执行 hrtimer_nanosleep，
正常 FIFO80 线程在首次 OP 后才出现。补丁后 PID/TID 211354 在启动中为 CPU14/FIFO80，
其他 ROS 线程仍为 TS；正常控制线程创建前主线程已恢复。

第2轮的 1 Hz 只读 DC 抽查显示，参考节点误差从约 86.7 us 逐步减小；节点1/2 有
约 65 us 超调后收敛，传感器14 从约 -267.7 us 经过正向超调后收敛。不能把这条采样
轨迹当成每周期峰值或初始偏移函数输入。该轮 scope restored 时间为
1788881767.319661676，正常 FIFO80 创建为 1788881767.388158749，相差 68.497073 ms。
这证明两个用户态循环之间存在调度交接空档；尚未以连续报文时间戳证明其全部影响。

最终 Native stopped、EtherCAT Idle/Inactive、18 个从站全 PREOP；14 个运动轴 0x6041
均成功读回，Operation enabled 全部为 false。位置2/3/8/9的 Ti5 故障仍锁存，最后
`/rt/disable` 返回 right_joint2 / fault_requires_reset / 0x1208，因此 stop 返回
UNCLEAN_SHUTDOWN，不能报告干净停机。证据为 `startup-scope/final/final-state.json`。
新模块及两份库保留在目标 Native 环境，备份分别位于目标
`rt-control-backups/electri97-offset-20260908` 和
`rt-control-backups/electri97-startup-scope-20260908`；本轮无提交、推送或 Linear 写入。

本次实机操作经用户会话授权，使用 Native start/stop 确认入口及受控模块重载；没有
调用 reset、enable 或运动接口。PLC 节点随标准启动运行，无独立 PLC 写命令。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 初始 DC 偏移应将 0x0910 读数与同一发送报文的 application_time 配对，处理时刻
  不应改变已取得读数的比较基准。
- F2: Native 初始化收发由激活调用线程执行，正常 FIFO80 更新线程尚不存在；初始化
  周期必须单独覆盖实时配置，随后恢复调用线程，避免后续线程继承临时调度。
- F3: 目前实测仍有长启动及 X503 短暂失步。修复确定的代码问题不等于 ELECTRI-97 已关闭。

## 遗留

初始偏移 1 ms 修正触发值与 DC 10 us 接受值之间的误差是否主要依靠慢速滤波收敛，
需要记录初始偏移输入并进行单变量验证；启动/正常控制循环约 68.5 ms 交接空档与
X503 0x001A 的因果关系需要连续发送时间戳取证。不要为缩短表面耗时放宽 OP/DC 门禁。
本次未执行新版本使能、运动、长时间负载、Docker 构建/启动、冷上电或断链恢复验收。
