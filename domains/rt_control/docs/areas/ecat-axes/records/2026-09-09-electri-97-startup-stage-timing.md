---
id: ecat-axes-20260909-02
area: ecat-axes
title: ELECTRI-97 分阶段计时区分固定配置开销和 DC 收敛波动
date: 2026-09-09
type: investigation
trigger: 用户批准先增加各从站启动阶段计时及初始偏移取证
commits: [fix/electri-97-startup-realtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PASS
evidence: [/home/kkozia/research_reviews/electri97-stages-20260909]
supersedes: []
related: [ELECTRI-97, ecat-axes-20260909-01, realtime-host-20260908-02]
---

## 背景

连续收发交接修正后，双 X503 的启动后 0x001A 已在三轮暖启动中消失，但首次全 OP
仍耗时15至23秒。本次只量化各阶段耗时；不修改4 ms周期、PDO、SYNC0 shift、偏移
算法或同步阈值，也不替换应用库和内核模块。

## 改动

新增宿主诊断工具 `tools/igh_startup_trace.py` 与26项测试。工具依据已加载模块 Build ID
核对未剥离调试信息的 ec_master.ko，再从 DWARF 获取字段偏移及尺寸，从 kallsyms
获取状态函数地址；字段或身份不匹配时拒绝采集，不硬编码结构体偏移。

使用 [Linux 6.8 kprobe/kretprobe](https://www.kernel.org/doc/html/v6.8/trace/kprobetrace.html)
在独立的 e97_* tracefs instance 中记录配置状态、执行返回、DC差值、初始偏移函数
输入/输出及阻塞SDO调用。常规退出、异常和信号均清理本次注册的探针及instance，
不清空全局跟踪或他人探针。首版试采因 Python 追加模式尝试 SEEK_END 被 tracefs 拒绝，
已用不执行seek的底层写入修正并增加回归；失败试采均发生在主站停止时。

状态发生变化时，将前一个配置执行调用的返回时刻作为新阶段起点，避免将下一次
轮询间隔错误归入上一阶段。阶段包括排队、报文往返、等待和主站调度时间，不等于
设备固件的纯计算时间；文本跟踪时间精度为微秒。无法配对返回、未完成配置、未知
状态、缓冲区丢失或探针漏采均显式标记分析失败。

工具输出 metadata.json、trace.txt、analysis.json 和逐从站 stages.csv。初始偏移
按32/64位和符号解码，并核对返回的新偏移是否符合捕获的输入；0x092c按符号-幅值
解码。阻塞SDO调用在第2/3轮加入，第1轮该项标记未测，不能记为零。

复现时先让主站处于Inactive，用匹配的未剥离模块开启采集，出现ARMED后通过已授权的
Native入口启动。工具自身不启动总线，也不调用任何使能接口：

```bash
sudo python3 tools/igh_startup_trace.py capture \
  --module /path/to/matching/ec_master.ko --output /path/to/new-capture --seconds 180
python3 tools/igh_startup_trace.py analyze /path/to/new-capture
```

## 验证

26项工具测试通过，工具行覆盖率91%；仓库质量门禁263项通过，规定的仓库门禁工具
覆盖率83%。本机未安装ShellCheck。本轮不修改C++、构建配置或运行库，不需要重新
构建运行包；没有Docker、使能、运动或长期负载验证。

现机为 x86_64 / 6.8.1-1057-realtime，ec_master Build ID仍为
`5cd310baa88d9ebccba4a8026204ef9284e62ec1`，应用库与上一条连续交接记录相同。
三轮分别完整捕获17个进入OP的配置流程（0..12、14..17）及18个DC时钟的初始偏移。
Hub13按既有策略保持PREOP。三轮无trace buffer overrun或kretprobe missed hit。

| 轮次 | 内核激活至最后配置完成 | DC收敛累计 | PDO配置累计 | 进入OP累计 | 非DC总计 | 应用首次全OP/WKC |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 17.332314 s | 5.832879 s | 4.224844 s | 4.567972 s | 11.499435 s | 17.364028196 s |
| 2 | 32.749083 s | 21.248409 s | 4.222800 s | 4.566970 s | 11.500674 s | 32.764046944 s |
| 3 | 28.245148 s | 16.743789 s | 4.225214 s | 4.569765 s | 11.501359 s | 28.284058523 s |

非DC总计包含PDO配置、进入OP和其他初始化/主站开销，不能与这些分项重复相加。
第2/3轮激活前均有38次阻塞SDO下载，累计分别151.680/152.541 ms，不包含在表内
“内核激活至最后配置完成”时间中。

PDO阶段：9个ZeroErr节点每个约384 ms，4个Ti5节点每个约192 ms；两台X503和
Updown没有进入PDO重配状态。Ti5仍有只读映射写入abort，其当前和目标映射在日志中
一致。本次只记录，不改变映射策略或屏蔽告警。进入OP阶段除Hub0约16 ms外，其余
16个节点每个约280至288 ms；还不能将这一等待全部归因于固件固定延时。

DC阶段的重要差异：

- 第1轮18个时钟初始偏差约-106至-121 ms，均修改offset值；DC等待主要位于
  position4（2.161 s）、8（1.592 s）、7（1.264 s）、1（0.720 s），无5秒超时。
- 第2轮18个时钟均保留旧offset值；运动从站的初始偏差多在+0.746至+0.830 ms，
  未达到现有1 ms修正触发值。position1/2/4各等待约5秒，position5仍耗时4.551 s、
  position3耗时1.608 s，说明没有超时告警不代表没有显著DC等待。
- 第3轮18个时钟初始偏差约-9.35至-11.96 ms，均修改offset值，仍有position8和14
  各约5秒等待。position8的0x092c从-15.338 us变为-53.598 us，position14从
  -13.759 us变为-10.433 us；这些已执行初始修正后的持续同步问题不能仅用1 ms
  触发值解释。

三轮均通过Native READY及后续至少15秒OP/WKC采样，未记录双X503的0x001A。
交接窗口最大发送调用间隔分别4.039893/4.011540/4.022118 ms，接管后256周期的
WKC均完整。探针带来观测开销，不能把有探针/无探针的耗时差直接解释为参数效果。

最终RT-Control停止、主站Idle/Inactive、18位全PREOP；14个运动轴0x6041的使能位
全为0。Ti5位置2/3/8/9故障仍锁存，最终stop返回fault_requires_reset和
UNCLEAN_SHUTDOWN，未报告干净停机。所有本次e97_*探针和instance均已移除。

本次实机操作经用户会话授权，采用Native start/stop确认入口，诊断工具单独经sudo
配置跟踪。未调用reset、enable或运动接口；PLC节点随标准启动运行，无独立PLC命令。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 在当前18位拓扑、4 ms周期和当前构建下，三轮非DC配置开销约11.5秒，其中
  PDO约4.22秒、进入OP约4.57秒；本样本启动耗时波动主要位于DC收敛阶段。
- F2: 激活前38次阻塞SDO下载约0.15秒，不是本样本15至33秒启动耗时的主要部分。
- F3: 第2轮确实出现亚毫秒初始偏差保留旧offset值的分支；第1/3轮均修改offset仍有
  不同程度DC等待。因此1 ms触发值是可验证因素，尚不是全部根因。
- F4: 分阶段计时完成不等于启动耗时已修复；本轮没有改变任何同步参数或运行二进制。

## 遗留

下一步将初始偏移触发值与持续DC同步分别做单变量对照，并记录/控制停机间隔。
已经修正offset的轮次仍可能缓慢收敛，需要进一步核对参考时钟与周期同步行为。
PDO配置和OP过渡另有稳定开销，可分别评估，但不能绕过映射校验或降低同步接受标准。
没有提交、推送或Linear写入。
