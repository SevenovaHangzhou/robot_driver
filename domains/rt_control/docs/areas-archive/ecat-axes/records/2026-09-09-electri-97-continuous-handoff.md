---
id: ecat-axes-20260909-01
area: ecat-axes
title: ELECTRI-97 初始化至正常循环保持周期收发连续
date: 2026-09-09
type: fix
trigger: 用户明确要求先修正初始化到正常运行之间的周期收发连续性
commits: [fix/electri-97-startup-realtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PARTIAL
evidence: [/home/kkozia/research_reviews/electri97-handoff-20260909]
supersedes: []
related: [ELECTRI-97, BQ-114, realtime-host-20260908-02]
---

## 背景

前次记录从日志确认初始化循环退出到正常更新线程创建约有 68.5 ms 间隔，但未直接
量化发送调用间隔。核对目标 ros2_control 2.54.0 的源码后，set_component_state 与
read/write 共同持有 resources_lock_，简单延迟硬件激活会阻塞正常更新循环，不能据
硬件激活服务存在就宣称无缝交接。本次在 EtherCAT 驱动内保持连续收发，不修改 CM。

## 改动

- 新增 ICube `0010-maintain-cyclic-handoff.patch`，在完整 OP/WKC 和 raw position preload
  完成后，以短期 `ecat-handoff` 线程接续同一 CLOCK_MONOTONIC 绝对周期。
- `EcMaster::maintainData()` 只收发现有 PDO image、刷新内部总线状态并同步 DC，不调用
  processData/onPdoCycleSent，不更新导出的状态或目标值。失败返回到所有权状态机，
  后续 read/write 返回硬件 ERROR，不授予正常收发权限。
- 无锁原子状态保证维护线程和正常循环互斥访问主站。read 在维护周期繁忙时只请求
  接管；维护线程完成该周期并释放后，由后续 read 获取所有权。write 必须先看到该
  所有权，不能在一次被跳过的 read 后抢先发送。
- 正常 read/write 不 sleep、不 join、不等待互斥锁。取消和 join 只发生在非实时激活
  失败、停用或析构路径，并先于主站 deactivate/release。维护等待沿用已有
  preload_timeout_ms=5000 的有界期限；延迟唤醒跳过过期节拍，不突发补发。
- 维护线程先命名，再配置已有 CPU14/FIFO80；Native 线程选择器等待该临时线程退出，
  然后绑定正常更新线程及 CANopen loop。没有改动 FIFO79 内核线程的设置。
- Native 默认 `RT_CONTROL_ECAT_CONTINUOUS_HANDOFF=1`；只为受控对照保留 `0` 路径。
  库在变量未提供时保持既有行为。Docker 注册同一补丁，但本次未配置其运行环境。
- 1024 项预分配发送记录环缓存保存 CLOCK_MONOTONIC 时间、阶段、WKC，在正常循环
  256 次发送后冻结，停机后才导出 CSV。时间戳位于用户态 ecrt_master_send 调用前，
  不是网卡硬件发包时间戳。Native 将文件写到当次日志旁的 `-ecat-send.csv`。
- 全程保持 4 ms 控制/SYNC0 周期、shift、PDO、DC offset 算法和同步接受阈值。

## 验证

离线：6 项所有权/竞争/取消/失败测试、2 项有界发送记录测试、2 项实际 EcMaster
伪传输测试，以及目标原有 5 项 PDO、21 项参数测试通过。隔离构建的 colcon 汇总为
41 项（包含 CTest 包装结果），0 failures；实际 Native 重编译的同一组 GTest 也通过。
实际工作区历史 test-result 含旧结果及 skipped 项，不将其总数解释为本轮新增测试数。
本地质量门禁 237 项通过；ASan/UBSan 通过。ThreadSanitizer 在本机因 unexpected
memory mapping 无法运行；ShellCheck 本机未安装。未执行完整上游 lint 套件。

补丁适用于 ICube `1390be742986f4e898ca112e49bb24805be9899a` 的 0001..0009 之后。
头文件布局变化的直接运行消费者只有 ethercat_driver，已与 ethercat_interface 同批
重编译；EcMasterAsync 为另一类型，不受此布局变更影响。

目标 `6.8.1-1057-realtime` 安装并核对运行进程 maps 后，库 SHA256 为：

- ethercat_interface: `da81ab964430d08ef24207e6e4b500750a137a52e9897ae6250345e60869b083`
- ethercat_driver: `1fd9c461deefe61d9b86e013b3c4e489ae2f52c13b36bb49ebe697d03eae86d6`

基线和候选使用相同库，仅切换连续维护开关。基线确认变量为0且没有维护帧：

| 组别 | 首次全 OP/WKC | 维护帧 | 交接窗口最大发送间隔 | 接管后256周期 WKC不完整数 | 两台X503 0x001A |
| --- | --- | --- | --- | --- | --- |
| 基线，关闭维护 | 23.044032885 s | 0 | 69.906416 ms | 173 | 各1次 |
| 连续维护，第1轮 | 15.284039044 s | 21 | 4.004765 ms | 0 | 0 |
| 连续维护，第2轮 | 16.644034024 s | 19 | 4.017923 ms | 0 | 0 |
| 连续维护，第3轮 | 23.204066577 s | 20 | 4.018699 ms | 0 | 0 |

第1轮 startup->maintenance 为 4.001938 ms，maintenance->control 为 1.984029 ms。
初始化最后一次发送到正常循环首次发送仍经过 85.971352 ms，但中间有21次维护发送，
说明本次修正覆盖了交接期间的通信需求。交接窗口定义为最后一个 startup 样本前两
个间隔，直到前几个 control 间隔；完整1024样本最大间隔为 4.294671 ms。

三轮候选均通过 Native READY，后续至少15秒状态采样保持预期 OP/WKC；完整发送记录
的最大间隔分别为4.294671/4.292244/4.327303 ms。三轮均无 X503 0x001A、无 TIMED OUT、
lost frames 增量0；每轮仍有一次启动 UNMATCHED。第3轮在首次 OP 前仍有一次
`Slave did not sync after 5007 ms`，证明初始 DC 等待尚未解决。

最后 Native stopped、主站 Idle/Inactive、18 位全部 PREOP，14 个运动轴0x6041的
Operation enabled 位全为0。Ti5位置2/3/8/9仍锁存原有故障，最后 `/rt/disable` 返回
right_joint2 / fault_requires_reset / 0x1208，停机为 UNCLEAN_SHUTDOWN，不能报告
故障已清除或干净停机。原始证据为 `final/final-state.json`。

当前记录只涉及不使能启动。首次 OP 发生在维护线程启动之前，因此不把首次 OP
时间变短归因于维护线程。本记录不关闭初始 DC 收敛耗时问题，也不替代长期验收。

本次实机操作经用户会话授权，采用 Native start/stop 确认入口；未调用 reset、enable
或运动命令。PLC 节点随标准启动运行，无独立 PLC 写命令。本记录不授权使能或运动。

## 结论与冻结事实

- F1: 初始化返回后的用户态发送空档实测为69.906416 ms；完整 OP/WKC 及 preload
  已完成不能替代正常周期真正接管前的持续收发。
- F2: 短期维护线程只交换已准备好的过程数据和 DC，正常 read 获得唯一所有权后再
  更新导出状态和接受正常写入；正常实时路径不通过 join/锁等待完成交接。
- F3: 发送记录是用户态调用证据，传感器 AL/WKC 是独立实机证据；两者应同时验收。
- F4: 同一构建的基线关闭维护时，交接空档69.906416 ms并复现双X503失步；开启后
  三轮交接窗口最大间隔约4.005/4.018/4.019 ms且无X503失步，完成本次T3交接验证。

## 遗留与回退

目标备份位于 `rt-control-backups/electri97-handoff-20260909`，包含原源码、两份库、
Native 启动入口和构建日志。需要回退时先确认失能并停止 RT-Control，再同批恢复两个
包及库和入口，避免新旧类布局混用。诊断开关0只用于行为对照，不等于恢复旧发行物。

初始 DC 收敛、Ti5 锁存故障仍需分别处理。新版本未做使能、运动、断链/冷上电、长期
负载或 Docker 构建/启动验证；没有提交、推送或 Linear 写入。
