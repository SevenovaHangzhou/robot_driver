---
id: control-20261106-01
area: control
title: Rolling 右 J7 小幅轨迹停流后再次启动实机复测
date: 2026-11-06
type: commissioning
trigger: 用户批准逐关节 ±2° 测试，并要求在修正启动确认脚本后实物重测
authority: 用户 2026-11-06 明确授权，运动前逐段通知
commits: [e075db2]
env: native
risk: T4
writes: {reset: no, enable: yes, motion: yes, plc: no}
verified: PARTIAL
evidence:
- ar@192.168.100.40:~/rt-control-arch-20261105d/tools-local/rolling-trace-1793964452972456975.jsonl
- ar@192.168.100.40:~/rt-control-arch-20261105d/tools-local/rolling-trace-1793964481784597025.jsonl
supersedes: []
related: [control-20261105-02, control-20261105-03]
---

## 范围

A2/B2 候选部署 `~/rt-control-arch-20261105d`，scope `bench_dual_arm_head`，ROS domain 78。本次只动右 J7，其余关节与头部保持。逐段均已通知现场人员。右 J7 以每段开始的实测位置为中心：0 → +2° → −2° → 0，运动共 6 s，三段零端速三次插值，峰值 3°/s；30 Hz 发布 0.4 s Rolling 窗口，轨迹回原位后继续发送静止尾段。

脚本已补命令订阅匹配等待、首批 sequence 对应的 ACCEPTED 确认和逐批发送/状态日志。两次运行之间不切模式、不重启控制器；公共 epoch 保持 2。

## 结果

| 项目 | 第一段 | 第二段 |
| --- | --- | --- |
| 批次序号 | 1–208 | 209–416 |
| 状态回报中观察到的拒绝 | 无 | 无 |
| 首批确认延迟 | 9.6 ms | 12.9 ms |
| 首次 RUNNING 距首批发送 | 109.6 ms | 112.9 ms |
| 实测负/正峰值 | −1.9988° / +1.9933° | −1.9981° / +1.9892° |
| 回位误差 | +0.00275° | −0.00069° |
| 其他机械臂关节最大漂移 | 0.00687° | 0.00481° |
| 最后发送到 HOLDING 状态回报 | 216.5 ms | 211.2 ms |

两段均在发送期间进入 RUNNING；主动结束静止尾段发送后，再按 200 ms update timeout 回到 HOLDING。第二段 RUNNING 时 `stop_reason=STOP_UPDATE_TIMEOUT` 仍保留前一段停止原因；不能用该字段单独判断当前已超时。状态话题只给最新批次结果，因此“未观察到拒绝”不等于逐批独立确认全部接受。

测试结束 `/rt/disable` 返回 success。readiness 为 IDLE / not_enabled，errors 为空，未再次使能。

本次实机操作经现场授权，操作人：现场用户与 Agent；确认短语入口：用户“授权全部步骤，已安排……运动之前和我说一下”及“在实物上重新测试吧”。

## 结论与边界

- 右 J7 在同一 epoch 下停流后再次启动通过，未复现旧脚本 HOLDING 提前退出。
- 原失败没有逐批时间线，不能据本次通过追认原失败的唯一原因。隔离复现证明首批缺失加旧脚本计时方式能够产生同样表象。
- A2/B2 完整实机验收仍未结束：其余关节 Rolling、运动中断流制动、头部独立小幅动作与其他既定用例待完成。10/15°/s 大幅队列按用户改为小幅测试的指令暂停。
