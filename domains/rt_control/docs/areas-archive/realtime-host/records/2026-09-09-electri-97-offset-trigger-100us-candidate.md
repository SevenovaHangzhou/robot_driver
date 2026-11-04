---
id: realtime-host-20260909-01
area: realtime-host
title: ELECTRI-97 初始偏移100us候选三轮失能对照，结束恢复1ms基线
date: 2026-09-09
type: investigation
trigger: 用户要求继续初始偏移修正触发值的单变量对照
commits: [fix/electri-97-startup-realtime]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [/home/kkozia/research_reviews/electri97-offset-trigger-20260909]
supersedes: []
related: [ELECTRI-97, ecat-axes-20260909-02]
---

## 背景

阶段计时已区分约11.5秒非DC配置开销及波动的DC收敛。上次第2轮的部分初始偏差为
0.75至0.83 ms，未触发1 ms修正；第1/3轮即使全部修改offset仍有DC等待。本次准备
100 us初始修正触发值候选，测试这一个因素，不把它当作全部根因。

初次连接时EtherCAT Link DOWN、Slaves 0，先完成离线验证。用户要求继续后，链路已恢复
原18位拓扑，14个运动轴读回均失能且故障位未置位，随后完成1轮基线、3轮候选和1轮
恢复基线的失能启动采样。核心问题是：降低初始offset修正阈值是否实际改变修正决策，
并可重复地缩短DC等待，而不只是遇到不同的初始时钟状态。

## 改动

- 新增 `patches/igh/experimental/0003-dc-offset-trigger-100us.patch`，仅将
  fsm_master.c 的 EC_SYSTEM_TIME_TOLERANCE_NS 从1,000,000 ns改为100,000 ns。
- 保持10,000 ns DC接受条件、5,000 ms等待上限、4 ms周期、PDO、SYNC0 shift及连续
  交接实现。实验补丁不进入默认宿主/Docker安装或Native运行门禁。
- 新增实际C函数回归，覆盖实测正负亚毫秒偏差、100 us边界、32位时间回绕及偏移下溢。
  原偏移配对回归改为从被测源码读取实际宏值，避免用测试内写死的1 ms替代候选值。
- 从同一冻结提交及已部署的两个IgH补丁构建候选模块，保留hrtimer和ec_igb构建选项。
  先保存三个已安装模块及dependency-versions.env，再在RT-Control停止、失能读回后
  切换候选；结束恢复该备份。两次切换均验证磁盘哈希及已加载Build ID。

## 验证

RED：旧1 ms实际函数对+808100 ns偏差保留50000 ns旧offset，未达到候选要求。
GREEN：候选实际32/64位函数通过14组实测/边界用例及原配对回归；共6项相关测试通过。
本地质量门禁266项通过。冻结提交 `2f7f884f1c7d377c02a7d627eb06512126a0e50e`
顺序应用0001、0002及实验0003后构建成功，编译器gcc-12，目标内核6.8.1-1057-realtime。

离线回放上次54条初始偏移输入，旧函数输出全部匹配当时实测结果；候选输出如下：

| 原采样轮次 | 1 ms修改offset的节点数 | 100 us候选修改节点数 | 与旧结果不同的位置 |
| --- | --- | --- | --- |
| 1 | 18/18 | 18/18 | 无 |
| 2 | 0/18 | 17/18 | 0..16；position17的-41.723 us仍保留旧值 |
| 3 | 18/18 | 18/18 | 无 |

回放只证明给定输入下的计算和修正决策，不模拟DC闭环，也不能预测实机收敛时间。

候选位于目标 `rt-control-backups/electri97-offset-trigger-20260909/build`：

- ec_master SHA256: `a3ceeb7cddd16593ac63e6cf526ceb3ea51f3d2c3a637a7ddc2900c1999858ec`
- ec_master Build ID: `f670f7faf21bd5265a2c22351c1e39a55b63d3ff`
- ec_igb SHA256: `7df7dc75d57691b99ba71eb7842e3d03bc48d12d7b0073bc37f8cf20e68cc3b6`
- ec_generic SHA256: `177667d1c7163d9b5334cf1a7035047e2d8ad22490b3f94e8a46ddb6c274111e`

### 实机对照

目标robot-ipc，Ubuntu内核6.8.1-1057-realtime，IgH/ec_igb，原18从站、14运动轴及两只
X503。控制/SYNC0周期仍为4 ms，控制线程CPU14/FIFO80，EtherCAT-OP FIFO79。
保持现有hrtimer、发送时间配对和连续交接补丁。每轮等待全OP后至少观察15秒，并读取
0、1、2、4、14、15、16、17号0x092c；该离散读回不代表全程DC最大偏差。

| 轮次 | 全OP/s（应用日志） | DC阶段/s | 非DC/s（内核） | 初始offset修改数 | 5秒DC告警数 | 距上轮释放/s |
| --- | --- | --- | --- | --- | --- | --- |
| 基线A1：1 ms | 22.644 | 10.552 | 12.077 | 18/18 | 1 | 链路恢复后首次启动，未定义同类间隔 |
| 候选B1：100 us | 17.684 | 6.153 | 11.500 | 18/18 | 0 | 681.426 |
| 候选B2：100 us | 15.084 | 3.536 | 11.508 | 4/18 | 0 | 130.946 |
| 候选B3：100 us | 16.244 | 4.712 | 11.501 | 3/18 | 0 | 106.190 |
| 恢复A2：1 ms | 25.524 | 13.982 | 11.503 | 0/18 | 1 | 136.122 |

全OP时间从应用激活日志到所有配置从站OP日志；内核非DC时间为激活至最后配置结束
减dc_sync_check累计时间，因此两列相加与应用全OP时间有约15至40 ms差异。
停机间隔设定为至少60秒，表中为内核Released至下次activate的实际值，并非等间隔实验。

- B1初始偏差为+2.060至+16.799 ms，旧阈值也会全部修正，不能把该轮加速归因于100 us。
- B2在0、5、6、14号触发100 us至1 ms区间修正，其余保留旧offset；B3对应0、13、15号。
  所有实测修正决策均符合当前加载版本的阈值。
- A2初始偏差为-317.086至-62.373 us，1 ms版本全部不修正；其中0、2、3、5、14、15、17号
  落入候选会修正的区间。其DC等待主要在1、2、3号，分别4.496、5.000、4.384秒；
  两只X503的dc_sync_check各约7至8 ms，没有成为该轮的主要等待节点。
- 五轮WKC均48/48，两只X503在采样中保持OP，未见0x001A或其他传感器错误日志。
  hub13保持预期PREOP，不应把“全OP”理解为18个物理节点都需OP。
- 五轮lost frames窗口增量均0。B2/B3各有1条UNMATCHED，B1/A2无此告警。
  A1激活时有“994 datagrams TIMED OUT!”汇总及1条UNMATCHED；其lost frames为390到390。
  该汇总可能含先前链路离线期间的统计，来源未证实，不能当作候选消除994次丢帧的证据。
- 五轮短窗口发送记录最大间隔4.305至4.332 ms，startup到maintenance为3.988至4.016 ms，
  maintenance到control为1.161至3.698 ms，未复现此前约70 ms交接断档。记录为末段环形
  缓冲及256次正常write，不覆盖整个启动或长期运行。
- 五份内核trace完整、无记录丢失。B1启动前一次SSH监控查询超时，控制器尚未启动，
  旧记录保存在live-candidate/prestart-monitor-timeout，重启监控后正常完成B1。

五次受控停止均返回already_disabled。最终RT-Control stopped、主站Idle/Inactive、
18从站PREOP，14轴0x6041的operation-enabled及fault位均为false；所有采集探针已移除。
已恢复ec_master SHA256 `3720b496e782bb4b1bd7ed103bc6626a030e27f8c3e71cf1c54557c4f432859c`，
Build ID `5cd310baa88d9ebccba4a8026204ef9284e62ec1`，原ec_igb/ec_generic及元数据一并恢复。
两份应用库哈希保持原值。本轮没有发送使能、运动、复位或PLC命令。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 100 us候选只改变初始offset修正触发值，默认部署仍保留1 ms版本。
- F2: 候选三轮失能启动全OP为15.084至17.684秒、无DC等待超时，B2/B3实际改变4/3个
  节点的修正决策；恢复基线为25.524秒。存在改善迹象，但初始偏差、符号、停机间隔
  和模块重载条件不同，小样本不足以证明稳定因果关系或全部解决启动慢。
- F3: PASS仅指离线计算/构建及本次受控失能启动验证。未推广至默认部署，现场已恢复
  1 ms基线并确认停止、失能。启用工况、运行中重配和长期性能仍为UNVERIFIED。

## 遗留与回退

本次切换前备份为目标实验目录的live-baseline-modules.tar.gz，已完成恢复；不要使用
早期非hrtimer备份。原始数据在trace-{baseline,candidate}-N及live-{baseline,candidate}
目录，汇总为live-summary.json，最终状态为final-live/final-state.json。

下一步性能验证应增加往返对照次数、固定从Released到activate的实际间隔，并按初始
偏差大小和符号分组；保留每节点DC曲线，尤其1至3号及参考时钟0号。非DC约11.5秒
（PDO约4.225秒、OP状态转换约4.57秒）需独立分析，调初始阈值不会消除这些配置阶段。
本次没有验证1 kHz、运行中重新配置、使能、运动、长期负载或Docker，也未提交、推送
或写入Linear。
