---
id: ecat-axes-20260910-01
area: ecat-axes
title: ELECTRI-97 更正默认部署漏改的启动周期，修正后首次 OP 为 6.821 秒
date: 2026-09-10
type: corrective
trigger: 用户要求核查 ICube issue 154 拼写根因及类似 DC 超时；延续已批准的 1 kHz 默认部署
commits: [fix/electri-97-startup-realtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PARTIAL
evidence: [/home/kkozia/research_reviews/electri97-rate-correction-20260910]
supersedes: []
related: [ELECTRI-97, ecat-axes-20260909-03]
---

## 背景

问题的关键是实际 DC 同步报文周期是否与声明一致。此前把 32.924 秒解释为默认入口
采用不同 OP 门禁是错误的：9 月 9 日诊断脚本同样调用 Native start，统计相同的
Activated EcMaster → All configured slaves OP 日志。kernel-before.txt 是历史基线，
其中的告警不能归属于随后那次 8.951 秒测试。对比 before/after 增量确认该次启动
没有 5 秒告警；停机时的 X503 0x001A 应单独保留。

16 份实际安装 profile 的键均为 assign_activate，值均为 0x0300，没有 assign-activate。
真正遗漏是 Xacro control_frequency 仍为 250，而 controller_manager 为 1000。
13:27:10 那次发送统计清楚显示 startup/maintenance 约 4 ms，control 约 1 ms。
因此此前宣布全部 1 kHz、SYNC0 和发送周期已验证的结论不成立。

同时发现部署回退后源码与库不一致：库为含 0011 的诊断构建，部分源码已回退为
0001..0010，但两个 0011 新文件仍存在。增量构建可重用旧对象，不能证明源码一致。

## 改动

- EtherCAT Xacro control_frequency：250 → 1000。保持已批准的 controller_manager
  1000 Hz、Ti5/Updown 0x60C2=1 ms、统一 shift 0、显式发送间隔。
- 配置回归比较控制器频率、Xacro 频率、插补周期，同时检查 profile 的 DC 键。
- 核对遗留新文件内容后补齐 0011 源码，强制清理并重编 interface/driver，确保
  运行库与可复现源码一致；将已有 Native bootstrap 补丁清单同步到工控机。
- 未修改 IgH 内核、5 秒超时、10 us 接受阈值、PDO 映射或任何故障反应参数。

## 验证

回归先因 250 != 1000 失败，修正后本地/工控机配置测试均 26 项通过，包含 real/mock
Xacro 展开。目标四包构建通过；interface/driver 六个 CTest 及硬件包四个 CTest 通过。
本地质量门禁 264 passed、13 skipped；显式指定冻结源码后相关 16 项通过，覆盖这些
跳过项。门禁覆盖率 83%。完整 Docker 镜像和独立 mock 生命周期未重跑。

目标 Native 14:37:43 会话：主站激活 1789022266.414876028，全配置从站 OP/WKC 完整
1789022273.235947528，差值 **6.821071500 秒**。未设置诊断定时激活。
该次没有 Slave did not sync after 5000 ms，启动有一条 UNMATCHED。
实际读回 16 个 DC 从站 0x0980=0x0300、SYNC0 周期均 1000000 ns；起始时间模周期
均为 379254 ns，即相对 IgH dc_ref_time 同相。shift=0 不要求对齐 epoch 零点。
初版验证脚本误断言余数为 0，已按冻结 IgH 源码修正，并对同一保存的数据离线复核。

发送统计：startup 6822 次、间隔 0.830214–1.173947 ms；maintenance 81 次；control
57735 次，间隔 0.661561–1.279471 ms，control WKC 不完整 0。
启动 WKC 在各节点进入 OP 前不完整属于记录到的状态转换，不能计作稳态健康样本。
单次逐节点 DC 读回最大绝对偏差 329 ns，仅覆盖 16 个配置节点的离散读取。

受控 stop 返回 already_disabled/STOPPED。最终 18 PREOP、主站 Idle/inactive、
14 运动轴使能位及故障位均 0、无遗留控制进程。Ti5 1601/1A01 分配保持。
停机阶段 14:38:51 的两个 X503 仍各出现一条 0x001A；不能据此宣布完整生命周期
无同步错误，启动/稳态与停机的证据应分开。
修正后的 Native 默认配置保留。无使能、运动、复位、提交、推送或外部消息。
沿用用户已批准的 1 kHz 部署及失能启停，使用标准 Native 确认入口；PLC 随入口运行。
本记录不授权使能或运动。

用户随后要求再测一次，18:23:06 按原默认配置完成补测。主站激活至 OP
**6.381047431 秒**，Native READY 通过；16 个 DC 节点读回 1 ms 同相、WKC48/48，
控制段 158150 次发送的 WKC 不完整为 0，最大发送间隔 1.341237 ms。
无 5 秒 DC 告警或 TIMED OUT，启动有 1 条 UNMATCHED；停机双 X503 仍报 0x001A。
stop 返回 already_disabled/STOPPED；最终主站 inactive、18 PREOP、14轴失能无故障位。
配置与两份库的前后哈希一致。原始记录在 evidence 目录的 repeat-1 子目录。

## 结论与冻结事实

- F1: 当前 profile 没有 issue 154 的 assign-activate 拼写错误。
- F2: 13:27 会话是 4 ms 启动/1 ms 运行的混合配置；不能作为完整 1 kHz 验证。
- F3: 补齐周期后两次分别 6.821、6.381 秒进入 OP，均无 5 秒 DC 告警；仅为暖启动结果。
- F4: IgH 5 秒检查在设置 SYNC0 起始时间之前读取 0x092C，超时警告后继续配置；
  缩短超时不是让时钟更快收敛，也不会自动成为 fail-closed 门禁。

## 遗留

需要冷启动/多轮统计、长期负载和授权运动跟随误差验证；不能保证所有初始时钟状态
下均无 5 秒等待。本轮时Native 0011 尚未进入Docker链，后续软件容器补齐见
[main封装记录](2026-09-10-electri-97-main-packaging.md)，实机Docker切换仍未验收。
Ti5 掉电后的 PDO 分配保持性仍未验证，不将此前手工 SDO 读回与缓存差异归因为已证实掉电。
