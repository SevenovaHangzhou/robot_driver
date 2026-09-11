---
id: ecat-axes-20260909-03
area: ecat-axes
title: ELECTRI-97 Ti5保留已校验PDO并传播失败，三轮配置阶段减少约0.77秒
date: 2026-09-09
type: fix
trigger: 用户批准继续执行Ti5 PDO保留验证，减少重复配置开销
commits: [fix/electri-97-startup-realtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: yes }
verified: PASS
evidence: [/home/kkozia/research_reviews/electri97-ti5-pdo-20260909]
supersedes: []
related: [ELECTRI-97, ecat-axes-20260909-02, realtime-host-20260909-01, ecat-axes-20260902-01]
---

## 背景

阶段计时显示四个Ti5每次尝试改写只读0x1601/0x1A01，各消耗约192 ms并产生两次
0x06010002 abort。日志中的当前映射与目标一致。核心问题是能否通过保留已校验布局
减少配置往返，并确保不一致的布局不能继续启动。

准备过程中确认原PreservePdoConfig仅使PDO子状态机失败，外层fsm_slave_config仍会
打印告警后进入watchdog配置，未兑现“拒绝不一致”的完整错误传播语义。该问题也影响
已经使用保留标志的X503，必须在启用Ti5保留配置之前修正。

## 改动

- 修正 `patches/igh/0001-preserve-verified-pdo-config.patch`：将已有保留标志查询函数
  声明移到fsm_pdo.h；保留模式的PDO配置失败时，外层设置slave error_flag并进入
  config error终态。未设置或关闭标志时保持原有失败处理，配置进行中及移除配置路径不变。
- 四个Ti5 profile新增 `use_slave_pdo_defaults: true`，复用ICube已有配置注册与IgH
  比较机制。映射索引、条目顺序、位宽、SDO、极性、比例、接口及CiA402控制行为不变。
- 增加抽取冻结源码真实C函数的六场景回归，覆盖保留模式失败、无标志失败、标志为0、
  成功、进行中和配置被移除；补充四份Ti5 profile的有序固定布局检查。
- 沿用冻结IgH `2f7f884f1c7d377c02a7d627eb06512126a0e50e`、0002发送时刻配对补丁、
  hrtimer/ec_igb及应用连续交接实现。DC初始阈值仍1 ms，接受条件10 us、等待上限5 s、
  控制/SYNC0周期4 ms不变。本次不启用100 us实验补丁，不重编或替换应用驱动库。

## 验证

离线RED复现：保留PDO失败后watchdog调用数为1、error_flag为0、配置FSM仍运行；
修正后该场景终止且不进入后续配置。其余五个场景通过。负例为抽取真实C函数、替换外部
依赖的离线回归，未在实机注入错误PDO。配置与功能包相关测试合计53项通过。

目标内核6.8.1-1057-realtime上，隔离目录应用补丁检查通过，gcc-12完整构建IgH用户态及
三个模块成功，构建前9项偏移配对/PDO回归通过。隔离colcon构建robot_hw_ethercat通过，
四组CTest全部通过，colcon汇总51 tests、0 errors/failures/skipped。包含real/mock Xacro
展开与接口校验，未新增独立mock控制器生命周期运行。

仓库质量门禁270项通过、2项因未指定时基源码跳过；随后指定实际ICube源码运行时基
测试文件，4项全部通过，覆盖这两项。仓库门禁工具覆盖率83%。ShellCheck本机不可用。
首次目标stage缺少身份测试依赖文件，补齐后重新运行全部9项并构建，保留初次失败日志。

实机前后均读取2、3、8、9号的SII身份及0x1C12/0x1C13、0x1601/0x1A01：
RxPDO为0x1601，内容6040:00/16、607A:00/32；TxPDO为0x1A01，内容6041:00/16、
6064:00/32。分配数量、映射数量及顺序均符合配置，未向这些映射对象独立写入。

身份附带发现：2、9号的SDO 0x1018:01返回1，而SII厂商号均为0x00522227、产品号
0x00009253，与主站配置一致；单独重读2号仍为1。身份校验按主站实际使用的SII完成，
原始SDO差异保留在preflight-layout.json，未改设备身份或profile ID，待厂商解释。

安装前保存三个模块、元数据、原补丁及四个profile。所有采样使用相同的新模块，基线
保留旧profile；之后仅切换四份Ti5保留标志，未再次重载主站。控制线程CPU14/FIFO80，
EtherCAT-OP FIFO79，现场为原18位拓扑和14运动轴。每轮全OP后至少观察15秒并读取
八个节点的0x092c。停机到下一次激活至少60秒，实际间隔如下。

| 轮次 | 应用全OP/s | 内核DC/s | 内核非DC/s | PDO配置/s | Ti5映射abort数 | 距上轮释放/s |
| --- | --- | --- | --- | --- | --- | --- |
| 基线：原Ti5配置 | 11.644 | 0.128 | 11.493 | 4.224 | 8 | 首轮，未定义同类间隔 |
| 保留第1轮 | 13.724 | 2.977 | 10.732 | 3.456 | 0 | 93.419 |
| 保留第2轮 | 16.084 | 5.314 | 10.730 | 3.453 | 0 | 84.086 |
| 保留第3轮 | 25.484 | 14.744 | 10.732 | 3.455 | 0 | 122.446 |

基线Ti5的PDO阶段分别191.939/191.950/191.764/191.889 ms；保留后三轮不再出现单独的
异步pdo_conf等待阶段，不表示同步比较函数执行时间为零。全总线PDO阶段减少
768.471/770.962/769.673 ms，非DC总计减少约0.76秒。OP转换仍约4.57秒。
应用全OP日志与内核最后配置完成不是同一时刻，两类计时约有8至40 ms差异。

四轮均通过Native READY、WKC48/48及两只X503 OP观察，无0x001A或其他传感器错误日志。
hub13按配置保持PREOP。各轮lost frames增量0，候选每轮有1条UNMATCHED，均无TIMED OUT。
四份内核trace无丢失；短窗口最大发送间隔4.287至4.336 ms，startup到maintenance
约3.996至4.001 ms，未出现原先约70 ms交接断档。这不是全程或长期抖动上限。

DC仍未闭环：基线初始偏差+20.719至+25.532 ms，18节点全部修正；三轮保留配置的初始
偏差均未超过1 ms，全部不修正，不能以全OP时间评估PDO优化。第3轮2、3号各出现5秒
DC等待告警。第1轮全OP后的离散0x092c读回中，1、2号分别为16.931、13.922 us，
说明OP/WKC正常不等于持续偏差小于10 us；本次不宣称持续DC精度验证通过。

四次受控停机均返回already_disabled并正常退出。最终RT-Control stopped、主站
Idle/Inactive、Link UP、18从站PREOP；14运动轴的operation-enabled及fault位均false。
新模块与四份保留profile留在现场，已加载身份与磁盘一致，所有本次探针已清除。

当前ec_master SHA256 `5861d61fa8537ed06b55da2d10dd6adf1d22cdba468e320a0bac0bbeb15f1144`，
Build ID `d37e9762b9595221aa12cebfaa2739ef48fe60a4`；0001补丁SHA256为
`b83b5837d3748fb7f5c15d4066e3cfc753347dd2f20f16fd0f43585d2f9a1438`。
两份应用库哈希保持上一轮原值。

本次操作沿用用户对robot-ipc受控失能启动/停止的会话授权，使用Native确认入口；
未调用reset、enable或运动，PLC节点随标准入口运行，无独立PLC命令。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: Ti5保留模式完整校验PDO映射和分配，配置失败在外层终止，原来的失败继续路径已修正。
- F2: 在本次18位/4 ms三轮失能启动中，Ti5重复映射写入abort由每次8条降为0，PDO阶段
  稳定减少约0.77秒，非DC总计由约11.49降至10.73秒。
- F3: 总启动时长和持续DC同步仍有波动；本次只通过PDO配置优化及正常失能启动验证，
  不是1 kHz、使能运动、长期运行或整体DC验收。
- F4: 新模块及profile已保留，最终停止且14轴失能、故障位均0，18从站PREOP。

## 遗留

继续固定实际停机间隔、按初始偏差及符号分组验证DC，观察参考时钟及1至3号节点。
ZeroErr约3.45秒PDO和约4.57秒OP阶段保留，不在本轮扩展优化。SDO/SII厂商号差异待厂商确认。
未进行独立mock生命周期、错误映射实机注入、1 kHz、使能、运动或长期负载测试；Docker
镜像未构建，本次为Native现场验证，不具备main发布封装验收结论。无提交、推送或Linear写入。

回退材料在目标 `rt-control-backups/electri97-ti5-pdo-20260909/live-baseline-modules.tar.gz`，
包含模块、身份元数据、原0001补丁和四份profile；在受控停止、失能读回及探针清理后，
`switch-modules.sh baseline`可整体恢复本次切换前状态并核对加载身份。单独回退profile
可用baseline-profiles目录；不能将新保留配置与旧的失败继续内核补丁作为完整验收组合。

自审：域边界与接口兼容PASS；变更仅涉及PDO保留、失败传播及验证资料；正常生命周期
T3通过，失败传播为离线C回归；无用户无关改动，无生成物或敏感信息入Git，未获提交授权。
