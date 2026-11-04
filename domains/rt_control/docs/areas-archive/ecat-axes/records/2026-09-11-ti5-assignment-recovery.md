---
id: ecat-axes-20260911-01
area: ecat-axes
title: Ti5默认PDO分配与安装配置不一致导致启动失败，恢复后保持使能交付
date: 2026-09-11
type: corrective
trigger: 用户反馈oneclick启动失败并明确要求使能状态交付
commits: [fix/electri-97-ti5-startup-assignment]
env: native
risk: T3
writes: { reset: no, enable: yes, motion: no, plc: yes }
verified: PARTIAL
evidence: [/home/kkozia/research_reviews/electri97-ti5-recovery-20260911]
supersedes: []
related: [ELECTRI-97, ecat-axes-20260910-01]
---

## 背景

17:05用户启动在70秒OP/WKC门禁超时后报multiple ros2_control_node。
内核更早已确认Ti5位置2/3/8/9当前分配1600、要求1601，PreservePdoConfig拒绝配置。
直接SDO读回1C12/1C13确为1600/1A00；1601/1A01的两个位置映射条目仍正确。
该路径未由此前两轮暖启动覆盖，不能将暖启动结果作为掉电后配置恢复保证。

17:31安装目录中四个Ti5 profile曾改成1600/1A00完整PDO，而源码仍是1601/1A01。
本次只读源码的初始检查漏掉此差异；恢复驱动并rescan后，17:55启动反向出现
“当前1601、软件要求1600”，domain从475变为539字节、WKC36/48，再次失败。
这证明必须核对实际安装配置。原安装文件已备份，按用户指定方案恢复与源码一致。

## 改动

- 新增Native启动前Ti5准备工具，先核对安装profile、18 PREOP/主站idle、14轴
  失能、Ti5 SII身份、两个只读位置映射及已知分配。未知内容一律拒绝，不启动控制器。
- 已知1600/1A00默认分配只写1C12/1C13选择已验证1601/1A01；不写160x/1Axx映射
  条目，不写非易失保存对象。之后ethercat rescan刷新IgH缓存，逐项核对驱动与缓存。
  正确暖态仅检查，不写也不rescan；检查与写入在启动授权确认之后。
- 程序实际使用安装profile，安装文件不符合固定方案时在写设备之前报错。
- 进程发现改为精确匹配argv可执行文件与/proc/exe，避免apport/debugger的参数
  带有控制器路径时被误认。仍拒绝两个真实控制器；未捕获原第二PID的身份，
  因此仅证明旧匹配规则有误判可能，不声称原PID已确认为apport。
- 不改驱动库、1kHz/DC参数、PDO条目或使能状态机。工具与配置副本已部署至工控机。

## 验证

83项专项测试、288项完整质量门禁和ShellCheck通过。覆盖未知映射无写入、只读模式
无写入、赋值失败不扫描/不启动、缓存刷新、暖态无写入，以及两个真实控制器仍拒绝。
未重建Docker或独立Mock；本次新增逻辑仅用于Native入口，不宣称main发布验收。

实机从实际默认分配恢复后，SDO与IgH缓存都读回1601/1A01，恢复步骤没有关闭
PreservePdoConfig。第一次启动失败原因是上述安装profile差异，门禁仍生效；原
失败日志保留，最后一次oneclick在修复安装profile后返回RECOVERED/running and enabled。
/rt/enable返回ok=true/stage=success。未调用/rt/reset_fault；CANopen驱动在其原有
启动流程中执行内部Fault reset，未发送独立故障复位、FJT或速度命令。

18:00:19启动：激活到OP **19.121073212秒**，位置2出现一条5秒DC等待，启动1条
UNMATCHED，18:01:07又有6个UNMATCHED；控制器服务list_controllers有一次10秒重试。
这次不能沿用前两次6秒/零告警结论，也不将该范围标记为彻底解决。

18:02:13最后读回：14个EtherCAT运动轴均Operation Enabled、故障位0，Domain475字节、
WKC48/48，全部配置从站OP，Hub13按设计PREOP。标准oneclick已通过整组使能门禁，
用户要求保持使能交付，未在验证后停止。此后修改或停机需以新鲜状态为准。
本次实机操作经用户明确授权，确认入口：rt_control_native_oneclick.sh --ros-domain-id 7，
RT_CONTROL_NATIVE_MONITOR=false；未下发运动。

## 结论与冻结事实

- F1: 该次不能启动的首要原因是实际PDO分配与软件配置不符，不是简单重复启动。
- F2: SDO分配恢复后需刷新IgH缓存，且启动前必须验证实际安装profile。
- F3: 修复后已使能交付；本轮仍有DC等待及UNMATCHED，长期与掉电恢复重复性未验收。

## 遗留

需要掉电后重复恢复验证、缩短启动失败上报延迟、进一步定位DC等待和服务查询重试。
本次工具修复随 X503 PREOP 初始化 PR 以独立提交整理；实际安装profile原副本保留在目标备份目录。

后续只读溯源确认：17:31的四份安装profile与目标
`x503b-pdo-review-20260911.eG33tx`候选的哈希逐项一致，该目录部署脚本和
`production-backup-1_svwrod/manifest.json`记录了将源码符号链接替换为普通候选文件，
同时替换CiA402安装库。当前安装库哈希亦与该候选一致。可确认来源包与操作机制，
不能仅由文件确认具体操作人。17:31部署晚于用户17:05失败，只解释本次17:55重试的
反向冲突；驱动更早返回1600/1A00的原因仍未证实。溯源材料在evidence/provenance。
