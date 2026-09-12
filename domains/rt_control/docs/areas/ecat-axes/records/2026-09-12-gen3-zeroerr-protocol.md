---
id: ecat-axes-20260912-05
area: ecat-axes
title: 三代机零差 CSP/PP 只读核验与限力类型和 PDO 对齐修正
date: 2026-09-12
type: fix
trigger: ELECTRI-118，用户授权继续核对 CSP/PP 的 PDO 与 SDO
commits: []
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-147, BQ-146, ecat-axes-20260912-04]
---

## 背景

物理轴序确认后，核对当前 CoE 映射和对象能力，再修正软件与现场设备的协议差异。
只读采集与源码/测试部署在已授权范围内；不执行模式切换、PDO remap 或总线启停。

## 改动

- 增加硬件所有者的协议回读快照和 CSP draft；补齐 PP draft 的候选 Rx/TxPDO 与
  初始化对象结构。所有机械换算、限位、限力与 PP 轨迹参数继续 TBD，verified=false。
- ZeroErrPpSlave 的 6072 从 uint16 改为 int16，正数上限不超过 32767；这只是类型
  边界，不是允许的物理力限值。拒绝 unsigned profile、负值和超界配置。
- 单个 PP TxPDO 在五个反馈对象之后强制追加 0000:00:uint8 dummy，13 → 14 bytes；
  dummy 不注册为周期数据通道或 ROS interface。拒绝缺失、错位、重复、错误长度与接口绑定。
- 保留 605D 的配置要求，未引入未经确认的停止兜底；生产 variants、旧机 profile、
  Robot Model、官方 Action 接口和控制字所有权未改。

## 验证

- 目标机 master/slaves 查询、16 份对象字典与 656 次限定 SDO upload 完成：640 次成功，
  16 次均为 605D 返回 0x06020000。原始 gen3-live-protocol-20260912.json 保存在隔离部署
  目录，SHA256=f5d463dc2e039c913655b593894b85647228713a59486a6377dd7eca740571d2。
- 采集前后 master Idle/Inactive、18 PREOP 从站。SDO 状态字不等同于已使能证据。
- 本地手册《机械臂-零差.pdf》4.4.2 要求偶数字节对齐，8.2.65 描述 Halt option，
  8.2.84 描述限力对象；PDF SHA256=1d4026599ec09e601366da5c1d95432f5dfc5582c4e949e216b371f9c1266221。
  旧 ESI ZeroErr Driver_V3.2.0.xml 的 INT 6072 和 TxPDO dummy 与字典/对齐方案相符，
  SHA256=67f7f1179e2c14c07ab1e3611116e33e4932f290551456f6317104e68e52372c。
- TDD 回归先复现：signed/padded profile 被拒绝、32768 上限被错误接受；修正后
  8 个 adapter/plugin 测试通过，包含 112-bit TxPDO 结构、32767 正值落到线上的字节及
  非法 padding/类型拒绝。首次 GREEN 误加载旧 install .so，已用 ldd/哈希定位后纠正。
- 目标机两包构建成功；robot_hw_ethercat 的 85 条测试记录（含原始 PDO/Action 结合测试）
  零错误/失败/跳过，47 项配置/launch 测试通过；本地质量门禁 214 项通过、受门禁覆盖率
  83%，ShellCheck 本机未安装。独立 Debug 覆盖率构建中的 adapter/Action 测试通过，
  zeroerr_pp_slave.cpp 行覆盖率 96.41%（167 行），分支执行率 92.13%、分支命中率 59.75%。
- 更新已安装到隔离开发工作区；配置哈希校验通过。安装步骤移除了构建 RUNPATH，
  因而整库哈希不同；readelf 的 .text 段输出哈希一致，确认安装的是本次机器码。
  已执行的包测试也从安装前缀解析库。未切换原有服务。
- 未做实机 PDO remap、PP 模式进入、限力/取消/停止、4 ms DC 或带动力验证；本次协议
  可行性结论不能替代实机运动准入。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 所有驱动当前 6060/6061=8，6502=0x38D 宣告 PP/CSP 能力；当前 RxPDO 为
  607A/60FE/6040，TxPDO 为 6064/60FD/6041，各 10 bytes。夹爪尚未进入 PP。
- F2: 所有驱动字典一致且 6072 为 int16；软件字符串分四组。左/右夹爪分别为
  550403170/550729090。共享 identity/revision 不能替代逐固件语义核验。
- F3: 实机没有 605D 对象；PP Halt/cancel 的停止斜坡及 6072 在 PP 下的生效语义需厂家
  确认。当前上位控制器不能通过忽略这个缺口进入实机模式。
- F4: 位置 3/8/12/16 的 603F=0x730F，手册 7.2.14 解释为编码器多圈保持电池电压低。
  同次状态字为 0 或 0x1000；此快照不独立断言当前处于 CiA402 Fault。

## 遗留

需要零差针对夹爪固件 550403170、550729090 确认：

1. 605D 不存在时，PP 的 6040 bit8 是否受支持，采用 6084 还是 6085 或固定停止策略；
   是否有对应固件手册、替代对象或官方升级方案。
2. 6072 是否在 PP 下实时限流、其 signed/unsigned 与单位定义、有效范围和零值含义；
   能否在 PREOP 将其加入 1600，并将 6061/606C/6077 和 dummy 加入 1A00。
3. 730F 的处理是否影响多圈保持和零点，以及各驱动对应的电池接线与电池状态。

以上问题尚未通过工具发送给第三方。现场先核实左 J3、左夹爪、右 J4、右夹爪的电池。
机械标定、正式关节名、控制器/enable_manager 与运行组合、IgH PDO 保留补丁、新分支器
状态/DC 与完整 Docker/HIL 验证仍按各自原有阻塞项推进。
