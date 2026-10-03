---
id: contract-20261003-02
area: contract
title: 三代机 Rolling 轴合同纠错与扩大复核
date: 2026-10-03
type: corrective
trigger: ELECTRI-174，用户指出 Rolling 仍含 turn/updown 并要求扩大检查
commits:
  - feature/electri-152-interface-contract
  - feature/electri-174-interface-contract-gitea
  - feature/electri-174-rt-interface-draft
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [ELECTRI-174, BQ-152, BQ-153]
---

## 背景

恢复公共 Rolling 合同时保留了旧 `right_joint1..6,left_joint1..6,turn,updown` 注释，
但 driver 的 `kJointNames` 和 `kAxisSetHash` 早已采用 V3 双七轴。相同14元素并不意味着
相同轴语义；该不一致会误导客户端构造错误轨迹。用户要求修正并检查其他问题。

## 改动

- 公共候选 `ea582c8a711874813d2ad7f0801b366bd32ab67e` 的注册表、四个 Rolling IDL、
  FJT约束、生成视图统一到 `right_joint1..7,left_joint1..7`，所有位置 rad、速度 rad/s。
- `axis_sets.v3_dual_arm` 登记规范顺序和
  `f4c8ff8a32183d1733032494600c9f5d1e7e625673a4ad51556b3ac6c35adee4`；
  门禁重新计算 UTF-8 逐行含末尾 LF 的SHA-256，拒绝顺序、数量和哈希漂移。
- 不改变 Rolling wire 数组、协议1.0、会话／序号机制或运行控制周期。不能仅把V2 hash换为V3
  就继续使用旧轨迹；Motion必须按双七轴重新生成目标。
- G-01、P-01/P-02、M-08、N-17 在公共注册表和生成视图明确标为 Legacy V2；旧类型保留供
  消费方迁移，不宣称V3业务已实现。去除Autonomy“完全不得调用RT”的笼统注释，遵守实际
  泵／LED／人工保护复位职责。
- driver增加真实Rolling服务回调测试：完整旧双六轴＋turn/updown哈希被拒绝且不建会话，V3
  哈希通过。另校验导入契约轴序/单位/hash与控制器源码一致。
- 真空未知输出不再被任意周期快照解除；暂时要求调用方显式操作取得CONFIRMED才清除该侧
  未知锁存，RT不自动重试。准入快照读取移至资源锁内。
- 位置原型缺少速度观测时不能认定到位／停稳；settle计数只消费新样本。
  真实位置执行仍被preview gate禁止，不将这两项修正等同于完整停止后端。
- 预览跨进程测试统一父子ROS_LOCALHOST_ONLY；此前CI仅子进程为1，父进程依赖环境，出现
  discovery超时。修正后在外部ROS_LOCALHOST_ONLY=0条件下本地通过，CI仍需重跑确认。

## 验证

- 公共接口：全部生成/contract/error/changelog门禁、63项Python工具测试通过；六包构建通过。
  修正过程中曾因rosidl对注释中转义换行的处理造成生成失败，已改成文字LF说明并重建通过。
- driver：Rolling控制器及适配器隔离构建通过；colcon汇总178 tests、0 errors、0 failures。
  汇总数包含测试包装条目，不作为独立实物场景计数。
- 原问题用无设备Fake IO复现：同一份写前快照能清除未知标记并允许停泵；修正后的回归测试
  明确拒绝这种停泵，只有调用方显式取得输出确认后才解除。
- 位置原型回归：仅位置反馈不能证明已停止；原有19项真空／位置聚焦测试通过。
- 旧driver CI run 37109023983：编译完成，测试阶段仅preview discovery失败；不能称其CI通过。
  不将负向控制器测试日志中的预期ERROR误报为额外失败。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: V3 Rolling的14轴全部属于双臂；升降、PP、头部、底盘都不占此数组槽位。
- F2: 有效轴身份必须同时满足名称、顺序、单位与V3哈希；数组长度相同不构成兼容。
- F3: 周期IO消息尚无每路事务完成身份，不能用它解除未知写结果；显式确认策略是保守暂行收口。
- F4: Draft尚有以下软件缺口；这些不是仅靠补实物配置即可消除。

## 遗留：扩大复核发现（按影响）

| 等级 | 文件／位置 | 问题与影响 | 当前边界 |
| --- | --- | --- | --- |
| HIGH | `plc_io_modbus/src/node.cpp` 的输出写失败和 `publish_plc_state` | 失败路径可能保留上次输出valid，统一header刷新又不能代表每路线圈的新采样；压力同样需逐侧独立时效 | 新未知锁存降低停泵误判，但仍须补逐输出样本身份、失败置无效及并发完整测试 |
| HIGH | `position_action_adapter.py` 的 `_execute_resource` | 原型用实测位置覆盖命令不等于驱动停止确认；新Goal准入/抢占边界、控制器激活、失败收尾及逐轴范围仍需实现 | `configured=true`在节点构造时被拒绝，不允许投入真实执行 |
| HIGH | `module_state_adapter.py` 的 `ProtectiveStopCore` 与节点 | 纯逻辑核心没有接DI、全机stop、停稳证据、复位权限和重启锁存；禁止把状态发布当成触边保护 | `edge.protection_enabled=true`被拒绝；硬件未安装状态保留 |
| HIGH | `module_state_adapter.py` / `status_adapter.py` | 仍未形成按能力的DomainReadiness，旧总门禁未替换；IMU/力模块freshness未配置且力质量发布链未接入 | 不标记整机/力控可用；需能力依赖表及真实生产者接入 |
| MEDIUM | 公共 M-08/P-01/P-02/G-01/N-17 | 旧12臂轴、Turn、0.74m固定计划等上层协议仍存在，与V3流程不同 | 已显式加legacy标记；真实上层迁移仍由对应域实现 |
| MEDIUM | 独立头部runtime与整机状态组合 | preview不建旧命令publisher不代表真实头部controller旧入口退役；RSP开关不等于整机TF已验收 | 仍在Draft缺口清单，需单writer/全关节覆盖联合验证 |

实物参数按BQ-152的既定分批计划处理。接口和driver仍待评审，未合并／部署；Review中新增的
软件缺口不通过把参数设成“已验证”或只改readiness来掩盖。
