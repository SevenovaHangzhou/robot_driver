---
id: control-20261121-02
area: control
title: 急停软件锁存、台套配置与回归收敛
date: 2026-11-21
type: fix
trigger: SEV-135 / robot_driver#299；SEV-73 / robot_driver#300；用户请求先处理遗留再上库模型外置
commits: [codex/control-backlog-fixes]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [SEV-135, robot_driver#299, SEV-73, robot_driver#300]
---

## 背景

同周期急停变更阶段可能绕过硬件离线锁存；PRESSED 未推进序号水位，使后续旧 RELEASED
可能恢复释放证据。现场时间未确定，原通用包的 timeout=0 也不能完成部署配置。
用户于 2026-11-21 明确说明：目前只接普通 IO，经 Modbus/软件执行停机。
本记录不将该链路认定为经过验证的硬件安全急停。

## 改动

- 在急停阶段变更前独立记录活跃硬件离线的 restart_required；按下和离线同周期出现时两项均保持。
  同一控制器重新 activate 不再清除该锁存，启动清理完成后仍报告 Failed/restart_required。
- PRESSED 推进生产者序号水位；相同或更旧序号的 RELEASED 不能恢复释放证据，旧 PRESSED 不回退水位。
  无法标识的按下也使旧样本的新鲜度失效。停机事件仍保留到 RT 消费。
- 四个时间参数归 `units/<unit>/deployment.yaml` 的 emergency_stop 段，未知值为 TBD。
  实机响应范围在映射或时间未确定时拒绝生成 artifact；mock 不连接实际输入，缺接收阈值仍不能 configure 控制器。
- composer 为选中消费者生成输入节点参数和 input_peripherals；现有 scope launch 管理节点生命周期。
  接收器的原有失联/过期路径处理节点退出，控制器继续完成停机；不新增独立看门狗或自动恢复。
  构建、CI 和容器包集合包含 periph_emergency_stop。
- SEV-73：冻结快照由真实 composer 重新生成，取消断言时改写历史期望；测试读取模型限值，
  台套测试时间仅来自显式本地夹具。节点晚加入测试在后续定时发布前验证构造期缓存样本；
  接收器测试在首次发布后才创建订阅者，禁止第二次发布满足断言。保留重力、协议、停止和生命周期测试。

## 验证

T0：相关 8 包构建通过；组合/启动 pytest 317 通过；colcon 汇总 561 项、0 error、0 failure、2 skip，
包含控制器及本地 TCP/DDS、重力/协议、组合/启动与模型测试。首次发布观察后销毁旧 reader，
再创建晚加入 reader，确认缓存消息完全相同，避免把发现时序误当历史交付。
质量门禁通过（185 项工具测试通过，11 跳过；本机缺 ShellCheck）。Docker/Compose 工具不可用，镜像构建未验。
初次运行缺少 ROS/renderer Python 环境，修正本地验证环境后重跑；不把环境失败写成代码验证通过。
T1 全图、T2–T4 及现场时序、带载停止验收均未完成。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 按下/离线分别锁存，按钮释放和同实例 activate 不构成完整 RT-Control 重启。
- F2: 四个时间必须来自确认后的台套配置，不能把本地夹具或器件典型值当作现场默认值。
- F3: 用户报告现场目前仅有普通 IO/Modbus 软件停机；安全回路设计与验收仍未闭环。

## 遗留与验收交接

#299 保持未完成：先由硬件/安全负责人确认安全控制链、停止类别、所需性能与机械停止预算。
GB/T 16754-2021 等同采用 ISO 13850:2015；IEC 60204-1 涉及机械电气急停，ISO 13849-1
提供安全控制系统设计方法。这些不能直接给出本机的四个 Modbus/DDS 时间参数。
[国家标准平台](https://std.samr.gov.cn/gb/search/gbDetailedCNF?id=C3386C490D038B79E05397BE0A0AC288)、
[IEC 60204-1](https://webstore.iec.ch/en/publication/71256)、[ISO 13849-1](https://www.iso.org/standard/73481.html)。

例如 [G9SE 最大 15 ms](https://www.ia.omron.com/products/family/3419/specification.html) 和
[PNOZ s4 最大 20 ms](https://www.pilz.com/download/open/PNOZ_s4_Operat_Man_21396-EN-17.pdf)
是对应安全继电器自身断开指标，不是机械停稳时间，也不是普通网络链的时延保证。

现场交接按 T0→T4 逐级进行，由操作台执行实机动作：

1. 安全回路/停止方式和各级时间预算确定后，测量输入刷新、请求/网络最坏延迟及样本年龄，填写四个台套时间。
2. T1 全图确认输入缺失、节点退出、启动部分失败、通信中断以及停机收敛。T2 核实接线、极性、序号与时间基准。
3. T3 验证按下、释放不自动使能、人工复位、同周期离线和完整重启要求。
4. T4 在独立安全措施、隔离和监护条件下验证分级运动、最坏负载停止时间/距离。
   #300 的重力标定、力矩换算、静态姿态/生命周期及带载验收继续保留；重力前馈仍默认关闭。

三个 Issue 不因这些 T0 结果自动关闭；模型外置修改待遗留处理后单独上库。
