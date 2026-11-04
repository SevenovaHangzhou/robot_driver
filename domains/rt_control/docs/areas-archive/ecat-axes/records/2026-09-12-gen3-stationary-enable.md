---
id: ecat-axes-20260912-11
area: ecat-axes
title: 三代机机械零位归档与 14 CSP 加 2 PP 原位使能实测
date: 2026-09-12
type: commissioning
trigger: 用户确认当前位置为机械零位、无额外传动比，并要求继续 14 CSP 与 2 PP 使能
commits: []
env: native
risk: T3
writes: {reset: yes, enable: yes, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-147, BQ-146, ecat-axes-20260912-10, lifecycle-20260912-01]
---

## 背景

用户明确批准本次真实使能，并确认当时各轴位置为机械零位、电机输出后无额外传动。
本阶段只验证驱动原位使能和失能，不接收轨迹、夹爪开合或力控制目标。

## 改动

- UTC 08:43:26 采集十六轴原始 6064/20A0/2240/identity/mode 并保存；硬件包
  config/machines/alfa_v3_mechanical_zero.yaml 记录用户零位与 external_transmission_ratio=1。
  原始输出端 counts 不冒充 rad 或 m，不再乘一次电机内部减速比；正式模型方向/角度
  换算与夹爪米/牛顿标定仍独立待验证。
- 硬件所有者模板 zeroerr_stationary.yaml 使用零差可变 RxPDO 1600：607A:int32 +
  6040:uint16（6 bytes）；TxPDO 1A00 保留 6064/60FD/6041（10 bytes）。
  60FE bit0 是松闸位，本入口不下发该对象。唯一启动 SDO 为 6060=8 或 1。
- 复用固定版本 EcCiA402Drive 的原始位置预加载；不导出 position/velocity/effort
  命令接口，仅导出 control_word。未启用自动状态跳转或自动清错；PP 控制字不置 bit4，
  不触发新目标，也不依赖尚未验证的 605D Halt 和夹持力 Action 参数。
- 此模板不选择 use_slave_pdo_defaults/PreservePdoConfig，也不加载 X503 等固定 PDO
  设备。此前全生产栈 IgH 补丁检查缺口不适用于该可变映射测试，不修改主机 IgH/内核。

## 验证

- 写前验证 master 0 Idle、18 从站、身份与 X2/X3 分支；以配置的 250 Hz 激活总线。
  16 台电机全部 OP，Domain0 为 256 bytes、WC 48/48，驱动预加载成功。
  分支器主设备 0 为 OP、子设备 17 保持 PREOP：仅记录本次实测，不套用旧机位置。
- 实际 mode readback：1..7/9..15 为 8(CSP)，8/16 为 1(PP)。两夹爪 PP 模式进入成功，
  但本次不验证 PP 运动、Halt/cancel 或 6072 的力限制效果。
- 左 J3/右 J4 在 OP 后因 730F 进入 Fault；按用户确认的单圈用途调用运行栈
  /rt/reset_fault 成功清除，随后全部 16 轴 603F=0、SwitchOnDisabled。
- /rt/enable 在 UTC 09:22:49.751634 返回 success；两次逐轴回读全部 OperationEnabled：
  CSP statusword=1637、PP=0637、controlword=000F，603F 均为 0，WC 48/48。
  所有目标 607A 在两次使能态快照中保持不变；采样的最大 |607A-6064| 为 6 counts。
  完成后约 6.66 s 调用 /rt/disable 返回 success；全部 16 轴控制字 0、SwitchOnDisabled。
- controller_manager update 线程 FIFO80 绑定现场已有隔离 CPU14，其余线程保留 housekeeping。
  使用标准 rt_control_start 退出流程，失能、停止控制器、hardware deactivate 均成功，
  manager 退出 0；最终主站 Idle/Inactive、18 从站全部 PREOP。未切换原生产服务。
- 原位 CSP/PP 合成 PDO 测试验证精确预加载、发送确认前拒绝 enable、使能后目标锁定、
  PP bit4..6 为 0、失能后重新预加载。硬件包 88 条记录、配置/launch 共 52 项通过。
- 证据位于隔离部署目录：gen3-confirmed-mechanical-zero-20260912.json
  (SHA256 5f4f29cc1215e4ef9fb75ab0df5e3e90e962a4994d813f7979bbc87fde44bbf0)、
  stationary-enable-hold-20260912.jsonl
  (4d4ecdf4608fd4e0ef52fe0dc44ee46463f1230ce900672c38f7c120287bcd39)、
  real-enable-zwtu751y.log (415ca5226cf6761a7b98e5bcf7551a29381d64599c25894694a7b1e4d0c6e373)。

本次实机操作经现场授权，操作人：Codex（用户授权），确认短语入口：
“我确认当前位置为机械零位 除电机本身减速器外 无额外传动比，继续做14个CSP和2个PP模式的使能”。

## 结论与冻结事实

- F1: 14 CSP + 2 PP 已完成真实原位使能、短时保持与失能验证；当前不是使能状态。
- F2: PP 模式本身可进入且使能；605D/限力问题继续约束运动 Action，不能据此宣称
  PP 开合、取消停止或力控制已完成实机验证。
- F3: 当前机械零位已保存为原始输出端位置；使能预加载使用启动时实际原始位置，
  不向保存的机械零位自动运动。
- F4: 在本次运行的 PDO 控制字路径上，J3/J4 的单圈电池故障可清除；此前夹爪
  PREOP SDO 清错失败的完整固件原因仍未由此查明。

## 遗留

该入口是原位使能测试，不是完整三代机运动运行栈；标准模块 launch 仍保留原有准入。
需要继续完成正式 Robot Model、SI 换算/方向/限位、PP 行程与力标定和停止语义、
长期 DC/故障注入及完整生产 Docker 发布验证。退出时上游 ClassLoader 有对象仍在堆上的
卸载警告，进程退出 0 且总线已收尾；后续清理生命周期时再调查，不冒充没有警告。
