---
id: hw-config-20261023-01
area: hw-config
title: J6双编码器原始差值0x2241同步采集部署与未使能OP验证
date: 2026-10-23
type: commissioning
trigger: 用户确认2241必要时可以开启，并以“继续”确认有序停机重启
commits: []
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /home/ar/rt-control-electri127/dpi-staging/j6-2241-20261023/preparation.json
  - /home/ar/rt-control-electri127/dpi-staging/j6-2241-20261023/disabled_channel_trace.json
  - /home/ar/rt-control-electri127/sequence-preset-backups/20261023-j6-2241-1792763246615124193/deployment.json
supersedes: []
related: [arm-control-20261023-06]
---

## 背景

任务DPI-J6-2241-20261023。用户完成⑦后25段全部健康成功，但负重力保持下J6输出仍为零，需区分双编码器原始差值与0x3B69扭矩映射。吸盘已包含在left_tool_link，不重复计外载。

## 改动

新增J6专用zeroerr_cia402_csp_torque_encoder profile，仅在bench_left_arm的J6选用。0x1A00末尾追加0x2241:00 INT32，TxPDO由22B变26B（8项，手册上限76B），接口left_joint6/dual_encoder_difference_counts。保留0x3B69设备符号及mNm，原始差值不乘比例、不清零；其余轴PDO、RxPDO、SDO启动参数、模型几何惯量和控制/安全参数不变。

相应既有测试更新J6配置期望，正式组合器同步生成XML、recorder清单和artifact哈希。完整原源码及artifact备份见evidence目录；回退脚本保存在用户辨识项目，不自动启动。

## 验证

隔离组合重现现场基线、三项生成物结构差分通过；15项既有相关测试通过，两包隔离构建通过。正式安装首次构建因隔离副本在工作区内被重复发现失败，随后给dpi-staging添加COLCON_IGNORE，并限定--base-paths robot_driver/src；再次两包构建3.32秒通过。git diff --check和cached检查通过；既有全仓quality_gate的jsonschema版本阻塞未被宣称通过。

经arm stop有序失能、停控制器并清理硬件，17从站PREOP；安装后arm start left启动未使能。新run为20261023-215004-d77-bench_left_arm，管理器IDLE、JTC不active、无序列和运动。ethercat pdos -p 6回读包含原0x3B69及末项0x2241，字段72个。

真实OP共享内存10.035秒、10036帧，同帧两通道；seq连续、读者无丢帧、七轴OP、链路在线、WC始终0、时间间隔0.994～1.007ms。0x2241为−277～−75原始计数、21个不同值；同时间0x3B69均为0。此为未使能/制动状态，不据此校准扭矩或诊断故障。

所有轴错误对象保留有序停机产生的0xA000，状态字无Fault位且未Operation Enabled；未自动复位。严格健康辨识门槛的error_code==0未通过，这份短记录仅用于通信通道验证，不混入健康动作统计。固件0x100A单独读取未完成，未宣称已独立回读。

本次实机操作经现场授权，操作人：用户授权、Codex代执行；确认短语“继续”，入口arm stop、arm start left。未执行复位、使能或运动，本记录不授权后续使能或运动。

## 结论与冻结事实

- F1: J6新0x2241和原0x3B69已实际进入同一1kHz recorder帧；验证仅覆盖未使能OP通信。
- F2: 当前状态IDLE、JTC inactive，七轴未使能，0xA000记录未清；后续动作需现场恢复正常零错误使能状态。
- F3: 新生成XML哈希0859cf22e7fb5345dcfebf5dcbc8ce650e201b39a92cd0e515d6483a3f20b90b，仅新增ros2_control状态接口；旧录制仍按013950bc69c4a67a9cc73d9062591346438ffc29e425f2d4468b8e5d52379e29快照分析，不重标历史模型。

## 遗留

现场恢复正常状态后只需带新字段复测⑦；暂不全部重录。⑧只是固定姿态准备，独立已知双向外力矩测试需另外连续记录及真实加载几何/力信息。当前没有被采纳的动力学或传感器校准参数。尚未验证新配置的使能后动作数据或全栈Mock生命周期，不自动扩大授权。
