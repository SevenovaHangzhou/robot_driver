---
id: peripheral-control-20261110-01
area: peripheral-control
title: PLC Modbus TCP 可选直连网卡绑定
date: 2026-11-10
type: fix
trigger: ELECTRI-195 / GitHub issue 248 / 用户要求直连绑定后复测
commits: [bugfix/plc-modbus-interface-binding]
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence:
  - /home/ar/plc-bind-20261110/evidence/samples.json
  - /home/ar/plc-bind-20261110/evidence/node.log
  - /home/ar/plc-bind-pr-main-20261110/test.log
related: [ELECTRI-195, '#248']
supersedes: []
---

## 背景

台架数字 IO 地址的默认路由走 Wi-Fi，而旧 IO 服务绑定有线网卡，因此旧服务能读、新节点超时。
用户授权增加显式直连绑定并只读复测，未授权设备输出操作。

## 改动

`infra_modbus::Client` 接受可选 interface 字符串，每次创建 TCP socket 后、connect 前执行
SO_BINDTODEVICE。空值沿用宿主路由；绑定失败关闭 socket 并报错，不回退到其他网卡。
`periph_plc_io` 传入 `digital.module.interface` / `analog.module.interface`，默认均为空。
具体台架网卡只在运行配置中设置，不硬编码进通用库。未修改 ROS 话题、服务、消息、压力公式或输出策略。

构造函数的 C++ 符号变化，升级共享库时必须重建所有调用方；本次覆盖 PLC 和 LED/超声包。
调用位于外设非实时线程，不进入 ros2_control 1 kHz 更新路径。

## 验证

- T0：从 main 22fe7c1 建独立分支；infra_modbus、periph_plc_io、periph_led_ultrasonic 隔离构建，
  colcon test-result 22条记录，0错误/失败。新增真实loopback测试：缺失网卡拒绝且无默认路由回退、
  绑定lo后的连接断开和下一次请求重连、非法名称校验；不需创建虚拟网卡或修改宿主网络。
- T2：此前候选使用同一绑定实现，两个模块配置 interface=enp4s0、事务超时1500ms，
  15秒收到138条PlcIoState，全部connected/data_fresh=true；三个输出有效且关闭。
  AI0 raw1025 / -0.63125kPa，AI1 raw1033 / -0.83325kPa。
  节点SIGINT退出0，旧vacuum-io服务恢复且outputs全关、write_count0。
- 实机结果属于此前现场基线的候选，未将main分支构建再次装到硬件运行；断线重连专项为loopback测试，
  不冒充设备拔线恢复。LED/超声尚未安装，仅离线回归。ELECTRI-195保持部分完成。

本次实机操作经现场授权，操作人：用户授权的助手；确认短语：“加上直连绑定，再测试下数字IO”。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: interface为空保持原路由行为；指定网卡绑定失败时事务失败，不回退。
- F2: 网卡绑定由infra_modbus拥有，PLC只传入数字/模拟量各自的部署参数。
- F3: 本次台架验证仅证明已配置输入与输出反馈读取有效，不证明压力精度或输出动作。

## 遗留

设备真实断线恢复、PLC输出动作、LED/超声实机验证仍待独立安排。本PR不关闭#248。
