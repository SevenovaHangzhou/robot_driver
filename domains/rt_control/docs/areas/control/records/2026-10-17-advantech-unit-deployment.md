---
id: chassis-control-20261017-07
area: chassis-control
title: 研华台架单机 Alfa_V3-Advantech-01：部署数据、CANopen 产物与实时核
date: 2026-10-17
type: feature
trigger: ELECTRI-127；用户 2026-10-17 授权补研华台架单机部署数据（CAN 接口、主机身份、实时核）
commits: [feature/electri-127-advantech-unit]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261017-05]
---

## 背景

新架构的 chassis_only 真实组合缺单机部署事实、CANopen 主站产物和实时核约束。
研华台架只有底盘，与 `Alfa_V3-01`（双臂/头部）不是同一台实物。

## 改动

- 新单机 `units/Alfa_V3-Advantech-01`：`deployment.yaml`（can0、500 kbit/s、主站节点 127、
  CANopen 产物目录 `/var/lib/rt-control/canopen/Alfa_V3-Advantech-01`、EDS 引用、主机名、EtherCAT
  网卡 MAC、CAN USB 序列号、rt_cpu 7 与 housekeeping 0-6,8-19）；底盘标定从 `Alfa_V3-01` 移入本单机。
  底盘机型模块与本单机标定标为 verified（台架已做使能与运动），背隙列为待标定且不再阻塞组合（运行时不使用）。
- 单机部署 schema：新增 `host`、`canopen.bitrate_kbps`、`canopen.bundle_dir`；删除未使用的
  `master_config/master_bin`。
- robot_hw_canopen：总线配置采用台架验证过的主站/节点策略（不广播启动/复位、boot 5 s、轮询 4 ms、
  TPDO1 同步、TPDO2 关闭）；`dcf` 为相对名，`dcf_path` 为产物目录；组合产物新增 `canopen_sources.yaml`；
  `generate_canopen_bundle <artifact>` 离线运行 dcfgen 生成 master.dcf/bin 并记录源哈希；
  预检要求产物完整且与当前组合一致、接口 UP/ERROR-ACTIVE、USB 序列号匹配。
- robot_hw_ethercat：预检核对主站附着的网卡 MAC。
- rt_control_bringup：硬件激活前后按单机 `host` 校验并绑定实时线程（更新线程与 rtcan-master 到 rt_cpu，
  其余到 housekeeping；IgH EtherCAT-OP 须已绑定 rt_cpu 并设 FIFO 79）。

## 验证

- composer 测试：208 passed，1 failed（安装布局 CLI 用例，既有环境问题）；新增研华单机真实组合与缺产物拒绝用例。
- robot_hw_canopen 预检/产物生成测试：6 passed；robot_hw_ethercat：55 passed；bringup：20 passed；
  tools：247 passed；quality_gate PASS。
- 主机事实为只读读回：hostname、isolated/nohz_full=7、`ec_master run_on_cpu=7`、MASTER0_DEVICE、
  can0 USB 序列号与 500 kbit/s。

未连接总线、未启动新架构运行时。

## 遗留

- 在工控机部署新架构并实机验证后，再删除旧部署。
