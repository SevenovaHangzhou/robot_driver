---
id: ecat-axes-20260912-02
area: ecat-axes
title: 汇川 GR10-EC-6SW 双设备分支器 ESI 归档
date: 2026-09-12
type: investigation
trigger: ELECTRI-118，用户提供 INOVANCE-GR10-EC-6SW-1.4.2.3.xml
commits: []
env: none
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-146, release-deploy-20260912-01]
---

## 背景

三代机分支器换为汇川。用户提供 ESI 后确认正确型号为 GR10-EC-6SW，之前 GR10-EC-65W 为输入名称。
此记录只冻结描述文件内容，不外推实物身份或已连接拓扑。

## 改动

原始 ESI 经换行和尾空白规范化归档到 robot_hw_ethercat/config/esi，附原始/副本哈希和端口说明。
更新三代机 manifest 注释和待核验事项，旧机 profiles、ring position 和状态例外不变。

## 验证

- XML 完整解析，原文件与仓库副本递归元素/属性/文本比较相同；文件 SHA-256 记录于相邻 README。
- 本地仓库门禁和 10 个物理/控制范围静态组合通过，目标源码及安装路径副本做哈希核验。
- 真实设备扫描、接线、DC/运行状态及 WKC 尚未验证；不根据 ESI 自动启动主站或下载设备配置。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: ESI Vendor=0x00100000，主/子设备 Product=0x10F40931/0x10F40932，Revision 均为 0x00010000。
- F2: 两个 JunctionSlave Device 通过内部 EBUS 相连，分别对应 IN/X2/X3 与 X4/X5/X6；
  若仅一台分支器加 16 电机，预期 18 响应设备、16 执行器，待扫描确认实际数量与顺序。
- F3: ESI 不含 RxPDO/TxPDO/Mailbox/Sm；声明 DC(0x0300) 和 Synchron(0) 选项，不能据此
  假定现场 DC 配置或 OP/PREOP 终态；分支器不加入 CiA402 运动控制器和使能轴列表。

## 遗留

实际左右臂所接端口、分支遍历顺序和实物身份待确认；由现场扫描补齐 BQ-146。
