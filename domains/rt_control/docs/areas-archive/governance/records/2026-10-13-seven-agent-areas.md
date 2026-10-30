---
id: governance-20261013-03
area: governance
title: 九个历史区对齐七个agent职责区
date: 2026-10-13
type: decision
trigger: "口头需求（kkozia）"
commits: [feature/rt-control-seven-agent-areas]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: [governance-20260813-01]
related: [governance-20261013-01, governance-20261013-02]
---

## 背景

原九区混合旧机总线和发布职责，新控制器、CAN与semantic包没有清晰owner。用户要求七个agent一一对应七区，本次仅文档/索引迁移，不移动任何功能包。后续两条记录规则、历史冻结和T0–T4定义保持原意；supersedes仅取代旧九区划分。

验收：新七区覆盖指定资产；所有旧表条目有迁移或归档结果；旧记录字节不变；用例仅改area；全仓旧区名残留区分历史坐标与非功能区语义；质量门禁通过。

用户裁决：旧事实、过期接口归档掉。跨区事实按资产主责拆分并互引：协议/生命周期归protocol，运行拼装归integration，机型与标定归hw-config，宿主构建归platform。拿不准的新事实继续问用户，不推断硬件参数。

## 改动

| 原区 | 新主责 |
| --- | --- |
| ecat-axes | protocol / hw-config / arm-control / platform |
| canopen-chassis | protocol / hw-config / peripheral-control |
| lifecycle | protocol / arm-control / integration |
| motion | arm-control / chassis-control / hw-config / integration |
| io-power | peripheral-control（公共IDL边界仍属integration） |
| contract | integration / hw-config / peripheral-control / platform |
| realtime-host | platform（协议补丁归protocol） |
| release-deploy | platform / integration / hw-config / protocol / arm-control |
| governance | platform |

- 建立七区README，列明范围、owner和排除项。config/machines优先于包级归属，vacuum_adapter及专属测试优先于control_api_adapter整体归属。
- 九个旧区只在README顶部加历史声明；原记录不改不动。当前三任务按用户要求保留在旧governance，本条为最后一次旧区登记。PROGRESS历史行保留，本条新增索引用platform。
- **目标架构**：协议和硬件配置解耦为两层，由通用组合器组合。协议层拥有协议语义和通用转换机制，机型/标定层拥有单位换算参数、方向、offset等机型事实。这改变`hardware-configuration-layering-and-motor-variants.md`中相关事实归slave profile的目标归属；该文档现有实现说明保留，并在顶部注明过渡。
- **代码迁移是后续独立任务**。本轮不把任何比例、方向或offset从原文件搬走，不改数值、不改变运行或安全边界。既有BQ的设备/安全裁决不因owner划分改变。
- 冻结事实盘点155条，迁移103条，归档52条；其中统一力发布条目只保留架构事实，旧蓝点revision/布局子句归档。以下清单逐条给出落点；新区保留原文、来源与验证边界，历史实验不能外推至当前主机。

### 冻结事实处置清单

| 原表坐标 | 处置 |
| --- | --- |
| canopen-chassis/F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| canopen-chassis/F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| canopen-chassis/F3 | protocol |
| canopen-chassis/F4 | hw-config |
| canopen-chassis/F5 | peripheral-control |
| canopen-chassis/F6 | peripheral-control |
| canopen-chassis/F7 | peripheral-control |
| contract/01#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/02#F1 | integration |
| contract/02#F2 | integration |
| contract/02#F3 | integration |
| contract/02#F4 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/03#F1 | integration |
| contract/03#F2 | integration |
| contract/03#F3 | platform |
| contract/03#F4 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/04#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/04#F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/04#F3 | platform |
| contract/05#F1 | platform |
| contract/05#F2 | platform |
| contract/05#F3 | platform |
| contract/06#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/06#F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/07#F1 | platform |
| contract/07#F2 | platform |
| contract/07#F3 | platform |
| contract/08#F1 | hw-config |
| contract/08#F2 | hw-config |
| contract/08#F3 | hw-config |
| contract/09#F1 | integration |
| contract/10#F1 | integration |
| contract/11#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/12#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/13#F1 | integration |
| contract/13#F4 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| contract/14#F1 | peripheral-control |
| contract/14#F2 | integration |
| contract/14#F3 | chassis-control |
| contract/14#F4 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/X503-PREOP-1 | protocol |
| ecat-axes/TI5-ASSIGN-1 | platform |
| ecat-axes/MAIN-RATE-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/RATE-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/TI5-PDO-1 | protocol |
| ecat-axes/DC-STAGES-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/DC-HANDOFF-1 | protocol |
| ecat-axes/DC-TIME-1 | protocol |
| ecat-axes/01#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/01#F2 | protocol |
| ecat-axes/01#F3 | protocol |
| ecat-axes/02#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/03#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/04#F1 | arm-control |
| ecat-axes/05#F1 | protocol |
| ecat-axes/06#F1 | protocol |
| ecat-axes/07#F1 | hw-config |
| ecat-axes/08#F1 | hw-config |
| ecat-axes/09#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/10#F1 | hw-config |
| ecat-axes/11#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/12#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/13#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/14#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| ecat-axes/15#F1 | protocol |
| ecat-axes/16#F1 | hw-config |
| ecat-axes/17#F1 | arm-control |
| governance/F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| governance/F2 | platform |
| governance/F3 | platform |
| governance/F4 | platform |
| governance/F5 | platform |
| governance/F6 | platform |
| governance/F7 | platform |
| governance/F8 | platform |
| governance/F9 | platform |
| governance/F10 | platform |
| governance/F11 | platform |
| governance/F12 | platform |
| governance/F13 | platform |
| governance/F14 | platform |
| governance/STYLE-1 | platform |
| governance/CONTRACT-1 | platform |
| io-power/01#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/01#F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/02#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/03#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/04#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/04#F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/04#F3 | peripheral-control |
| io-power/05#F1 | peripheral-control |
| io-power/05#F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/05#F3 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/06#F1 | peripheral-control |
| io-power/06#F2 | peripheral-control |
| io-power/06#F3 | peripheral-control |
| io-power/06#F4 | peripheral-control |
| io-power/06#F5 | peripheral-control |
| io-power/07#F1 | peripheral-control |
| io-power/07#F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/08#F1 | peripheral-control |
| io-power/08#F2/F4 | peripheral-control |
| io-power/08#F3 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/10#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/10#F2 | peripheral-control |
| io-power/10#F3 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/10#F4 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/11#F1 | peripheral-control |
| io-power/11#F2 | peripheral-control |
| io-power/11#F3 | peripheral-control |
| io-power/11#F4 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| io-power/12#F1 | peripheral-control |
| lifecycle/20261009-03#F1 | protocol |
| lifecycle/20261009-02#F1 | arm-control |
| lifecycle/20261009#F1 | arm-control |
| lifecycle/01#F1 | protocol |
| lifecycle/01#F2 | protocol |
| lifecycle/02#F1 | protocol |
| lifecycle/03#F1 | protocol |
| lifecycle/04#F1 | integration |
| lifecycle/05#F1 | protocol |
| motion/RATIO-1 | hw-config |
| motion/F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| motion/F2 | arm-control |
| motion/F3 | chassis-control |
| motion/F4 | chassis-control |
| motion/F5 | hw-config |
| motion/F6 | chassis-control |
| motion/F7 | chassis-control |
| motion/F8 | integration |
| motion/F9 | hw-config |
| motion/F10 | chassis-control |
| motion/F11 | integration |
| motion/F12 | integration |
| motion/F13 | integration |
| realtime-host/HOST-DC-EXPERIMENT-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| realtime-host/HOST-STARTUP-RT-2 | 归档：旧主机CPU14初始化调度观测 |
| realtime-host/HOST-STARTUP-RT-1 | 归档：旧主机CPU14初始化调度观测 |
| realtime-host/HOST-IDENTITY-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| realtime-host/HOST-CAN-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| realtime-host/HOST-SCHED-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| release-deploy/2026-09-24#F1 | protocol |
| release-deploy/07#F1 | integration |
| release-deploy/V3-OLD-IPC-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| release-deploy/V3-ARM-RUNTIME-1 | integration |
| release-deploy/V3-HEAD-CAN-1 | arm-control |
| release-deploy/PR25-INTEGRATION-1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| release-deploy/01#F1 | integration |
| release-deploy/01#F2 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| release-deploy/01#F3 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| release-deploy/02#F1 | hw-config |
| release-deploy/03#F1 | hw-config |
| release-deploy/04#F1 | integration |
| release-deploy/05#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |
| release-deploy/06#F1 | 归档：仅保留旧区原表及原记录，不作为新区当前有效事实 |

归档包括：已退役履带/旧运行拓扑、旧主机CPU14/身份、明确被取代条目、旧接口pin/待合并状态、旧版本部署和瞬时故障观测、已被后续BMS路径替代的单电池基线、旧九区划分。归档不删除证据，也不宣称旧观测错误。

## 验证

- 旧records逐文件字节/路径比较、PROGRESS原内容前缀比较：PASS（新记录除外）。
- YAML逐用例去除area后对象比较：PASS；T0–T4、writes、命令、covers与其他字段不变。
- 155条原表坐标与处置表一一对应；103条新区来源链接解析存在；跨区来源使用原记录而非复制记录。
- 全仓旧区字符串审查：历史目录/记录id/来源链接/PROGRESS历史行保留。`motion`、`contract`、`lifecycle`、`governance`在代码、协议、CI job名中另有语义，禁止为了清零命中改变控制逻辑或CI身份。新增区名和area字段使用七区。
- `STYLE_BASE_REF=feature/rt-control-agent-contract tools/quality_gate.sh`：PASS：224通过、13跳过，门禁覆盖率84%。`python3 tools/release_test_runner.py validate`：33用例通过。125份旧记录字节不变，所有新区相对链接存在。
- 纯划分、注册、文档调整按第三档不新增测试；只同步已有fixture的area。无控制运行、构建或硬件操作，记录状态UNVERIFIED。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 当前功能区固定为protocol、hw-config、integration、arm-control、chassis-control、peripheral-control、platform，每区对应一个agent职责。
- F2: 更具体路径优先，跨区事实按资产主责拆分互引；旧区不再接收新记录，旧记录保持不可变。
- F3: 协议/机型配置两层与通用组合器是目标架构，本次只调整归属，实际源码迁移必须另立任务。
- F4: 过期事实和接口保留在历史区；不把历史平台验收外推为当前V3事实。

## 遗留

协议与机型配置的代码解耦后续单独推进。历史参考链接必须保留旧目录坐标，不能按字面消除所有旧区名。合并由人按前置PR顺序执行，本条记录不批准新硬件参数或真实运行。
