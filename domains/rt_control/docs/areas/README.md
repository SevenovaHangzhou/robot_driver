# RT-Control 功能区与开发记录

本目录按七个agent职责组织当前owner和冻结事实。源码包位置本轮不变。
调整决策与历史条目处置见 [划分decision](../areas-archive/governance/records/2026-10-13-seven-agent-areas.md)。

## 当前七个功能区

| 功能区/agent | Owner资产 | 特殊排除 |
| --- | --- | --- |
| [protocol](protocol/README.md) | robot_hw_ethercat、robot_hw_canopen、robot_hw_can、rt_control_semantic_components、enable_manager；ecat_icube/igh/ros2_canopen补丁 | config/machines、robot_hw_ethercat/variants归hw-config |
| [hw-config](hw-config/README.md) | 各包config/machines、robot_hw_ethercat/variants、alfa_v3.yaml、src/description快照 | 公共模型权威仍在独立仓库 |
| [integration](integration/README.md) | rt_control_bringup其余部分、control_api_adapter非真空部分、src/interfaces | 机型配置归hw-config，真空适配器归peripheral-control |
| [arm-control](arm-control/README.md) | rolling_trajectory_controller、rt_force_torque_broadcaster、x503_force_sensor；ros2_controllers补丁 | 协议实现归protocol |
| [chassis-control](chassis-control/README.md) | swerve_driver | 机型配置归hw-config，协议/编码器提供器归protocol |
| [peripheral-control](peripheral-control/README.md) | plc_io_modbus、modbus_tcp_rtu485、bms_node、lpms_nav3_can | 公共IDL归integration，config/machines归hw-config |
| [platform](platform/README.md) | hostsetup、docker、tools、.github、deps.repos、versions.env、testing、各级AGENTS、协作规范及记录体系 | 工具归属不改变其检查对象的owner |

未写完整路径的包均位于`src/rt_control/`，补丁位于`patches/`。
更具体路径优先于包级范围。机型事实和标定目标归hw-config，但当前位置和运行行为不因文档划分自动迁移。

## 归属规则

1. 每条新记录唯一归属一个当前功能区。
2. 跨区工作按资产主责拆分事实并互引原始记录。协议/生命周期归protocol，运行拼装归integration，机型/标定归hw-config，宿主/构建归platform。
3. 拿不准的归属提交用户裁决，不猜。新边界调整先在platform留decision。
4. 旧记录内容不修改。2026-10-13 前的 9 个旧区已整体移到 [`../areas-archive/`](../areas-archive/)（arch-s3d，只改目录位置与指向新区的链接）；新表引用旧记录时保留原记录 id。
5. 旧事实和过期接口归档到旧区，不在新区冒充V3有效配置。本轮逐条迁移/归档清单保存在划分decision。

## 记录规则

1. 只有行为变更或硬件事实变更才写记录（根 `AGENTS.md` 第 7 节）；纯结构/文档变更不记。一条记录对应一次可验证变更事件，不按commit拆分。
2. 新记录放`<area>/records/YYYY-MM-DD-<slug>.md`，使用[TEMPLATE.md](TEMPLATE.md)的完整frontmatter及五段正文。
3. 合并后的记录不可修改；推翻结论用新记录及supersedes。
4. 各区README冻结事实表是当前结论的可变视图，必须同步更新来源和状态。
5. `PROGRESS.md`保持时间线索引，历史行不改；只为实机验证追加新行，使用七区名。
6. 验证状态沿用既有规则。没有运行证据时写UNVERIFIED，不把静态/Mock当实机。
7. 本次三个准备任务按用户明确要求在原治理区留记录；第三条decision是关闭旧区前的最后一次登记。后续治理工作归platform。

历史区入口、九区到七区的对应关系及归档条目统一保存在[本次decision](../areas-archive/governance/records/2026-10-13-seven-agent-areas.md)。历史来源链接必须可追溯，不为消除字符串命中而改写旧记录。
