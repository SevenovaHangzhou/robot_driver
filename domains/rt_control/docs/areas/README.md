# RT-Control 功能区与开发记录

本目录按七个职责区组织当前owner和冻结事实，与 `ARCHITECTURE.md` 的分层一一对应（control 层一个区，另加 platform）。2026-10-31 用户裁定：arm-control 与 chassis-control 合并为 control（记录原样迁入 `control/records/`，原 id 保留），新开 infra 区。
调整决策与历史条目处置见 [划分decision](../areas-archive/governance/records/2026-10-13-seven-agent-areas.md)。

## 当前七个功能区

| 功能区/agent | Owner资产 | 特殊排除 |
| --- | --- | --- |
| [protocol](protocol/README.md) | hw_ethercat、hw_canopen、hw_can_damiao、hw_semantic_components；ecat_icube/igh/ros2_canopen补丁 | 机型事实归hw-config，控制器归control |
| [hw-config](hw-config/README.md) | hwcfg_machines、src/description快照 | 公共模型权威仍在独立仓库 |
| [integration](integration/README.md) | integ_composer、integ_bringup、integ_api_adapter、两个操作网页、src/interfaces | 机型配置归hw-config |
| [control](control/README.md) | ctrl_enable_manager、ctrl_rolling_trajectory、ctrl_force_torque_broadcaster、ctrl_swerve；ros2_controllers补丁 | 协议实现与编码器提供器归protocol |
| [peripheral-control](peripheral-control/README.md) | periph_plc_io、periph_led_ultrasonic、periph_bms、periph_imu_lpms | 传输库归infra，公共IDL归integration |
| [infra](infra/README.md) | infra_modbus、infra_renderer、infra_shm_ring、infra_socketcan | 调用方实现归各自区 |
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
