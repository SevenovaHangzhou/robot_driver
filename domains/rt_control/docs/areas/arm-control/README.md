# arm-control — 机械臂轨迹、头部控制与力反馈

**范围**：机械臂轨迹、头部控制与力反馈。本区对应一个同名职责agent。

**Owner路径**：rolling_trajectory_controller；rt_force_torque_broadcaster；patches/ros2_controllers。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：EtherCAT/CAN协议与enable_manager归protocol；启动拼装归integration；config/machines归hw-config。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| ecat-axes/04#F1 | X503B raw shadow bridge 只消费 rt-control state frame；V1.6 已定义单位码 `5=N`、`7=N·m`，采样原码采用 `-999999..999999` 范围门禁，实际回读/TF 未完成时禁止 WrenchStamped | [ecat-axes-20260906-01](../../areas-archive/ecat-axes/records/2026-09-06-electri-116-x503b-shadow-bridge.md)#F1-F3 | PARTIAL（T1；目标 SDO/实际参数/TF 待验） |
| ecat-axes/17#F1 | X503 周期发布迁入共享 semantic component + C++ broadcaster。原行中的旧蓝点revision/布局描述归档，不作为当前实物身份 | [统一六维力发布与蓝点可选配置](../../areas-archive/ecat-axes/records/2026-09-17-force-torque-cpp-broadcaster-and-bluepoint-option.md)#F1-F4 | PARTIAL（T1 Mock；X503/蓝点实机待验） |
| lifecycle/20261009-02#F1 | 头部会话已归档；两台临时配置速度0.2rad/s，历史PI比较0.1rad/s | [会话归档](../../areas-archive/lifecycle/records/2026-10-09-damiao-session-readme.md)#F1-F2 | PARTIAL（新速度未加载验证） |
| lifecycle/20261009#F1 | 达妙控制器切换保留 ListControllers 响应生命周期，20次Mock失能通过 | [响应生命周期修复](../../areas-archive/lifecycle/records/2026-10-09-damiao-controller-response-lifetime.md)#F1-F2 | PARTIAL（T1；实机待验） |
| motion/F2 | 官方夹爪 controller 的 PP 扩展默认关闭；启用后按同序号反馈完成到位/取消，max_effort 使用 N，不接受无限力请求 | [PP Action 记录](../../areas-archive/motion/records/2026-09-08-pp-gripper-action.md) | PARTIAL，实机待验 |
| release-deploy/V3-HEAD-CAN-1 | V3 头部使用 `head_joint/head_pitch_joint`、非 JTC ForwardCommand + `/rt/head/*` manager；达妙 T 型由 ACC/DEC/MAX_SPD 承担，参数工具默认不写 flash | [V3 达妙头部非 JTC 位置速度运行链](../../areas-archive/release-deploy/records/2026-09-19-v3-damiao-head-runtime.md)#F1-F4 | PARTIAL（T1 Mock；CAN/机械/整机 full runtime 待验） |

| arm-control-20261014-01#F1–F3 | 显式批次/容差/包络保留旧数值与顺序，provisional 不冒充实测 | [显式手臂策略](records/2026-10-14-explicit-arm-policies.md) | UNVERIFIED（T0；策略/金样验证通过，实机未验） |
| arm-control-20261019-01#F1–F3 | JTC 在非实时回调拒绝超过 `max_trajectory_points`（默认 2000）的轨迹；2000 点时新轨迹周期约 1.17 ms | [JTC 点数上限](records/2026-10-19-jtc-max-trajectory-points.md) | PARTIAL（T1 mock；台架未发轨迹） |

| integration-20261014-03#F1–F4 | 控制包只依赖共享HeadPhase；控制器集合/参数保持，D-b不准入新真实头部 | [第四步记录](../integration/records/2026-10-14-head-force-composition.md) | PARTIAL（头部T1等价；实机未验） |

## 跨区事实关联

底层PREOP/PDO/semantic约束见[protocol](../protocol/README.md)；FJT/Rolling启动拼装见[integration](../integration/README.md)。本区负责controller实现，不扩大为硬件使能授权。

[arm-control-20261023-05](records/2026-10-23-dpi-tool-preset-groups.md)#F1-F3：新增吸盘装配T0/T2/T4六个大组，保留旧预设；静态/API验收，不授权运动，实际新采集待用户执行（PARTIAL，T2）。

[arm-control-20261023-06](records/2026-10-23-dpi-j6-measurement-sequences.md)#F1-F3：新增⑦J6已建模吸盘自重换向诊断及⑧固定姿态准备；不自动加载外力矩，0x2241待准备（PARTIAL，T2）。

[arm-control-20261024-01](records/2026-10-24-vacuum-web-controls.md)#F1-F3：临时真空三按钮复用现场脚本，右阀00、左阀01、泵02；网页轮询只读，开关不自动联动。105项测试与浏览器模拟通过，实际读回三路OFF，实机开关待验（PARTIAL，T2）。

[arm-control-20261027-01](records/2026-10-27-vacuum-pressure-display.md)#F1-F3：操作台右上角消费左右压力 ROS 话题，AI0=左、AI1=右，显示换算后的 kPa 表压；单路无效或超过 1 秒未更新清除数字，与泵阀和运动准入独立。107 项包测试、构建、质量门禁及真实页面只读验收通过（PASS，T2）。
