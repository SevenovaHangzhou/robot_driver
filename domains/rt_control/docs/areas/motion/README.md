# motion — 轨迹与运动执行

**范围**：whole_body_jtc 与 FJT 执行、diff_drive 底盘速度、控制器参数
（`controllers.yaml`）、关节限位（`joint_limits.yaml`）、控制环频率配置。
**Owner 包/资产**：`src/rt_control/rt_control_bringup` 的控制器/限位配置、
`patches/ros2_controllers`、`src/rt_control/rt_arm_dynamics`、
`src/rt_control/gravity_ff_controller`。

不属于本区：轨迹规划与任务编排（运控域，不在本仓库）、使能状态机（→ lifecycle）、
launch 结构与容器（→ release-deploy）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| F1 | `diff_drive_controller.wheel_separation=0.95089496 m`，为用户明确要求的有效轮距参数 `1.9598 * 0.4852`；该值不等同于已实测物理轮距 | BQ-140；[2026-08-24 记录](records/2026-08-24-track-effective-separation-95089496mm.md) | 已裁决，待 T4 实车转向/里程计复验 |
| F2 | 官方夹爪 controller 的 PP 扩展默认关闭；启用后按同序号反馈完成到位/取消，max_effort 使用 N，不接受无限力请求 | [PP Action 记录](records/2026-09-08-pp-gripper-action.md) | PARTIAL，实机待验 |
| F3 | 三代机转向 CSP、驱动 CSV；外置编码器用于实际舵角观测与校验，不增加转向位置外环或运行中慢速修正 | [CSP 舵角决策](records/2026-09-08-swerve-csp-feedback-decision.md)；BQ-144 | 已裁决，待实现/实机验证 |
| F4 | swerve_driver 算法与 ros2_control 插件已迁入，只申请 4 路位置与 4 路速度命令，不含 rezero/MIT/力矩前馈 | [源码迁移记录](records/2026-09-09-swerve-controller-migration.md) | PARTIAL（T1 Native/容器；真实后端待接） |
| F5 | HT-WS-HH270 厂家图纸已登记名义轮径、传动、安装包络和悬挂参数；数据保持 draft，不能替代逐轮有效半径、零位、硬限位和背隙标定 | [厂家机械图纸提取](records/2026-09-15-swerve-module-mechanical-drawing.md)#F1-F4 | UNVERIFIED（T0 图纸提取；厂家澄清/实测待完成） |
| F6 | 四轮速度使用固定规模最小二乘残差剔除；同一异常轮从 twist 与位置里程计排除且恢复无补跳，阈值保持待标定；共模一致误差不可辨识 | [残差剔除记录](records/2026-09-16-swerve-slip-residual-rejection.md)#F1-F4 | PARTIAL（T1 Native/Mock；阈值和实机打滑待验） |
| F7 | 舵轮只做逐轮机械区间内的正/反轮速分支选择和线性 slew；不存在 continuous-joint 模式，输出层不二次选支或静默 clamp | [有限转向记录](records/2026-09-16-swerve-bounded-steering-planner.md)#F1-F4 | PARTIAL（T1 Native/Mock；逐轮限位与实机跟踪待验） |
| F8 | 七轴重力前馈以 Pinocchio reduced model 输出 N.m，shadow 无命令，active 独占 effort 并受模型哈希/逐轴换算双门禁；未接默认启动面 | [ELECTRI-136 离线实现](records/2026-09-19-electri-136-gravity-feedforward-offline.md)#F1-F4 | PARTIAL（T1 Docker/Mock 定向测试；全量测试/镜像/实机待验） |
| F9 | 2026-09-24 机械重导的惯量必要条件与 JSON→URDF 转换通过 T0 检查，但左右质量属性/网格装配变换不一致，末端归属及公共模型迁移待决；不能据此验证或替换生产模型 | [机械参数核对](records/2026-09-24-electri-136-mechanical-inertia-review.md)#F1-F3 | UNVERIFIED（机械/模型所有者待确认，BQ-152 保持开放） |
| F10 | UR/Doosan 官方接口明确支持负载渐变，ABB 明确支持抓放工件负载切换；辨识、档案选择、时间渐变与实时承重估计须区分，不能据厂商 API 推导本机 60B2 准入 | [厂商负载调查](records/2026-09-24-electri-136-payload-vendor-survey.md)#F1-F4 | UNVERIFIED（仅公开资料调查，未实现/实机验证） |
| F11 | 条件性 COM 换算对比：右臂 J1 采样差约 5.48N·m、解析上界约 5.94N·m，J7 约 0.82N·m，部分候选姿态符号改变；不是已确认实机误差 | [质心力矩敏感性](records/2026-09-24-electri-136-com-gravity-sensitivity.md)#F1-F3 | UNVERIFIED（离线数值交叉验证通过，机械来源/实机待验） |
| F12 | 用户接受新机械数据作为待标定CAD初值，并授权每侧J1零位对齐的临时FK映射；16条惯性参数已进入模型，原始文件与矩阵保留 | [临时FK惯性合入](records/2026-09-25-electri-136-inertia-fk-integration.md)#F1-F3 | UNVERIFIED（数据已合入，hardware_verified仍为false） |
| F13 | RT-Control副本、source-lock及机器绑定固定独立模型feature分支11f6d90，V3.1.1运动学与接口不变；两种末端共享新J1–J6，只有夹爪末端更新J7/活动件 | [临时FK惯性合入](records/2026-09-25-electri-136-inertia-fk-integration.md)#F1-F3 | UNVERIFIED（模型/bringup 122项、相关配置43项通过；插件/外部消费者/实机待验） |

## 记录索引（倒序）

- [2026-09-25 ELECTRI-136 新机械惯性初值按授权临时FK映射合入](records/2026-09-25-electri-136-inertia-fk-integration.md)

- [2026-09-25 ELECTRI-136 按用户选择迁移公共V3.1.1模型基线](records/2026-09-25-electri-136-v311-model-migration.md)
- [2026-09-25 ELECTRI-136 新机械惯性初值来源接收与模型迁移边界](records/2026-09-25-electri-136-inertial-source-intake.md)
- [2026-09-24 ELECTRI-136 质心装配疑点的重力力矩敏感性](records/2026-09-24-electri-136-com-gravity-sensitivity.md)
- [2026-09-24 ELECTRI-136 商业机械臂负载切换、渐变与辨识调查](records/2026-09-24-electri-136-payload-vendor-survey.md)
- [2026-09-24 ELECTRI-136 机械重导双臂惯性参数核对](records/2026-09-24-electri-136-mechanical-inertia-review.md)

- [2026-09-19 ELECTRI-136 七轴 CSP 重力前馈离线库与控制器](records/2026-09-19-electri-136-gravity-feedforward-offline.md)
- [2026-09-16 四舵轮机械限位内转向规划与输出门禁](records/2026-09-16-swerve-bounded-steering-planner.md)
- [2026-09-16 四舵轮最小二乘残差剔除与里程计隔离](records/2026-09-16-swerve-slip-residual-rejection.md)
- [2026-09-15 HT-WS-HH270 舵轮厂家机械图纸提取](records/2026-09-15-swerve-module-mechanical-drawing.md)
- [2026-09-09 swerve_driver 实际源码迁移](records/2026-09-09-swerve-controller-migration.md)
- [2026-09-08 舵轮保留转向 CSP 与外置舵角观测](records/2026-09-08-swerve-csp-feedback-decision.md)
- [2026-09-08 官方夹爪 PP Action 扩展](records/2026-09-08-pp-gripper-action.md)

- [2026-08-24 履带有效轮距调整为 0.95089496 m](records/2026-08-24-track-effective-separation-95089496mm.md)

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-019 三条（partial / corrective / 14-axis minimal FJT）
- `docs/fjt-14axis-low-speed-commissioning-20260727.md`
- 相关 BQ（不完全）：BQ-119、BQ-120、BQ-122
