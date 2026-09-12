# 硬件验证状态

更新时间：2026-09-12

## 已完成的软件证据

| 项目 | 状态 | 证据 |
|---|---|---|
| MIT/反馈/寄存器协议 | 通过 | codec 逐字节单测 |
| 严格启动门（限幅/反馈/使能/p_m） | 通过（内存总线） | `test_control_loop`, `test_swerve_driver_node` |
| 单周期 8 帧批发 | 通过（内存总线） | `test_control_loop` |
| 实测 twist、缺帧位移与协方差 | 通过（内存总线） | `test_odometry`, `test_odometry_covariance`, `test_control_loop` |
| 单电机失联全车停车与自动恢复上限 | 通过（注入时钟与丢帧） | `test_safety`, `test_control_loop` |
| 故障分类、锁存、人工清错复验 | 通过（注入故障/ACK） | `test_safety`, `test_control_loop`, `test_diagnostics` |
| transport 异常安全窗口与 stale 队列恢复 | 通过（注入 write/collect/closed） | `test_control_loop` |
| IMU/cmd 时间戳与姿态输入校验 | 通过（边界输入） | `test_control_loop`, `test_imu_validation` |
| 超时舵角保持与故障态转向保持 | 通过（内存总线） | `test_control_loop`, `test_swerve_module` |
| ±180° 有限舵角分支、margin、量测 tolerance、输出与启动门 | 通过（纯函数/内存总线） | `test_kinematics`, `test_swerve_module`, `test_control_loop`, `test_safety`, `test_diagnostics` |
| rezero 保存确认与独立零位回读 | 通过（内存总线） | `test_steering_rezero`, `test_swerve_driver_node` |
| lifecycle、diagnostics、服务 | 通过（ROS client） | `test_swerve_driver_node`, `test_diagnostics` |
| Linux vcan 成功 I/O | 未执行 | 当前主机内核 `CONFIG_CAN_VCAN` 未启用；测试按条件 skip |
| Kinco DS402/CSP/CSV、BRT 主备与持久化 | 通过（应用层 fake） | `test_kinco_*`、`test_external_steering_encoder`、`test_canopen_encoder` |
| 外置编码器校验先于使能、掉线备份、过流全车零速 | 通过（跨组件冒烟） | `test_kinco_runtime` |
| IgH 1.6 用户库实际编译/链接 | 通过（无硬件） | 官方源码提交 `2f7f884f1c7d377c02a7d627eb06512126a0e50e`，`DM_SWERVE_ENABLE_IGH=ON` 节点构建通过 |
| 差模打滑轮定位/剔除及协方差 | 通过（纯函数/内存总线） | `test_kinematics`、`test_control_loop`、`test_odometry` |

## 必须由现场硬件完成

以下项目没有在本开发环境执行，不能视为通过：

| 项目 | 状态 | 主要风险 |
|---|---|---|
| 单电机真实 CAN 收发 | 待执行 | 适配器/终端/波特率 |
| PMAX/VMAX/TMAX 实机回复格式 | 待执行 | 协议文档该处信息最薄弱 |
| p_m 断电持久性 | 待执行 | G_s>1 时绝对角模糊 |
| TIMEOUT 0x09 易失/保存语义 | 待执行 | 固件版本差异 |
| 运动中物理拔 CAN 后按时失能 | 待执行 | 50 µs/计数假设及实测延迟 |
| 单模块力矩与方向 | 待执行 | 齿比、invert、机械装配 |
| π 边界有限路径、机械 margin 与量测 tolerance | 待执行 | 真实关节限位、背隙、零偏、端点噪声和 PMAX 映射 |
| 整车架起 8 帧时间散布 | 待执行 | 实际 CAN qdisc/ENOBUFS |
| 单电机失联触发全车停车 | 待执行 | 实际总线时序、反馈阈值 |
| 欠压/通信恢复次数上限 | 待执行 | 固件清错/重使能语义 |
| 硬件错误锁存与人工解锁 | 待执行 | 各错误码的实机注入方式 |
| 落地轮径和前馈标定 | 待执行 | 轮胎、载荷、地面打滑 |
| Kinco/BRT 实际 PDO、心跳、双绝对源和标零 | 待执行 | 见 `kinco_bringup.md`，软件 fake 不代表总线协议验收 |
| FD DC 周期、PDO watchdog、0x605E 和物理断链停止 | 待执行 | 实机固件及总线时序 |
| 0x60FB 内部值与工程量、CSV 低速跟踪 | 待执行 | 前馈写入默认关闭；CST 闭环仅在 CSV 实测不足时追加 |

`write_timeout_register` 已按 Phase 7 策略默认开启，但这不等于 0x09 协议假设已经通过实机验收。现场人员应按 `hardware_bringup.md` 顺序执行，并把结果、日志路径、测试人和日期写回本文件。物理拔线停车（P0-07）只能由该现场记录闭环，软件 fake 测试不能代替。软件 transport fault 测试只证明上位机在异常后保持零速窗口，不能替代电机固件 TIMEOUT 测试。

## 不改代码的验收口径

- P0-01：当前机械尺寸、齿比和前馈值是有意保留的占位参数；按 `calibration.md` 完成实车标定后再替换，不能把占位值当作硬件验收结果。
- P0-07：物理断线停车不能由上位机 fake 测试证明；必须在单电机台架/整车现场确认固件 TIMEOUT，并记录最后一帧到失能的实测时间。
- P1-02：20° 对齐门是既定控制设计和可调参数；导航连续性由导航层调参处理，本次不绕过该门。
