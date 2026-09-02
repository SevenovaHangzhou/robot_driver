# 硬件验证状态

更新时间：2026-08-25

## 已完成的软件证据

| 项目 | 状态 | 证据 |
|---|---|---|
| MIT/反馈/寄存器协议 | 通过 | codec 逐字节单测 |
| 8 电机启动与 fallback | 通过（内存总线） | `test_control_loop` |
| 单周期 8 帧批发 | 通过（内存总线） | `test_control_loop` |
| 直线/圆弧里程计 | 通过（内存总线） | `test_integration_fake_bus` |
| cmd/IMU/单电机/总线降级恢复 | 通过（注入时钟与丢帧） | `test_safety`, `test_control_loop` |
| lifecycle、diagnostics、服务 | 通过（ROS client） | `test_swerve_driver_node`, `test_diagnostics` |
| Linux vcan 成功 I/O | 未执行 | 当前主机内核 `CONFIG_CAN_VCAN` 未启用；测试按条件 skip |

## 必须由现场硬件完成

以下项目没有在本开发环境执行，不能视为通过：

| 项目 | 状态 | 主要风险 |
|---|---|---|
| 单电机真实 CAN 收发 | 待执行 | 适配器/终端/波特率 |
| PMAX/VMAX/TMAX 实机回复格式 | 待执行 | 协议文档该处信息最薄弱 |
| p_m 断电持久性 | 待执行 | G_s>1 时绝对角模糊 |
| TIMEOUT 0x09 易失/保存语义 | 待执行 | 固件版本差异 |
| 单模块力矩与方向 | 待执行 | 齿比、invert、机械装配 |
| 整车架起 8 帧时间散布 | 待执行 | 实际 CAN qdisc/ENOBUFS |
| 落地轮径和前馈标定 | 待执行 | 轮胎、载荷、地面打滑 |

现场人员应按 `hardware_bringup.md` 顺序执行，并把结果、日志路径、测试人和日期写回本文件。
