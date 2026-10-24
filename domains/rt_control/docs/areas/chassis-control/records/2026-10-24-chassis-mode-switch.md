---
id: chassis-control-20261024-01
area: chassis-control
title: 导航保持使能时停稳切换对齐或连续模式
date: 2026-10-24
type: feature
trigger: ELECTRI-127；用户选择停稳后切换，不重启、不失能，运动中拒绝
commits: [feature/electri-127-chassis-mode-runtime]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - 研华 v3/deployments/ELECTRI-127-chassis-mode-20261024/runtime-check.json
  - 研华 v3/deployments/ELECTRI-127-chassis-mode-20261024/install-transaction.json
  - 研华 v3/deployments/ELECTRI-127-chassis-mode-20261024/source-merge.json
  - 研华 v3/runs/chassis-20261024T114838Z/launch.log
supersedes: []
related: [chassis-control-20261021-01, chassis-control-20261022-01]
---

## 行为

运行时切换门控策略，不重建 ControlCore，不复位参考位置/速度/加速度、分支、门控锁存、保持角或整车平滑器。
正常停驱规则、S 曲线、看门狗和故障处理不变。模式切换不是驱动器 CSP/CSV 切换。

- 对齐模式使用 `align_first_threshold` / `align_first_release`（当前配置 5°/3°）。
- 连续模式使用 `alignment_threshold`（当前配置 90°，保留余弦补偿）。
- 只有控制器已激活、无锁存故障、导航拥有执行权时允许请求。
- 控制周期必须看到新鲜、显式的零速请求。
- 四个驱动参考必须为零，四个有效实测轮速连续满足已有 `fault_stop_speed` / `fault_stop_hold`。
  当前默认为轮缘 0.01 m/s、持续 0.05 s，复用既有门限，不新增物理标定数值。
- 电机和外置编码器反馈必须有效且一致。缺失或过期不能被当作停稳。
- 运动中拒绝；不自动停车，不把请求排队等待以后停车。
- 控制周期确认生效后才返回成功。模式更新后，已接收的缓存速度指令失效，等待新的本地接收请求。
- `drive_mode` 参数只表示启动默认值，仅在 configure 前允许设置；配置后直接设置会被拒绝。实际模式从专用状态话题读取。
- 普通 deactivate/activate 保留运行模式；重新 configure 或进程重启恢复部署默认。运行模式不写现场调参文件。

## 公共契约候选

在独立 `robot_interfaces` 工作树登记，不新增自定义 IDL，也不复制 RT 私有接口：

| 编号 | 端点 | 标准类型 | 语义 |
|---|---|---|---|
| R-IN-18 | `/control/set_chassis_align_first` | `std_srvs/srv/SetBool` | `data=true` 对齐，`false` 连续；成功才确认实际应用 |
| R-OUT-28 | `/rt_control/chassis/align_first` | `std_msgs/msg/Bool` | `true` 对齐，`false` 连续；Q_LATCHED，变化立即发布，稳定 1 Hz |

提供方为 RT-Control，调用/消费方为 Motion（Navigation）。状态只表示模式，不表示使能、停稳或运动准入。
公共工作树基于 RT 已锁定的 `ea582c8`。候选提交为
[`652973e`](https://github.com/SevenovaHangzhou/robot_interfaces/commit/652973e52c271ba6ea018f663136a7718ab575a4)，
远端分支 `feature/electri-127-chassis-drive-mode`。公共主线尚未收录这个依赖候选；未修改下游锁定 SHA。
本增量仅使用标准 ROS 类型，但正式接入仍必须完成契约评审、双方注册与发布身份确认。

## 导航使用方式（部署后）

1. 导航的唯一速度发布者持续发送零速，等待底盘停稳；不要同时另起一个 `/cmd_vel` 发布者。
2. 调用模式服务。控制栈无需停止，驱动无需失能。
3. 收到 `success=true` 后，再发送新的运动指令，并同步预测器的实际模式。
4. 收到 `success=false`、服务超时或响应丢失时，保持零速，核对新鲜的模式状态；禁止解析 message 作运动准入。
   失败响应不证明模式一定未变化：结果可能已经应用但确认未送达。
5. 禁止自动循环重试；确认反馈和停稳条件后，再由调用方显式请求。

```bash
# 对齐模式
ros2 service call /control/set_chassis_align_first std_srvs/srv/SetBool '{data: true}'

# 连续模式
ros2 service call /control/set_chassis_align_first std_srvs/srv/SetBool '{data: false}'

# 读取实际模式；仍需结合反馈新鲜度、使能状态等判断运行条件
ros2 topic echo /rt_control/chassis/align_first std_msgs/msg/Bool
```

服务采用延迟响应：非实时回调不等待控制周期，不阻塞同一执行器中的速度订阅；
RT 只处理固定大小请求和原子确认，回复与状态发布在非实时计时器中完成。
传输请求有效期为 1 s，不是自动等待停稳的时间。请求过期后尝试原子取消；
已由 RT 认领但尚未确认的请求报告结果未知，由状态话题核对，不宣称取消成功。
`Twist` 无源时间/序号：新命令以 RT 本地接收时刻判断，不能识别任意网络上的源端迟到指令；
调用方必须保持单一写入者和确认前零速。

## 实现与验证

- 执行核心只更新对齐策略，不重建生成器或平滑器；参考历史对照测试通过。
- 模式服务在控制周期边界检查停稳条件并应用；原子取消防止未执行请求在超时后迟到应用。
- 网页订阅实际模式；状态过期时显示未知并禁用门限保存。旧控制器没有此话题时保留启动参数显示。
- 本地 `swerve_driver` 构建及 177 项软件测试通过（0 errors/failures/skipped）。
- 新增 4 项测试：历史保持和反馈拒绝；运动中拒绝与停稳双向切换；无 RT 周期时超时且不迟到生效；启用整车 S 曲线时，模式切换后 deactivate/activate 仍保留实际模式。
- 回归曾发现模式重载会拒绝未改变的合法无限转向加速度/jerk；修复为仅对改变的运动参数重新执行调参上限检查，核心构造校验仍保留。
- 网页既有 126 项测试、JS 语法检查及仓库质量门禁通过。模式变化时刷新门限输入，避免把前一模式的编辑值套给新模式。
- 公共仓库生成文档/契约/错误码/变更日志门禁通过；63 项工具测试通过；6 包全量构建通过。

## 研华联调部署（2026-10-24）

用户明确要求部署。本次在公共契约候选尚未合并的情况下进行标准类型端点联调部署，未修改正式依赖 SHA，未宣称公共版本已发布。

- 初始底盘为 IDLE，无 `/cmd_vel` 发布者；在独立目录构建驱动和网页，研华 colcon 汇总 309 项无设备测试通过。
- 原控制栈有序退出：运动停用、驱动失能、控制器停用、两类硬件清理均成功。
- 替换安装清单中的 79 个文件并保留原副本；安装后逐文件 SHA-256 核对一致。
- 生成配置全部保持原字节，模式默认 align_first、整车平滑 1.0/4.0、停稳门限 0.01 m/s 与 0.05 s 不变。
- 现场源码与待部署 odom 改名在同一插入点冲突；经三方对照仅保留该已知改名并合入模式切换。
  未覆盖其它现场改动；本次安装仍由独立候选构建，不把待部署 odom/TF 一起上线。
- 启动至 `RT_SCOPE_HARDWARE_ACTIVE_DISABLED`，实际模式状态为 true。服务调用 false 时因控制器未激活返回 success=false，未改变模式。
- 网页 HTTP 200；认证后的只读 WebSocket 返回实际模式 align_first、状态 IDLE、enabled=false。
- 更新线程 FIFO80/CPU6，EtherCAT-OP FIFO79/CPU6，与本次部署前运行日志一致。没有改绑核配置。
- 候选包 SHA-256：`96501b8f1a2917a931dc7ff7701929ef556f113a7f6b4782ac2164e56ed45ffc`。
- 回退目录 `v3/deployments/ELECTRI-127-chassis-mode-20261024/backup`；停栈后运行该目录同级的
  `install-package.py restore` 恢复安装产物。回退脚本拒绝在 controller manager 运行时替换库。

未验证：真实使能状态下成功切换、导航端调用与模式预测同步、运行中故障或网络中断下的跨进程验收。
本次没有自动使能或发送速度指令，也没有执行 SDO 写入或故障复位。
实现及公共契约分别通过功能分支交付；公共契约仍需双方评审和正式发布。

本记录不授权使能或运动。
