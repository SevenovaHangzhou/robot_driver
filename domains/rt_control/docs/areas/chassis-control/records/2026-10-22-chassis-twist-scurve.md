---
id: chassis-control-20261022-01
area: chassis-control
title: 舵轮控制器在逆运动学前对整车速度做 S 曲线平滑
date: 2026-10-22
type: feature
trigger: ELECTRI-127；用户要求为行走 CSV 增加整车速度 S 曲线
commits: [feature/electri-127-body-velocity-scurve]
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - 研华 v3/deployments/ELECTRI-127-body-scurve-20261023/runtime-check.json
  - 研华 v3/deployments/ELECTRI-127-body-scurve-20261023/install-transaction.json
  - 研华 v3/deployments/ELECTRI-127-body-scurve-20261023/controlword-zero.json
  - 研华 v3/runs/chassis-20261023T052345Z/launch.log
supersedes: []
related: [chassis-control-20261021-01]
---

## 背景

转向 CSP 已有 S 曲线；行走 CSV 原来只有逐轮加速度限制。
用户要求先平滑整车速度，再进行逆运动学分解。本分支基于对齐模式分支；
不包含 `/wheel/odom` 改名、IMU 接入或本体 TF 的独立改动。

## 改动

- 新增 `BodyVelocityShaper`，使用 Ruckig 固定三轴 Velocity 接口。
  输入为 `(vx, vy, ω·R)`，R 取现有几何中模块到车体原点的最大距离。
- 新参数 `max_body_acceleration`、`max_body_jerk` 为该向量的范数限值，单位分别为 m/s²、m/s³。
  每轴限值除以 √3，保证三个分量同时变化时仍满足范数限制。
  两项必须同时为有限正数，或同时为正无穷大。仅在 configure 时读取。
- 起步或其他共线状态使用相位同步；任意运动中改目标时，如无法保持相位同步，则使用时间同步。
  不承诺所有变向过程保持 vx:vy:ω 比例不变。
- 对齐等待时，只规划转向，行走目标为零；全部停驱且门限释放后才从零启动速度曲线。
  不使用非零种子速度。死区作用于输入指令，不截断曲线的起始/末尾小速度。
- 正常零速指令按 S 曲线减速。超时、对齐等待和故障停车保留原有策略，不为平顺性推迟保护。
  余弦补偿、轮速饱和、逐轮加速度限制仍存在，因此不承诺所有情况下的严格轮速协调或 CSV jerk 限制。
- 里程计仍取实际反馈，禁止把平滑指令当成测量值。
- 新增包依赖 `ruckig`，通过 package.xml/rosdep 安装；本地和研华均为 Humble 0.9.2。
- 用户 2026-10-23 要求先部署、再提交并开 PR。本次模板启用 1.0 m/s²、4.0 m/s³，
  作为现场初始调参值；这不代表停车距离、导航交互或实车平顺性已验收。
  参数声明默认仍为双 `.inf`，没有提供新参数的旧配置保持原行为。

## 验证

已执行（本地 Native，软件测试）：

- 受影响包构建成功；完整 `swerve_driver` 测试 173 项通过，0 errors/failures/skipped。
- `BodyShaperKeepsDirectionAndLimits` 检查从零起步、正常停车、低速修正、正反转及持续改目标。
  使用变化周期 0.8/1.2 ms 检查加速度范数和 jerk 范数，无末周期两倍放宽。
- `GatedStartWaitsThenRampsOnTheSCurve` 检查实际 CSV 输出：对齐等待、从零加速、零速停车、
  低速微调、运动中重新对齐和指令过期。门限等待期间不会积累一个高速度目标。
- 本地 20,000 次速度更新审计：C++ new/new[] 计数为 0；中位约 0.075 µs，P99 约 0.355 µs，
  最大约 27.112 µs。测试包括每 20 周期改目标。该数据不是研华 1 kHz 线程性能证据。
- 候选值 1.0/4.0 的纯软件计算：0.3 m/s 收零速后约 0.770 s 停止，位移约 0.1153 m。

已执行（T3，研华，2026-10-23）：

- 在独立部署目录构建 `swerve_driver`，不使用现场脏源码混编；目标机 173 项无设备测试全部通过。
  首次测试命令漏写 `--merge-install` 被 colcon 拒绝，补齐后通过，没有测试代码修改。
- 仅安装 `ros-humble-ruckig` 0.9.2，无其它包升级；仅替换安装清单中的 44 个包文件，原文件和部署产物已备份。
- 原位置重新 compose 后，部署产物只有 `controllers.yaml` 和 `artifact.yaml` 变化；
  新增两项整车平滑参数，其余几何、总线与门限不变。
- 上电后 8 台 EtherCAT 从站在线、CAN 500 kbit/s ERROR-ACTIVE。首次预检发现控制字均为 0x0006；
  经用户单独授权，仅写 8 台驱动的 0x6040:00=0x0000 并读回确认，随后原预检通过。
  未复位故障、未写 Flash、未自动使能或发送运动指令。
- 正式运行日志确认 `body velocity shaping: enabled, acceleration 1.000, jerk 4.000`，
  对齐模式仍为 5°/3°；参数服务读回一致；使能状态为 IDLE。
- 控制进程以 sev_v3 运行，实际加载新 `libswerve_controller.so`、`libswerve_math.so` 与 `libruckig.so`。
  更新线程 FIFO80/CPU7，EtherCAT-OP FIFO79/CPU7；网页 active、HTTP 200。
- 现场源码仅应用本分支增量，保留同事已有修改。安装包刻意未包含现场源码中尚未部署的 odom/TF 改动。

未验证：

- 使能后整车平滑的实时最坏执行时间、实车响应、停车距离和运动中故障停止。
- 导航原地旋转问题的根因，以及与导航速度平滑器叠加时的响应。
- 本体 TF 与里程计名称迁移不在本分支验证范围内。

## 部署与回退

研华部署目录 `v3/deployments/ELECTRI-127-body-scurve-20261023` 保存源快照、测试、安装事务和原产物备份。
源码快照 tar SHA-256：`4bfdce0df0e7f9307b078b4ca08c73c51f53cea8108943276cafe073a7442e8e`。
`install-package.py restore` 只在控制进程停止时恢复本次替换文件和旧部署产物。
也可将两项设为 `.inf`，重新生成部署产物后重启；禁止直接篡改带校验的生成文件。
控制器从启动配置读取限值，网页尚不提供这两项的调节入口。

本次用户授权安装、部署、启动及控制字清零；本记录不授权使能或运动。
