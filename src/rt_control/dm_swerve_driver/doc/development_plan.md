# ROS2 Humble 四舵轮底盘驱动 — dm_swerve_driver 指导计划

**分工**:本文档是给 **Codex(实现者)** 的执行指导;**Claude 负责每阶段 review**(评审清单见文末)。

**安全设计总原则(2026-09-03 修订,依据验收决议全面转为严格策略;下文各节中与本节冲突的"宽松"表述以本节为准)**:

- **启动全通过才放行**:8 台电机必须全部完成限幅寄存器读取成功(禁止 fallback 放行,重试后仍失败 → initialize 失败、activate 失败)、有效反馈、使能确认、转向绝对角初始化(G_s>1 时 p_m 必须可读且与主值一致,否则拒使能并提示 rezero),才允许控制环接收非零指令
- **单电机失联/故障 = 全车停**:任一电机连续静默超阈值或报硬件故障 → 全部驱动轮速置零(转向保持)并进入故障态,不再是"其余 7 台照跑"
- **故障锁存 + 分类恢复**:按 ERR 类型区分——欠压/通信类允许自动清错重使能,**次数上限(默认 3 次)**,超限锁存;过流/过温/编码器/过载一律**立即锁存**,只能通过 `~/clear_faults` 人工恢复,恢复前全车保持停止
- 保留:cmd_vel 超时置零自恢复、IMU 降级轮式 yaw(附协方差膨胀)、转向对齐门控(控制品质手段)
- 电机固件 TIMEOUT 寄存器仍是 CAN 物理断线的最终兜底,实机确认寄存器语义后默认开启写入,验收含物理拔线实测

## Context(背景)

基于达妙(DaMiao)电机做四舵轮(swerve)底盘的 ROS2 Humble 控制驱动,CAN 通信,机械参数待标定(全部占位符化)。仓库 `/home/kkozia/damiao` 是导航壳(50 个 submodule 均未初始化,几乎无代码),但本地 `电机产品/中空电机/DM-G8518/DM-G8518 中空云台电机使用说明书 V1.0.pdf` 含达妙全系通用的完整 CAN 协议,协议信息已提取齐全(见下节),无需 submodule。

已确定的四项架构决策:

1. **独立自定义 C++ 节点**(不用 ros2_control)——固定周期控制循环,对 CAN 写入时序完全可控
2. **协议层按文档自实现**(达妙 Gitee 仓库无 LICENSE,官方 SDK 只作参考)
3. **全部 MIT 模式**——唯一能注入力矩/速度前馈的模式
4. **yaw 订阅 sensor_msgs/Imu 话题**(可配置话题名,IMU 型号解耦)

六项强制技术要求(源自 WPILib 实践):① 里程计用位置增量+位姿指数,不用速度积分;② yaw 直接用陀螺,轮子只解平移;③ cmd_vel 先做 discretize 离散化补偿;④ 转向环 Kff·ω 前馈 + 驱动环 ks/kv/ka 前馈;⑤ 转向对齐硬门控(max|Δθ|>阈值时轮速全零);⑥ 8 帧指令同周期原子下发。

## 达妙 CAN 协议要点(已从本地 PDF 提取验证)

- CAN 2.0 标准帧 11-bit ID,默认 1 Mbps;电机**轮询式**反馈:每收 1 条指令帧回 1 条反馈帧(ID=MST_ID)
- **MIT 指令帧**(ID=ESC_ID, 8 字节,大端位域):`D0=p[15:8], D1=p[7:0], D2=v[11:4], D3=(v[3:0]<<4)|kp[11:8], D4=kp[7:0], D5=kd[11:4], D6=(kd[3:0]<<4)|tff[11:8], D7=tff[7:0]`;p 16 位、v/kp/kd/tff 12 位;kp∈[0,500]、kd∈[0,5] 固定,p/v/t 按 [-PMAX,PMAX] 等区间线性映射
- **反馈帧**:`D0=ID|ERR<<4, D1-2=POS(16b), D3-4=VEL(12b), D4-5=TORQUE(12b), D6=T_MOS, D7=T_Rotor`;ERR: 0=失能 1=使能 2/5=编码器 8=过压 9=欠压 A=过流 B=MOS过温 C=线圈过温 D=通信丢失 E=过载
- **特殊命令**(ID=ESC_ID, `FF×7 + xx`):0xFC 使能 / 0xFD 失能 / 0xFE 保存零点 / 0xFB 清错。上电默认失能;上电后位置回绕到 [-π,π]
- **寄存器访问**(ID=0x7FF):读 `D0=CANID_L, D1=CANID_H, D2=0x33, D3=RID`,写 `D2=0x55, D4-7=数据(LE)`,回复走 MST_ID。关键寄存器:0x15/0x16/0x17=PMAX/VMAX/TMAX(缩放常数,**必须开机读回**)、0x09=TIMEOUT(失联自动失能看门狗)、0x50=p_m(精确多圈位置,只读)
- 总线预算:100 Hz × 8 电机 × (1 发 + 1 收) = 1600 fps ≈ 20% @1Mbps,单总线足够

## 包结构(新建独立 colcon 包,不依赖 submodule)

```
dm_swerve_driver/
├── CMakeLists.txt / package.xml / README.md
├── config/swerve_params.yaml            # 全部占位参数,唯一标定入口
├── doc/calibration.md                   # 标定流程文档
├── include/dm_swerve_driver/*.hpp
├── src/
│   ├── dm_frame_codec.cpp     (~250 行) # 纯函数:MIT 编码/反馈解码/寄存器/特殊帧
│   ├── socketcan_interface.cpp(~220 行) # open/filter/批量写/poll 收集
│   ├── dm_motor.cpp           (~300 行) # 单电机:多圈解绕、健康状态、限幅
│   ├── swerve_kinematics.cpp  (~250 行) # IK/FK/discretize/optimize/desaturate(纯数学)
│   ├── swerve_odometry.cpp    (~150 行) # 位姿指数 + 陀螺 yaw 融合(纯数学)
│   ├── swerve_module.cpp      (~250 行) # 模块=转向电机+驱动电机+前馈+单位换算
│   ├── control_loop.cpp       (~350 行) # 专用线程:算→原子发8帧→收8帧→里程计→发布
│   ├── safety_monitor.cpp     (~200 行) # 看门狗/降级标志/自动清错重使能调度/diagnostics(无锁存态)
│   ├── params.cpp             (~250 行) # 参数声明/加载/校验
│   └── swerve_driver_node.cpp (~300 行) # rclcpp 节点:订阅/发布/服务/生命周期
├── tools/fake_motor_sim.cpp   (~300 行) # vcan0 上的 8 电机模拟器(集成测试用)
├── launch/swerve_driver.launch.py
└── test/  (test_frame_codec / test_kinematics / test_odometry /
            test_motor_unwrap / test_safety / test_integration_vcan)
```

codec/kinematics/odometry 为无 ROS 依赖的纯 C++,便于 gtest 达到 80% 覆盖。

## 线程模型

- **执行器线程**(单线程 executor):cmd_vel / IMU 回调只做"带时间戳存入互斥保护邮箱"
- **控制线程**(activate 时创建的 std::thread):`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 定频 100 Hz(可配),可选 SCHED_FIFO(参数 `realtime_priority`,失败仅告警)。**不用 rclcpp timer**(避免执行器抖动)
- **同步收发,不开独立 RX 线程**:协议是 1 发 1 收轮询式。每周期:`write_batch` 连续写 8 帧(要求⑥的原子下发)→ `collect(expect=8, deadline=4ms)` 用 poll() 循环收齐。8 帧反馈最坏 ~2-3 ms,100 Hz 下余量 >5 ms
- 反馈缺失:按 MST_ID 匹配,超时未到的电机 `missed_count++`(仅计数+告警,处置见安全策略节);当周期缺反馈的模块**从 FK 均值中剔除**(而不是按 Δd=0 计入)

## 核心数学(REP-103:x 前 y 左,CCW 正;`wrapPi(a)=atan2(sin a, cos a)`)

### ① discretize(逆位姿指数)——要求③

```
θ = ω·dt, h = θ/2
B = |θ|<1e-9 ? 1 − θ²/12 : h·sin(θ)/(1−cos(θ))     # = h·cot(h)
vx_d =  B·vx + h·vy
vy_d = −h·vx + B·vy
ω_d  = ω
```

校验:vx>0 且 ω>0(CCW)时 vy_d<0。等价 WPILib `ChassisSpeeds.discretize`。

### ② 逆运动学 + 去饱和 + 门控——要求⑤

模块 i 位于 `(rx_i, ry_i)`:`vix = vx_d − ω_d·ry_i; viy = vy_d + ω_d·rx_i; speed=hypot, angle=atan2`(速度低于死区时保持上次角度)。去饱和:max speed 超 `v_wheel_max` 时全体等比缩放。

**对齐硬门控**:optimize 后 `err_i = wrapPi(target_i − measured_i)`,若 `max|err_i| > align_threshold`(默认 0.349 rad ≈ 20°)→ 本周期所有 speed:=0(转向照常指令)。余弦补偿始终叠加:`speed_i *= max(0, cos(err_i))`。

### ③ 位置增量正解(Δθ 由陀螺给定)——要求①②

```
Δd_i = distance_i(k) − distance_i(k−1)        # 驱动轮累计行驶距离差分
θ̄_i  = 本周期转向角(可用回绕安全的中点角)
Δθ   = wrapPi(yaw_gyro(k) − yaw_gyro(k−1))    # 陀螺,不用轮子反解
Δx = (1/N)·Σ(Δd_i·cosθ̄_i + Δθ·ry_i)          # N = 本周期有有效反馈的模块数
Δy = (1/N)·Σ(Δd_i·sinθ̄_i − Δθ·rx_i)
```

### ④ 位姿指数更新——要求①

```
s = |Δθ|<1e-9 ? 1 − Δθ²/6 : sin(Δθ)/Δθ
c = |Δθ|<1e-9 ? Δθ/2      : (1−cos(Δθ))/Δθ
tx = Δx·s − Δy·c;  ty = Δx·c + Δy·s
(x,y) += Rot(θ₀)·(tx,ty);  θ = yaw_offset + yaw_gyro(k)   # yaw 直接取陀螺
```

### ⑤ 角度优化 + 有限关节指令

```
safe_range = [joint_limit_min + margin, joint_limit_max − margin]
forward = {wrapPi(target)+2kπ | 候选在 safe_range 内}
reverse = {wrapPi(target+π)+2kπ | 候选在 safe_range 内}   # 同时反转轮速
按 |candidate−current| 的线性区间距离和滞回选择分支
target_next = target_prev + clamp(selected−target_prev, ±max_slew·dt)
err = selected−current                                    # 不回绕,供门控/余弦补偿
motor_target = (target_next + θ₀_i)·G_s·sign_s            # 超出 PMAX 显式失败,禁止静默钳位
```

电机输出层只做换算和校验,不得基于 PMAX 再次选择 ±π 分支。

### ⑥ MIT 字段映射——要求④

- **转向**:`p_des=motor_target, v_des=clamp(Kff·ω_target_axis·G_s, ±VMAX), kp=steer_kp, kd=steer_kd, t_ff=0`;ω_target 由相邻两周期有界 target 差分
- **驱动**:`p_des=0, kp=0, v_des=clamp(v_wheel/r_w·G_d·sign_d, ±VMAX), kd=drive_kd`(kd 兼作速度环增益,硬件限 [0,5]),`t_ff=clamp((ks·sgn(v)+kv·v+ka·a)/G_d, ±TMAX)`;a 由限斜率的目标速度差分得到

### ⑦ 转向多圈解绕与开机初始化

反馈相邻采样差 > PMAX → turns−1,< −PMAX → turns+1;`unwrapped = raw + turns·2·PMAX`。

- G_s=1:`wrapPi(反馈) − θ₀` 即可
- G_s>1:开机读 0x50 p_m 播种解绕器;校验 `|wrapPi(p_m) − wrapPi(反馈)| < 0.1 rad`,不符打 WARN 提示建议重标零,**照常使能**。提供 `~/rezero_steering` 服务(轮子用治具对正、失能状态下发 0xFE)

## YAML 参数纲要(config/swerve_params.yaml,全部占位)

- `can`: interface(can0/vcan0), feedback_deadline_us=4000, write_timeout_register=false, timeout_register_ms=100
- `control`: rate_hz=100, realtime_priority=0, cmd_vel_timeout_s=0.25, hold_steer_on_timeout=true
- `chassis`: wheelbase_m / track_m / wheel_radius_m(占位), max_wheel_speed_mps, max_linear/angular, max_wheel_accel_mps2(喂 ka), v_deadband_mps, **align_threshold_rad=0.349**
- `limits_fallback`: p_max/v_max/t_max(占位;仅当读不回 0x15/16/17 时作后备缩放值,读回不符只 WARN)
- `steering`: gear_ratio, kp=30, kd=1.0, **kff_omega=0.9**, zero_offset_rad[4], invert[4]
- `drive`: gear_ratio, kd=2.0, **ks/kv/ka**(占位), invert[4]
- `motors`: steer/drive 各 4 个 esc_id + mst_id(顺序 FL FR RL RR)
- `safety`: feedback_silent_cycles=50(转入补使能重试的阈值), reenable_period_s=1.0
- `odometry`: imu_topic=/imu/data, imu_timeout_s=0.2(超时转轮式反解降级,非停车), publish_tf, odom/base frame, publish_rate_hz=50

模块坐标由 wheelbase/track 推得:FL(+L/2,+W/2), FR(+L/2,−W/2), RL(−L/2,+W/2), RR(−L/2,−W/2)。

## 启动 / 停机序列

**启动**(控制线程内,循环前):

1. 开 socket + RX 过滤(8 个 MST_ID + 0x7FF 回复)。打不开 socket 是唯一的启动即退出条件
2. 逐电机读 PMAX/VMAX/TMAX,**以读回值为准直接用于编解码缩放**(这才是电机真实用的值);与 YAML 期望值不符只打 WARN,不拒绝使能。读不回来(重试 3 次)则退用 YAML 值 + WARN
3. 可选(参数门控)写 TIMEOUT 寄存器 0x09
4. 转向电机读 p_m 播种解绕器;与反馈主值交叉校验,不符打 WARN(提示可能需要 rezero),**照常使能**。读不到 p_m 时用反馈主值播种(G_s=1 时完全等价,G_s>1 时 WARN 提示角度可能有 2π/G_s 模糊)
5. 驱动电机以当前反馈播种距离累计器(任意值,只用差分)
6. 发 0xFC 使能 ×8,收不到 ERR==1 的电机重试(每周期补发,最多 ~1s);仍失败的电机打 ERROR 并继续用其余电机跑(该模块剔除出 FK 均值),后台持续每秒重试使能
7. 锁存里程计原点(yaw_offset=−yaw_gyro),起循环(初始指令:转向保持当前角、驱动 v=0)

**停机**:停收 cmd_vel → 1-2 周期零速(转向保持)→ 0xFD 失能 ×8 → 关 socket。SIGINT / 析构 / `~/disable` 服务共用此路径;进程崩溃由电机 TIMEOUT 寄存器兜底(这是硬件层的事,上位机不另设软件急停状态机)。

## 安全策略(⚠️ 已被 2026-09-03 严格策略修订取代,本节仅存档 Phase 5 实现时的行为;目标行为见文档开头总原则与 Phase 7)

- **cmd_vel 超时**:目标 twist 置零(转向按参数保持),新指令到达即恢复。唯一的"停车"行为,且是自恢复的
- **IMU 超时**:自动降级为轮式正解 Δθ 做 yaw(明知精度差,打 WARN),IMU 恢复即无缝切回陀螺(切换时对齐 yaw_offset 防跳变)。**不停车**
- **单电机反馈丢失**:该周期从 FK 均值剔除、里程计照常;diagnostics 记 missed 计数。连续丢失超阈值(默认 50 周期 ≈ 0.5 s)→ 对该电机每秒补发一次使能帧尝试唤回,**其余 7 个电机照常工作**;恢复即自动归队。全部 8 个电机同时长时间无反馈(总线断)→ 停发驱动速度指令(置零),持续重试,总线恢复即继续
- **ERR 半字节非 1**:报 diagnostics(WARN/ERROR 分级);电机固件自己会保护自己。对可恢复错误按限频(1 Hz)自动发 0xFB 清错 + 重使能,不需要人工;`~/clear_faults` 服务保留作手动兜底
- **不做**:上位机温度阈值停车、限幅不符拒使能、p_m 校验拒使能、faulted 锁存态——这些要么重复电机固件已有的保护,要么把可运行状态变成不可运行
- diagnostics:每电机一条(ERR、温度、missed 计数)+ 底盘汇总,1 Hz

## ROS 接口

| 类型 | 名称 | 消息 |
|---|---|---|
| Sub | /cmd_vel | geometry_msgs/Twist |
| Sub | 参数指定 IMU 话题 | sensor_msgs/Imu(取 orientation yaw) |
| Pub | ~/odom + /tf(可关) | nav_msgs/Odometry |
| Pub | /joint_states | 8 关节(4 舵 + 4 轮) |
| Pub | /diagnostics | DiagnosticArray |
| Srv | ~/enable ~/disable ~/clear_faults ~/rezero_steering | std_srvs/Trigger |

## 测试与验证(TDD,gtest/ament_cmake_gtest,≥80% 覆盖)

**单元测试先行**:

- codec:float↔uint 往返(含边界钳位)、MIT 打包对照手算字节向量、反馈解码含 ERR、寄存器/特殊帧逐字节
- kinematics:纯 +x / 纯自旋(角度切向、速度=ω·hypot(L,W)/2)/ 对角;去饱和保比例;optimize(170° 目标 → −10°+反速);余弦补偿;门控;**discretize:exp(discretize(v)·dt) 与目标位姿增量误差 <1e-12,并量化对比欧拉在 90°/s+1m/s 用例下的漂移**
- odometry:恒曲率圆弧闭式解对比(位姿指数精确落圆,欧拉可见误差)、直线、原地旋转(平移为零)
- unwrap:±PMAX 双向跨越、p_m 播种、开机不符检测
- safety:cmd_vel 看门狗时序、丢帧计数与自动补使能调度、IMU 降级切换及恢复时 yaw 无跳变(注入时钟)

**无硬件集成测试**:`fake_motor_sim` 在 vcan0 模拟 8 电机(MIT 一阶动力学 + 使能/寄存器状态机 + 可配延迟/丢包),断言:每周期 8 帧背靠背(时间戳散布 <1 ms,验证原子下发)、直线/圆弧里程计收敛、注入单电机丢帧后其余 7 电机照常+恢复后自动归队。无 vcan 权限时 skip。

**标定文档**(doc/calibration.md 大纲):① 上位机分配 ID/审计限幅寄存器(须在刷 gs_usb 固件前用官方 GUI)② 治具对轮 → rezero(0xFE)→ 断电复验 ③ 推行 5 m 标 wheel_radius/gear_ratio,**并逐轮比对累计转角得到每轮轮径**(配置改每轮四值,消除旋转漂移的轮径分量;摩擦差异不标——速度环积分器自动消除,只查力矩饱和)④ kd=0 下 t_ff 斜坡找起动力矩 = ks(双向平均)⑤ 恒速平台拟合 kv ⑥ 可选阶跃拟合 ka ⑦ 转向 90° 阶跃调 kp/kd(<5% 超调)再正弦跟踪调 kff ⑧ 实车验证门控阈值与超时 ⑨ **原地旋转反解舵角零偏精标**:命令纯 wz 正反两向,由各轮实测速度分量最小二乘解 δθ_i 写回 zero_offset_rad;同时以"平均速度向量模"(非瞬时速度模)评估残余漂移。此科目先在仿真验证流程后真机复用(源自 2026-09 Gazebo 旋转中心审计的结论:漂移主因是接触动力学/数值抖动,系统分量优先归因于零偏与运动学参数,不平移 TF)

## 实施阶段顺序(Codex 执行,每阶段结束交 Claude review 后再进入下一阶段)

1. codec + socketcan(测试先行)+ fake_motor_sim 骨架,vcan 回环通
2. 纯数学核:kinematics(IK/discretize/optimize/desaturate)+ odometry(exp/log),单测全绿
3. DmMotor 解绕/健康 + SwerveModule 前馈映射与单位换算
4. 参数 + 控制线程 + 启停序列 + ROS I/O,vcan 集成测试通过
5. 降级监控 + diagnostics + 服务
6. 硬件 bring-up:单电机台架 → 单模块 → 架起整车 → 落地,走标定文档

## Claude 各阶段评审清单(review gates)

- **Phase 1**:MIT 打包/反馈解码逐字节对照本计划协议节(尤其 D3/D6 的拼半字节);float↔uint 钳位行为;write_batch 是否真的单处连续写(无分散 callback 写);ENOBUFS/部分写处理
- **Phase 2**:discretize 的 B=h·cot(h) 矩阵与符号(vx>0,ω>0 ⇒ vy_d<0);位姿指数小角度分支;FK 是否用位置增量而非速度;单测是否含欧拉对比用例
- **Phase 3**:齿比/方向符号的换算方向(易错:轮侧力矩→电机侧要**除** G_d);解绕跨界双向;±0.9·PMAX 回中翻转
- **Phase 4**:8 帧原子下发时序断言;控制线程与 executor 的数据交接无锁争用;启动序列的"读不回就退 YAML + WARN"路径确实不阻塞
- **Phase 5**:**重点审查是否引入了计划禁止的过度安全门**(锁存态、拒使能、上位机温度停车等);所有降级路径可自动恢复
- **通用**:文件 200-400 行、函数 <50 行、无魔法数字、覆盖率 ≥80%、错误显式处理不吞

## Phase 1–3 评审记录(2026-08-25,Claude review,HEAD 8fbaf43)

**结论:通过,进入 Phase 4**。全部 review gate 项核对无误:

- Phase 1:MIT D3/D6 拼半字节、反馈 D0=ERR<<4|ID、寄存器帧 CANID 小端与 0x33/0x55 操作码、float_to_uint 先钳位后映射、sendmmsg 单系统调用原子批发+部分写检测、ENOBUFS/EINTR/EAGAIN 分类处理、RAII fd
- Phase 2:discretize 的 B=h·cot(h) 矩阵与符号(vx>0,ω>0 ⇒ vy_d<0 有专测)、位姿指数小角度分支、FK 用位置增量+回绕安全中点角、缺反馈模块剔除出均值、欧拉对比与圆弧闭式解测试齐全
- Phase 3:齿比换算命令/反馈两侧互逆、轮侧力矩→电机侧除以 G_d(有专测)、解绕双向跨界、p_m 播种一致性只返回 bool 不拒使能、0.9·PMAX 回中翻转与 20° 门控接力覆盖整个 π 摆动(前半程 recentered 压零驱动、后半程门控接住),收敛
- 质量:无锁存态、无过度安全门;文件 ≤400 行、函数 <50 行、常量具名;覆盖率 81%(核心 95.7%);vcan skip 合理(内核未编 CONFIG_CAN_VCAN)

### Phase 4 开工时必须一并完成的修正与约定(Codex 注意)

1. **[较高] 转向 ω 前馈阶跃尖峰**(`src/swerve_module.cpp` `make_steering_command`):ω_ff 由目标角差分得到,±π 回中或方向突变时一个周期差出 π → 被钳到满 VMAX 的速度前馈 → kd·(VMAX−实际) 力矩踢腿。修正:① recentered 周期强制 ω_ff=0;② 新增参数 `steering.max_ff_speed_radps`(轴侧)钳位差分结果后再乘 G_s。约 10 行 + 1 个单测(断言回中周期 v_des==0、阶跃周期 v_des 不超新参数)
2. **[中] 帧路由约定**:`DmMotor::accept_feedback` 对 ID 不匹配帧抛异常是既定契约。控制循环 collect 之后必须先按 MST_ID 路由到对应电机,无主帧丢弃并打日志,不得让异常传进实时循环
3. **[中低] ENOBUFS 忙转**:CAN qdisc 满时 poll(POLLOUT) 可能立即返回可写,重试环在 deadline 内空转。有 write_timeout 兜底,暂不改;实机阶段若见 CPU 尖峰,设小 SO_SNDBUF 或加微睡眠
4. **[低] float_to_uint 截断不舍入**:与达妙参考实现一致(±0.5 LSB 偏差),保持现状,在 README 注明是有意为之
5. **[低] 首帧自动 seed 的静默风险**:G_s>1 的转向电机若启动时漏读 p_m,首帧反馈会自动 seed 出错误绝对角。Phase 4 启动序列必须先 seed 再进循环(本计划已规定);并在 `DmMotorHealth` 加 `seeded_from_multi_turn` 布尔标志,由 diagnostics 上报

## Phase 4–6 评审记录(2026-08-25,Claude review,HEAD ab93474)

**结论:软件部分通过,但存在 1 项高危 + 1 项数据竞争,须在硬件 bring-up 前修复**。

**Gate 逐项核对(全部 ✓)**:
- Phase 4:原子下发有断言(fake transport 层断言每周期恰好 1 次 write_batch 含全部 8 帧);cmd/IMU 邮箱整体加锁快照无撕裂读;stop 顺序正确(置停→join→零速→失能→关 socket),控制线程 join 完成后才 deactivate/reset publisher,无 use-after-free;`step()` 捕获全部异常再进 noexcept 的 `run()`,transport 抛错不会杀线程;启动序列降级路径不阻塞(socket 打不开是唯一硬失败,限幅读回失败退 YAML+WARN、以读回值为准,p_m 缺失/不符仅 WARN,使能失败降级续跑+后台每秒重试)
- 前置修正 4 条全落实:recentered 周期 ω_ff=0 + `steering.max_ff_speed_radps` 轴侧钳位、`seeded_from_multi_turn` 标志入 health 并上报 diagnostics、collect 后按 MST_ID 路由/无主帧丢弃打日志/异常逐帧隔离、启动 YAML 后备路径
- Phase 5:**无违禁安全门**——无锁存态、无上位机温度停车(温度仅作 diagnostics 值上报)、无拒使能;所有降级自恢复:cmd 超时置零自恢复、IMU 降级为轮式 Δθ(新增 3×3 正规方程最小二乘,≥2 有效模块才解,数学核对正确)且恢复时对齐偏置防位姿跳变、单电机静默限频补使能其余照跑、全总线静默仅门控驱动不失能
- Phase 6:标定/bring-up/硬件验证状态文档齐全,硬件待办如实标注未签字,无冒充通过

### 修复清单

**合并前必须修**:
1. **[高] `~/enable` 服务回调未包 try/catch**(`src/swerve_driver_node.cpp:155` 一带):`start_control()` 可抛(ControlLoop 构造校验、bad_alloc、`initialize()` 内未 guard 的 `initialize_odometry`);异常穿出 rclcpp 服务回调 → std::terminate 整进程。修法:回调体包与 `activate()` 相同的 try/catch,并把 `initialize()` 里的 `initialize_odometry(now)` 纳入 guard
2. **[中] `initialized_` 混锁数据竞争**(`src/control_loop.cpp:173`):写在 io_mutex 下、读在 status_mutex 下,C++ 内存模型 UB(TSan 在 WSL2 跑不了所以没暴露)。且该行冗余——`refresh_status()` 已在正确锁下拷贝过。修法:删除该行,或 `initialized_` 改 `std::atomic<bool>`

**建议修(可与硬件阶段并行)**:
3. [中] `odom.twist` 发布的是**指令** twist 而非实测(`src/ros_output.cpp:94-96`)——下游 EKF/nav2 期望实测底盘速度;用模块实测速度做正解或位置增量/dt 计算
4. [中低] diagnostics summary 的 degraded 判据用**累计** unknown/rejected_frames≠0(`src/diagnostics.cpp:99-100`)——历史上出现过一帧杂帧就永久 WARN;改为增量或滑窗计数
5. [低] `realtime_priority` int64→int 截断先于范围校验(`src/ros_params.cpp:155`)
6. [低] RT 控制线程内直接 RCLCPP 日志(分配+IO,SCHED_FIFO 下抖动/优先级反转);实机若见抖动改环形缓冲由非 RT 线程排空
7. [低] IMU 回调未检查 orientation 有效性(`orientation_covariance[0]==-1` 表示无姿态的 REP 约定);全零四元数会喂进 yaw=0
8. [低] joint_states 转向 velocity 恒 0;`make_default_transport` 写超时 2000µs 硬编码,可挂参数

### 硬件阶段备忘(随 Phase 6 现场执行)
- `kTimeoutCountsPerMillisecond=20`(TIMEOUT 寄存器 50µs/计数)是协议假设,首台电机上必须实测确认(write_timeout_register 默认 false,安全)
- 恢复动作帧(补使能/清错)在主批之外单独 write+collect,恢复期间每周期最多多占 ~4ms;100Hz 下预算尚可,若 loop_overruns 计数上涨再合并进主批

## 风险与开放问题

1. **驱动速度环靠 kd∈[0,5] 硬件封顶**:重载下 P 速度环偏软,ks/kv 前馈必须扛主力(标定重点);不够则备选:上位机外环速度 PI 调 t_ff,或驱动轮退回原生速度模式。kd/前馈参数做成热更新
2. **p_m 断电持久性未知**:若多圈计数不保持,G_s>1 的转向绝对角开机可能有 2π/G_s 模糊(表现为轮向偏差)→ Phase 6 第一步实机验证;失败则需追加 homing(硬限位/索引)工作项
3. **TIMEOUT 寄存器写(0x09)**:写操作码与"易失 vs 需 0xAA 保存"语义需实机确认;最稳妥是先用官方上位机配置一次,驱动内写入仅作可选校验
4. **已由 Phase 7.5 关闭**:删除电机层 PMAX 回中;机械安全区间映射不进 PMAX 时启动拒绝
5. **0x33 寄存器读的回复帧格式是协议中文档最薄弱处**:Phase 1 尽早对实机验证;codec 独立成文件,改格式只动一处。读不回也不阻塞——启动序列已定义了 YAML 后备路径

## Phase 7:验收整改——严格安全策略 + 缺陷修复(2026-09-03,依据同事验收报告与用户决议)

背景:同事验收报告(four_swerve_driver_code_acceptance_report.md)提出 8 条 P0、11 条 P1。经 Claude 逐条核实与用户决议:安全策略**全面转严格**(推翻原"宽松方针",见已修订的文档开头总原则);确认 2 条真缺陷与 8 条有效 P1。Codex 按下列顺序执行,完成后交 Claude review。

### P1 逐条核实结论(Claude 对照实际代码核实,用户已认可)

| 条目 | 核实 | 结论与修法 |
|---|---|---|
| P1-01 ±90° 高频翻转 | 属实,`optimize_module` 是无迟滞的硬边界 | 修:加翻转迟滞(仅当 \|err\| > π/2 + ~0.1 rad 才改变上周期翻转选择,每模块记忆翻转态) |
| P1-02 20° 门控走停 | 设计如此(强制要求⑤),报告也判功能 PASS | 不改代码;阈值本就是参数,MPPI 连续性问题留给导航层调 |
| P1-03 迟到帧当新帧 | 属实,collect 只按 ID 匹配;影响被位置增量法自愈,但会掩盖一个周期的静默计数 | 修(低优先):内核时间戳已有,路由时校验 frame 时间戳 ≥ 本周期发令时刻 |
| P1-04 无 covariance | 属实,ros_output 全零 | 修:基线协方差参数化,IMU 降级/模块缺失时膨胀 |
| P1-05 IMU 校验弱 | 属实(仅 isfinite+超时) | 修:四元数模长、`orientation_covariance[0]==-1`、yaw 跳变合理性 |
| P1-06 转向无执行级限速 | 属实,90° 是一次位置阶跃,`max_ff_speed_radps` 只管前馈 | 修:目标舵角斜坡限速(新参数 `steering.max_slew_radps`),顺带压小 P1-01 的抖动幅度 |
| P1-07 电机侧独立钳位破坏比例 | 属实,`make_drive_command` 各自 clamp VMAX | 修:去饱和上限改为 `min(max_wheel_speed, min_i(VMAX_i·r/G_d))`,启动读回限幅后计算 |
| P1-08 p_m 不可信继续跑 | 属实,原是宽松方针;随严格策略反转 | 改:p_m 缺失/不一致且 G_s>1 → 拒使能,提示 rezero(归入 7B-1) |
| P1-09 rezero 不核对回复 | 属实,collect 结果被丢弃 | 修:逐台按 MST_ID 确认 + 回读位置≈0 复验,任一失败报 failure |
| P1-10 降级 odom 无质量标记 | 属实;严格策略下单电机失联即停车,降级窗口大幅缩小 | 并入 P1-04 的协方差膨胀 + diagnostics 已有模块级状态 |
| P1-11 固定 dt | 属实,overrun 时加速度/ka 失真 | 修:dt 用实测周期,夹在 [0.5×, 2×] 标称之间 |

同时确认的两条 P0 真缺陷:P0-03 丢帧位移永久丢失(Claude Phase 4 评审误判放过,修法见 7A-1)、P0-02 odom.twist 发目标值(修法见 7A-2)。

### 7A. 缺陷修复(先做,独立于策略改造)

1. **[P0-03] 丢帧位移永久丢失**:`SwerveOdometry::update` 与 `update_cycle_odometry` 的差分基线由"上一周期样本"改为**每模块最后有效位置**;模块恢复反馈时对最后有效基线做差(位置增量法可无损跨缺口)。基线仅在当前样本 valid 时更新。回归测试:周期序列 有效→丢失×N→恢复,断言累计位移无损;含转向角跨缺口用中点角的容差断言
2. **[P0-02] odom.twist 实测化**:用模块实测轮速+实测舵角做正解得实测 (vx,vy),ω 取 IMU 角速度(降级时轮式反解);`ControlLoopOutput` 增加 measured_twist,ros_output 改用。测试:门控生效(车实际静止)时 twist≈0 而非指令值
3. **[P1-07] 去饱和统一上限**:启动读回限幅后计算 `v_cap = min(max_wheel_speed_mps, min_i(VMAX_i·r_w/G_d))`,desaturate 用 v_cap;电机侧 clamp 保留作最后防线
4. **[P1-01] 翻转迟滞**:`optimize_module` 增加每模块翻转记忆,仅当 |err| > π/2 + hysteresis(参数,默认 0.1 rad)才改变翻转选择
5. **[P1-06] 转向目标斜坡**:新参数 `steering.max_slew_radps`(轴侧),对连续目标角限速;对齐门控仍以最终目标 vs 实测角计算
6. **[P1-11] 实测 dt**:控制周期用实测 now−last,夹于 [0.5, 2]×标称;喂给加速度限制/ka/前馈差分
7. **[P1-05] IMU 有效性**:四元数模长偏差 >0.1 拒收、`orientation_covariance[0]==-1` 拒收、单周期 yaw 跳变超限(参数)拒收并 WARN
8. **[P1-04+P1-10] 协方差**:pose/twist 协方差基线参数化;IMU 降级、有效模块 <4 时按参数系数膨胀
9. **[P1-09] rezero 复验**:save_zero 后逐台按 MST_ID 确认回复,再回读反馈断言位置≈0(容差参数),任一台失败返回 failure 并列出电机
10. **[P1-03] 反馈时间戳校验**(低优先):路由时丢弃时间戳早于本周期发令时刻的帧(仍计入 accepted 统计口径需同步调整)

### 7B. 严格安全策略改造(对应 P0-04/05/06/08、P1-08)

1. **启动硬门**(`initialize`):限幅读取重试后仍失败 → 返回 false(activate 失败);任一电机无反馈/使能未确认 → 同样失败;G_s>1 且 p_m 不可读或与主值不一致 → 失败并在日志给出 rezero 指引。`limits_fallback` 仅保留给 `can.interface=vcan*`/fake 测试(显式参数 `allow_fallback_limits`,默认 false)
2. **单电机失联全车停**:`SafetyMonitor` 增加故障态:任一电机连续静默 ≥ `feedback_silent_cycles` 或报硬件类 ERR → 全部驱动置零(转向保持)+ 进入 faulted
3. **故障分类 + 锁存 + 次数上限**:欠压(0x9)/通信丢失(0xD)→ 允许自动 0xFB+重使能,限频 1 Hz、上限 `auto_recovery_limit`(默认 3),超限锁存;过流(0xA)/MOS 过温(0xB)/线圈过温(0xC)/编码器(0x2/0x5)/过载(0xE)→ 立即锁存。锁存态:持续发零速指令、diagnostics ERROR,仅 `~/clear_faults` 服务可解锁(解锁时清计数、复验使能)
4. **TIMEOUT 寄存器**:实机确认 0x09 写入语义(50µs/计数假设)后,`write_timeout_register` 默认改 true;验收增加"运动中物理拔 CAN 线,电机须在 timeout_register_ms 内自失能"实测项
5. 更新文档:README 安全行为说明、hardware_bringup 验收清单增加断线/单电机失联/故障锁存三个实测科目
6. **Phase 5 旧评审 gate 反转**:原"审查是否引入过度安全门"作废;新 gate = 严格策略逐条落实且测试覆盖(启动拒使能路径、单失联停车、分类锁存、次数上限、clear_faults 解锁流程)

### 不改代码的验收回复口径

- P0-01(占位参数):设计即占位,标定流程在 doc/calibration.md,属实车 bring-up 前置项
- P0-07(断线停车):软件侧无法证明,由 7B-4 的电机固件 TIMEOUT + 物理拔线实测科目闭环
- P1-02(20° 门控走停):功能符合设计(用户要求⑤),阈值可调,导航连续性由导航层调参处理

## Phase 7.5:±180° 机械限位适配(2026-09-11,Gazebo 左后轮跨 π 异常旋转触发的需求变更)

**需求变更**:舵轮转向轴存在 ±180° 机械硬限位,推翻原"连续旋转"假设。方向:保留硬限位、改转向规划,不改连续关节。根因两条:① wrapPi 最短路把 +179°→−179° 当 2° 运动,实际穿越限位;② 门控/余弦补偿吃 wrapPi 误差,跨 π 时误判 2° 放行驱动 → 异常旋转;另 swerve_module 的 PMAX 回中在下层擅自改分支,属分层违规。

1. **分支选择改限位内行程代价**(swerve_kinematics 纯函数):正轮速候选 wrapPi(φ) 与反轮速候选 wrapPi(φ+π),±π 双边界表示均入候选,留 `joint_limit_margin_rad`;代价 = 与 θ_c 的有界区间线性距离(无回绕);滞回改代价制(另一分支代价 + hysteresis < 当前才切换)。两端点在限位内 ⇒ 直线段轨迹全程合法,叠加既有斜坡限速
2. **误差定义换血(修症状的关键)**:error = 所选分支带符号线性行程(非 wrapPi 差);对齐门控、余弦补偿、ω 前馈差分全部改用。+179°→−179° 场景门控看到 178° → 驱动置零至对齐
3. **电机层降级为纯换算+校验**:删除 select_steering_target 的 PMAX 回中(及 recenter_trigger_fraction、相关测试);启动校验 [(limit_min+θ₀)·G_s, (limit_max+θ₀)·G_s] ⊆ ±PMAX(含裕量),不满足拒使能;运行时电机目标越出机械限位映射区间 = 上层 bug → 显式报错+保持,禁止静默钳位;单测断言下层永不改分支
4. **参数**:新增 `steering.joint_limit_min_rad/max_rad`(默认 ±π)、`joint_limit_margin_rad`;flip_hysteresis_rad 语义改代价差滞回;多圈解绕对限位关节退化为平凡,加启动断言
5. **回归测试**:+179°→−179° 选反分支行程 178° 且门控闭合;θ_c=+179° 目标 0° 用 +180° 边界候选(1° 行程);属性扫描任意 (θ_c,φ) 目标与轨迹均在 [min+margin, max−margin];电机层不改分支;Gazebo 左后轮跨 π 场景回归
6. **连带修订**:核心数学第⑤节、第⑦节与本节冲突处以本节为准;Phase 8(Kinco 平台)同样适用本节——CSP 目标位置生成沿用同一分支选择器,限位校验换算用 Kinco 单位

## Phase 7.5 评审记录(2026-09-11,Claude review,HEAD 1c8c529)

**结论:五项要求全部落实,通过;附 1 条必修鲁棒性缺口 + 2 条注记**。

核对:分支选择改 2πk 窗口取最近等效角(`nearest_equivalent_in_limits`,±π 双边界表示被 turn-window 自然覆盖,验证 θ_c=+179°/目标 0° 得 +180° 候选 1° 行程 ✓);代价制滞回 ✓;误差改线性行程且门控/余弦/前馈差分全部改用 ✓(RearLeftPiCrossing 测试断言 178° 误差全轮门控);PMAX 回中及 recenter/command_limit_fraction 参数删除,电机层降级为换算+校验、越界显式抛错 ✓;启动双硬门(机械区间 ⊆ PMAX 可表示性 + 播种角在限内)✓;safe_range ≥ π 校验保证任意朝向可达 ✓;属性网格扫描/边界候选/滞回/斜坡邻界测试齐全 ✓。

**必修(下一批带上)**:
1. **[中高] 量测角越界导致周期失败死循环**:`optimize_module` 对 measured current 落在 margin 区/略越物理限位(背隙、限位柔性、零偏误差在实机必然出现)直接抛 out_of_range → step 捕获 → 每周期"cycle failed"、不发帧 → 电机 TIMEOUT 失能、无恢复路径。修法:量测角在 [物理限位 ± tolerance] 内时**钳入 safe 区参与计算**(margin 只约束目标,不拒绝量测);超出物理限位+容差才算故障,且走 SafetyMonitor 正规故障路径(可锁存、可 clear),不是异常循环。网格测试补 current ∈ margin 区/略越界用例
2. [低] `validate_seeded_steering_positions` 用含 margin 的 safe 区拒启动:轮子恰停在 margin 区内会拒使能且 rezero 无解(轮子物理就在那);改用物理限位+容差,与上条同一原则
3. [低] 旧 2 参 `optimize_and_apply_alignment`(默认限位、无滞回)仍存在,确认仅测试引用;标记 deprecated 防生产误用

### Phase 7.5 评审必修闭环(2026-09-11,Codex,实现 HEAD 820f687,文档 HEAD 10b204d)

- 新增 `steering.joint_limit_tolerance_rad`(默认 0):仅用于物理端点附近量测容差,不扩大扣除 margin 后的目标安全区间。
- margin 区或物理限位 ± tolerance 内的量测钳入 safe 区参与规划,目标和整条 slew 轨迹仍严格留在 safe 区。
- 超出物理限位 ± tolerance 的量测不再抛异常导致周期停帧,改走 `SafetyMonitor` 正规路径:立即锁存、四驱归零、控制周期继续发帧；活动越界时拒绝 clear,量测恢复后仍需人工 clear。
- 启动播种角按物理限位 ± tolerance 校验；完整 safe 目标区间映射进实际 PMAX 的启动硬门保持不变。
- 旧 2 参 `optimize_and_apply_alignment` 已标记 deprecated,生产代码与测试均无调用。
- 已补参数/ROS、边界量测、启动、运行锁存、人工清错与 diagnostics 回归测试。
- 最终验证:普通/`-Werror`/ASan+UBSan/coverage CTest 均 21/21；全新隔离 colcon 193 tests、0 errors、0 failures、0 skipped；cppcheck 29/29；28 个核心源文件加权行覆盖率 85.98%；全部生产 `.cpp` 不超过 400 行(最大 397 行)。

## Phase 8(规划,待硬件最终确认):EtherCAT 后端 + CANopen 外置编码器

背景:下一代舵轮模组换用步科(Kinco)FD 系列低压伺服(EtherCAT,手册确认支持 CSP/CSV/CST 周期同步模式),转向轴选配 16 位 CANopen 外置编码器。达妙版冻结为算法验证平台,控制核心整体平移。

### 架构

- 上位机双总线:EtherCAT 主站(8 台 FD 驱动器)+ SocketCAN can0(4 个 CANopen 编码器,总线独占)
- **复用不动**:kinematics(discretize/IK/FK/optimize+迟滞/去饱和/门控/斜坡)、swerve_odometry、swerve_setpoint、safety_monitor 策略框架、control_loop 结构、ROS 接口、参数框架。要求⑥原子下发由 EtherCAT 单帧+DC 同步原生保证
- **替换**:dm_frame_codec/socketcan_interface/motor_startup → EtherCAT 层(IgH/ecrt)+ DS402 状态机(0x06→0x07→0x0F 使能、0x0000→0x0080→0x0000 有界脉冲清故障、状态字 bit3 故障位)+ PDO 映射(RxPDO 0x1601 可配)
- **模式**:转向 = CSP(8),目标位置 0x607A 每周期下发;驱动 = CSV(9),目标速度 0x60FF。単位换算:速度 DEC=(rpm·512·编码器分辨率)/1875,位置 inc

### 要求④前馈的归属(已与步科确认:前馈对象仅上位机/SDO 可写,不可 PDO 映射)

- 转向 ω/加速度前馈 → 驱动器内 0x60FB.02/.03(位置环速度/加速度前馈),**启动序列 PREOP 阶段 SDO 写入**或标定时用 KincoServo 整定后 0x2FF0 存盘;上位机 ω_ff 代码路径对 Kinco 后端禁用
- 驱动轮 ks/kv → 驱动器速度环 PI(0x60F9 Kp/Ki)积分器天然消稳态摩擦误差(达妙 kd 封顶风险随之消失)
- **兜底**:低速 stick-slip 实测不达标 → 驱动轮切 CST(10),0x6071 目标力矩走 PDO 每周期下发,上位机闭速度环 + ks/kv/ka 前馈原样回归(等价 MIT kp=0 用法)

### CANopen 编码器集成(CiA 406)

- 上报模式优先 SYNC 触发(每周期主站发 0x080,4 编码器同步采样回 TPDO),备选周期定时 5-10ms;1 Mbps;心跳挂 diagnostics
- 舵角源:编码器为底盘级绝对舵角主源(对齐门控/余弦补偿/里程计吃真值,闭掉齿圈背隙);电机侧角度作降级备份,掉线告警降级、恢复归队,遵循既有安全策略
- 达妙的 p_m/解绕/rezero 机制在此后端不再需要;rezero 服务语义改为"记录编码器安装偏移参数"

### 故障与安全映射

- ERR 半字节分类表 → 0x2601 十六位错误字重映射(过流/过温/编码器/跟随误差 0x6065 超限等),沿用 Phase 7"可恢复/立即锁存 + 次数上限"框架
- 达妙 TIMEOUT 兜底 → EtherCAT 从站 PDO watchdog(断链自动进安全态)+ 驱动器报错停止模式 0x605E + 主站 WKC/AL 状态监控
- 实机环境:主站需独占网卡 + 建议 PREEMPT_RT;WSL2 不能跑真 EtherCAT,开发期用 SOEM 仿真从站/录制回放做 CI

### 外置编码器已确认形态:多圈绝对 + 小齿轮啮合齿圈(非 1:1)

由此派生的设计要求:

- **换算**:轴角 = 编码器多圈总角度 / G_e − 安装偏移,G_e = 齿圈齿数/小齿轮齿数。参数:**精确整数齿数**(禁用测量近似,误差会随圈数累积)、方向符号、每模块安装偏移(治具标定一次,多圈保持期间永久有效)
- **多圈保持是新命门**:停机时持久化编码器读数,开机比对——停放期间大跳变(多圈复位/换电池)→ 按严格策略**拒使能**并提示重标零(复用 Phase 7 启动硬门框架)。电池保持型的话,电池更换进维护清单
- **背隙实测科目**(bring-up):同一轮向双向逼近测读数重复性,确认编码器自身啮合背隙量级可接受
- 收益:轴侧分辨率 = 单圈分辨率 × G_e;配合保留的 ±π 等价翻转回中策略,轴角有界,多圈计数不会溢出
- CiA 406 读数对象 0x6004(含多圈的 32 位位置)映射进 TPDO

### 打滑检测(差模,小增量,2026-09-11 决议纳入)

- **最小二乘残差检测**:复用 `wheel_chassis_delta` 的 3×3 正规方程,解算后计算**逐模块残差**(每模块 2 行观测的拟合残差)。残差超阈值(新参数 `slip_residual_threshold`)→ 该模块本周期从 FK 剔除(复用现有 valid 机制)+ diagnostics 上报**定位到轮**
- **协方差**:odometry 膨胀机制新增 `slip_covariance_scale` 系数,检测到打滑周期按此膨胀
- 辅助信号(后续可选):陀螺 ω vs 轮式反解 ω 比对、IMU 加速度 vs 轮式加速度比对、力矩-运动不匹配(0x6077 实际力矩在 TPDO)
- **分工原则(写进文档)**:差模打滑(单轮/不一致)归驱动层残差检测;**共模打滑**(急刹/坡道四轮同滑,残差盲区)归上层 EKF 跨传感器融合——驱动层职责是诚实协方差 + 预清洗的 odom,不越界做整车状态估计
- 测试:注入单轮虚假轮速断言检测/定位/剔除/膨胀齐动作;注入共模一致偏差断言残差不触发(明示盲区,防误用)

### 已确认决策(2026-09-07)

- **电机自带编码器同为多圈绝对**,但电机→轴传动链有传动误差 → **舵角主备关系定案**:外置编码器为主源(测轴侧真值,闭掉传动链背隙/弹性);电机多圈绝对为备份 + **开机双绝对源交叉校验**(各自换算轴角比对,差异 > 阈值(背隙+弹性余量,标定实测)→ 按严格策略拒使能;可同时捕获任一侧多圈复位、齿数配置错、安装松动)。运行时外置编码器掉线 → 以掉线时刻两源偏差即时校准后降级用电机侧角度,恢复归队
- **EtherCAT 主站定 IgH**:内核态 ec_master 按目标机内核编译,配 PREEMPT_RT;驱动链接用户态 ecrt 库。开发/CI 影响:实机绑定,WSL2/容器只能做应用层 mock,协议层验证依赖台架

### 开放问题(开工前必须确认)

1. 编码器"16 位"的构成:**单圈分辨率几位 + 多圈计数几位**;多圈保持机制(电池 / 韦根自发电);精确齿数比
2. FD 驱动器支持的最小 DC 周期与 PDO watchdog 超时配置范围

### Phase 8 开工前资料核验(2026-09-11,Codex)

- 本地 Kinco FD ESI 与《Kinco 低压伺服驱动器使用手册》确认 CSP/CSV/CST=8/9/10、0x60FB.02/.03 为位置环速度/加速度前馈、0x2601 为 16 位实时错误字。
- 实现采用 `0x0000→0x0080→0x0000` 的 bit7 复位脉冲。更正此前解释：0x0086 也包含 bit7 和 shutdown 低位，不能仅因 Modbus 异常响应同值便断言它不是控制字；本实现显式分开复位与使能步骤。
- BRT 编码器 EDS/手册确认 0x6004 为 32 位无符号位置，0x6501/0x6502 可读实际单圈分辨率/硬件总圈数，TPDO2 默认 transmission type=1 可由 SYNC 触发；该系列多圈绝对值不依赖电池或停电记忆。
- 采购型号的 0x6501/0x6502 实际值与精确齿数比仍需现场读回/机械资料确认；软件以期望值对读回值做严格启动校验，不猜位宽拆分。

### Phase 8 软件推进记录(2026-09-12,Codex)

- 本轮提交：`8ebbf99`；增量 review：`git diff df11631..8ebbf99 -- dm_swerve_driver`。尚未推送远程。
- 按用户要求取消每个小改动独立 RED/GREEN 提交，只补一条跨组件运行冒烟并统一回归。
- 已实现真实 IgH 1.6 可选适配：八轴 PDO 重映射、DC、WKC/AL/链路及 PDO watchdog；已从固定官方源码构建用户库并完成真实节点动态链接，非接口桩验证。
- ROS `driver.backend=kinco` 接入实际 Kinco 控制线程，保持 cmd/IMU 邮箱、odom/TF/joint states、enable/disable/clear/rezero 服务。
- 启动先禁能取样，经外置编码器规格/持久值/机械角/双绝对源检查后才使能。运行沿用有限角规划、对齐门控、主备切换和故障锁存/恢复上限；恢复控制字嵌入单次周期批。
- BRT 初始化校验 TPDO2 映射和缩放，配置 SYNC/心跳，确认 SDO 后进入 Operational；旧 TPDO、杂帧、重复及缺帧被隔离。
- 标零改为保存安装偏移/电机参考与快照；新配置模板 `config/kinco_params.yaml` 的未确认字段默认 0/空，拒绝配置。
- 0x60FB 速度前馈参数明确使用 `position_velocity_feedforward_raw`，ESI 默认 raw=256，不将工程百分数直接下发；写入默认关闭。
- 普通/Werror/ASan+UBSan/coverage 回归各 29/29，已有测试结果汇总 220 tests、0 errors/failures；cppcheck 47 个生产源文件通过，核心源行覆盖率约 83.3%。
- 现场工作见 `doc/kinco_bringup.md`：FD/BRT PDO、DC/watchdog 范围、背隙、真实标零、π 边界和拔线停止尚未验收。当前运行路径为 CSP/CSV；CST 闭速度环留待 CSV 低速实测不足后追加。

### 当前方案确认(2026-09-12,用户决议)

- 取消达妙 CAN/MIT 电机方案，四舵轮电机方案只保留 Kinco FD EtherCAT：4 转向 CSP + 4 驱动 CSV。
- 保留 4 个 BRT 外置绝对编码器及其 CANopen 通信，继续以轴侧编码器为舵角主源、电机编码器为备份。
- 保留精确齿数换算、双源标零与交叉校验、持久快照、SYNC/心跳、掉线备份和恢复归队。
- 标定资料按上述总线分工维护；历史达妙资料不再列入当前桌面交付。

### GitHub 推送完成(2026-09-12)

- 远程仓库：`https://github.com/SevenovaHangzhou/robot_driver`。
- 分支：`feature/damiao舵轮驱动`；远端提交：`b5ecdc4ed7d4eca6576836c8b8e9c7dff0a1c4bd`。
- 包内容同步自本地 `10ab59b`，包含最小二乘残差/异常轮剔除/协方差、有限角规划、
  Kinco EtherCAT + BRT CANopen 编码器、标定说明与记录表。
- 干净 Werror 构建及包测试通过：CTest 29/29，colcon 249 tests、0 errors/failures/skipped；
  GitHub 仓库质量门禁通过：197 策略测试、覆盖率 83%。本机缺 ShellCheck，待 CI 验证。
- 已通过 `git ls-remote` 核对远端 SHA。本次为功能分支源码上传，未做实机操作或生产部署。

### Phase 8 旧电机后端清理完成(2026-09-13,Codex)

- 本地实现提交：`f495c87a2956f94c6d080396439388a1492127f3`。
- 删除达妙 MIT 编解码、寄存器读写、使能/清错/标零、启动序列、反馈路由、
  电机/模块封装、旧控制循环、虚拟电机、bring-up 工具及其专属测试和参数。
- 节点不再声明或选择 `driver.backend`，启动路径固定为 Kinco EtherCAT；
  默认 launch 与配置模板均指向 `config/kinco_params.yaml`。
- 独立保留外置 BRT 编码器所需的通用 `CanFrame`、`CanTransport`、SocketCAN
  和 CANopen。SocketCAN 帧长校验改为标准 0～8 字节，允许 CANopen SYNC/短帧。
- 达妙安全状态类型已替换为 Kinco DS402 轴状态与故障分类，ROS diagnostics
  只报告 8 个 EtherCAT 轴、4 个外置编码器及底盘汇总。
- 默认 fail-closed 构建完成，CTest 18/18 通过；使用本机 IgH 1.6 头文件和静态库的
  `DM_SWERVE_ENABLE_IGH=ON` Release 构建通过。未执行实机 EtherCAT/CANopen 验收。
