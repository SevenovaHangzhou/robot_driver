# ROS2 Humble 四舵轮底盘驱动 — dm_swerve_driver 指导计划

**分工**:本文档是给 **Codex(实现者)** 的执行指导;**Claude 负责每阶段 review**(评审清单见文末)。

**安全设计总原则(最高优先级约束)**:不设过于严谨的安全门。具体准则:

- 能**降级运行+告警**就不锁存故障;能**自动恢复**就不等人工干预;能**警告**就不拒绝使能/拒绝动作
- 电机固件自身已有过流/过温/失联(TIMEOUT 寄存器)保护,上位机**不重复设卡**,只做报告和兜底
- 整个驱动里唯一的"硬停"路径:用户显式调用 `~/disable` / 进程退出 / SIGINT
- 转向对齐门控(要求⑤)不是安全门,是控制品质手段,保留

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

### ⑤ 角度优化 + 连续多圈指令

```
err = wrapPi(target − wrapPi(current));  若 |err|>π/2: err −= sign(err)·π, 轮速反向
continuous_target = current_unwrapped + err               # 无 ±π 跳变
motor_target = (continuous_target + θ₀_i)·G_s·sign_s      # 钳位 ±0.95·PMAX
```

漂移防护:`|motor_target| > 0.9·PMAX` 时允许一次 ±π 等效翻转回中(需专门单测)。

### ⑥ MIT 字段映射——要求④

- **转向**:`p_des=motor_target, v_des=clamp(Kff·ω_target_axis·G_s, ±VMAX), kp=steer_kp, kd=steer_kd, t_ff=0`;ω_target 由相邻两周期 continuous_target 差分
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

## 安全策略(全部为"降级+告警+自恢复",无故障锁存状态机)

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

**标定文档**(doc/calibration.md 大纲):① 上位机分配 ID/审计限幅寄存器(须在刷 gs_usb 固件前用官方 GUI)② 治具对轮 → rezero(0xFE)→ 断电复验 ③ 推行 5 m 标 wheel_radius/gear_ratio ④ kd=0 下 t_ff 斜坡找起动力矩 = ks(双向平均)⑤ 恒速平台拟合 kv ⑥ 可选阶跃拟合 ka ⑦ 转向 90° 阶跃调 kp/kd(<5% 超调)再正弦跟踪调 kff ⑧ 实车验证门控阈值与超时

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
4. **G_s 大时转向电机位置向 ±PMAX 随机游走**:0.9·PMAX 处 ±π 等效翻转回中,需专项单测
5. **0x33 寄存器读的回复帧格式是协议中文档最薄弱处**:Phase 1 尽早对实机验证;codec 独立成文件,改格式只动一处。读不回也不阻塞——启动序列已定义了 YAML 后备路径
