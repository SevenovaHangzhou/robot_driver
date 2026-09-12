# 四舵轮标定说明

适用版本：软件提交 `8ebbf99`；编写日期：2026-09-12。
已确认方案：8 台 Kinco FD 电机只走 EtherCAT，转向 CSP / 驱动 CSV；
4 个 BRT 外置多圈绝对编码器保留 CANopen，作为舵角主测量，电机编码器作为备份。
四个转向关节均有机械限位，通常为 −180°～+180°。

| 设备 | 总线 | 职责 |
|---|---|---|
| 4 转向电机 + 4 驱动电机 | EtherCAT / IgH | CSP 位置控制、CSV 速度控制及电机反馈 |
| 4 外置绝对编码器 | CANopen / 独立 SocketCAN | 轴侧绝对舵角、SYNC 采样与心跳 |

取消的是达妙 CAN/MIT 电机方案，外置编码器及其标定、双源检查和主备切换流程保留。

现场操作见 [四舵轮标定手册](swerve_calibration_manual.md)，
测量结果填入 [标定记录模板](swerve_calibration_record_template.md)。
本文中的默认值和算例不构成实车标定结果。

## 1. 标定究竟要解决什么

让软件中的“轮子在哪里、朝哪里、转多快”与真实机构一致，再确定可用的动态参数和诊断阈值。
四类工作需要分开：

| 类别 | 示例 | 取得方式 |
|---|---|---|
| 硬件事实核实 | 齿数、减速比、电机分辨率、编码器圈数、设备地址 | 图纸、铭牌、厂家参数、设备读回，再用独立角度/圈数复核 |
| 几何与零点标定 | 轴距、轮距、零偏、负载下有效轮径 | 治具、尺寸测量、已知距离/角度实验 |
| 动态整定 | 伺服环、转向斜率、轮速加速度、主备恢复步长 | 低速阶跃、正反向和不同负载下记录响应 |
| 阈值与验收 | 两源差异、机械裕量、打滑残差、协方差、停车时间 | 健康数据分布、受控异常和真实停车测量 |

应先固定硬件比例关系再标零，先修正几何和方向再整定闭环。一次只改一组参数。
单次直行只能约束“轮径/齿比”的比例，不能同时辨识二者；只看底盘轨迹也无法可靠
区分四个转向零偏。需要独立的角度、圈数和距离基准。

## 2. 坐标、顺序与方向

从上方看，车体 +x 朝前、+y 朝左、+z 朝上；绕 +z 逆时针为正舵角和正 yaw。
舵角 0 表示轮子正向滚动方向朝车体 +x。轮面平行本身存在 180°歧义，必须同时核对正轮速方向。

| 数组下标 | 模块 | 车体位置 | EtherCAT 逻辑轴下标 |
|---|---|---|---|
| 0 | FL / front_left / 左前 | (+L/2, +W/2) | 转向 0，驱动 4 |
| 1 | FR / front_right / 右前 | (+L/2, −W/2) | 转向 1，驱动 5 |
| 2 | RL / rear_left / 左后 | (−L/2, +W/2) | 转向 2，驱动 6 |
| 3 | RR / rear_right / 右后 | (−L/2, −W/2) | 转向 3，驱动 7 |

L、W 测的是转向轴中心间距，不是车壳尺寸。`slave_positions` 可以按实际布线重排，
但其数组语义必须始终为“四转向 FL/FR/RL/RR，再四驱动 FL/FR/RL/RR”。

## 3. 参数清单及影响

下表全部是当前 ROS/YAML 的实际参数名。长度用 m、关节角用 rad；不要填 mm 或度。

### 3.1 几何、比例和地址

| 参数 | 当前形态 | 核实或标定内容 | 错误时的典型表现 |
|---|---|---|---|
| `chassis.wheelbase_m`、`chassis.track_m` | 各一个值 | 前后、左右转向轴中心间距 | 转弯轮速和轮式 yaw 不对 |
| `chassis.wheel_radius_m` | 四轮共用一个值 | 指定载荷、胎压/轮胎状态下的有效滚动半径 | 直行里程与速度整体成比例偏差 |
| `steering.gear_ratio` | 四转向共用一个值 | 电机转数 / 舵轴转数 | 角度误差随离零点距离增加 |
| `drive.gear_ratio` | 四驱动共用一个值 | 电机转数 / 车轮转数 | 与轮径误差容易混淆 |
| `steering.invert`、`drive.invert` | 各四个布尔值 | 电机正方向到关节正方向的符号 | 转向或行驶方向反向 |
| `kinco.steering_encoder_resolution`、`kinco.drive_encoder_resolution` | 各四个整数 | 电机每转对应的实际位置计数，核对电子齿轮/设备缩放 | 所有位置/速度换算错倍数 |
| `kinco.ethercat.slave_positions` | 八个整数 | 物理从站到逻辑轴映射 | 控制一轮时另一轮响应 |
| `kinco.ethercat.vendor_id`、`kinco.ethercat.product_code` | 各一个整数 | 实际 FD 身份 | 启动身份校验失败 |
| `kinco.encoder.node_ids` | 四个整数 | 1～127，互不重复，顺序 FL/FR/RL/RR | 编码器角被配给错误模块 |

当前不支持逐轮半径、逐轴电机减速比和任意独立模块坐标。四轮测出的半径明显不同
时，应先查轮胎、载荷和机构；不能向 YAML 填一个不存在的四轮半径数组。

### 3.2 外置编码器与零点

以下短名均加前缀 `kinco.encoder.`。

| 短名 | 单位/形态 | 作用 |
|---|---|---|
| `expected_counts_per_revolution` | 四个正整数 | 对应 0x6501 读回，每转计数 |
| `expected_distinguishable_revolutions` | 四个正整数 | 对应 0x6502 读回，可区分圈数 |
| `ring_gear_teeth`、`pinion_teeth` | 各四个精确正整数 | 齿圈/小齿轮齿数，不能用估算浮点齿比代替 |
| `direction` | 四个 +1 或 −1 | 外置编码器增加方向对应的舵轴方向 |
| `installation_offset_rad` | 四个轴侧 rad | 外置编码器安装偏移 |
| `source_disagreement_threshold_rad` | 轴侧 rad，>0 | 两绝对源线性角差容许值，启动/运行均使用 |
| `maximum_offline_axis_motion_rad` | 轴侧 rad，≥0 | 当前外置读数与上次可信停机读数之间的容许变化 |
| `maximum_rejoin_correction_rad` | rad/周期，>0 | 恢复主源后每周期消除对齐偏差的最大步长 |
| `snapshot_path` | 路径 | 可信停机读数；同路径加 `.calibration` 保存标零结果 |

`steering.zero_offset_rad` 是另一组四个轴侧零偏，作用在电机角换算上。
标零服务会同时确定电机参考和外置安装偏移。

### 3.3 限位、运动和诊断

| 参数 | 含义 | 标定要点 |
|---|---|---|
| `steering.joint_limit_min_rad/max_rad` | 舵轴物理上下限，四模块共用 | 使用每轮测量后共同可用的范围，绝不扩大机械行程 |
| `steering.joint_limit_margin_rad` | 从物理端点向内缩的目标裕量 | 容纳跟踪过冲、背隙和测量不确定度 |
| `steering.joint_limit_tolerance_rad` | 物理端点之外的测量容差 | 只接受测量噪声，不扩大命令目标范围 |
| `steering.max_slew_radps` | 舵角目标最大变化率 | 以实际跟踪能力与过冲确定 |
| `steering.flip_hysteresis_rad` | 正/反轮速分支切换的路程代价差滞回 | 抑制分支抖动，不能拿来绕过机械限位 |
| `chassis.align_threshold_rad` | 任一轮未对齐时的全车驱动门控阈值 | 默认约 20°；衡量侧向拖动和通过性后整定 |
| `chassis.max_wheel_acceleration_mps2` | 每轮目标速度变化率 | 受轮胎附着、驱动能力和载荷约束 |
| `chassis.max_linear_speed_mps/max_angular_speed_radps/max_wheel_speed_mps` | 运动上限 | 确认电机/轮胎/机构能力后逐步提升 |
| `chassis.velocity_deadband_mps` | 近零速保持轮向的阈值 | 高于无意义指令噪声，低于需要执行的最小速度 |
| `odometry.slip_residual_threshold` | 每轮速度向量残差阈值，m/s | 默认 0.25；从健康工况数据和受控异常确定 |
| `odometry.slip_covariance_scale` | 打滑时额外协方差倍数 | 默认 4，和其他降级系数相乘 |

## 4. 与实现一致的换算公式

周期和超时也要现场确认：`control.rate_hz` 与 `kinco.ethercat.dc_cycle_ns` 必须一致；
`kinco.ethercat.pdo_watchdog_ms` 依据驱动器支持范围和实测停止需求配置；
`kinco.encoder.feedback_deadline_us` 依据实际 CAN 收集耗时设置。
`control.cmd_vel_timeout_s`、`odometry.imu_timeout_s` 和
`odometry.max_imu_yaw_step_rad` 要结合上游发布频率、正常抖动与最大合法运动核对。
这些参数不能仅靠静态标零确定。

记 Rm 为电机每转计数，Ns/Nd 为转向/驱动电机的 0x6064 位置计数；
Gs/Gd 为两类电机减速比，sm/sd 为 invert 转成的 ±1。

```text
q_motor = 2π × N_motor / Rm
θ_motor_joint = q_steer_motor / (sm × Gs) − steering.zero_offset_rad
φ_wheel = q_drive_motor × sd / Gd
wheel_distance = φ_wheel × r
```

外置编码器 N 为 0x6004 的完整多圈计数，Re 为 0x6501，Zr/Zp 为齿圈/小齿轮齿数，
se 为 `direction`，Ge = Zr/Zp：

```text
θ_external = se × (2π × N / Re) × (Zp/Zr) − installation_offset_rad
Δθ_per_count = 2π / (Re × Ge)
```

示例：Re=65536，Zr=120，Zp=20，则 Ge=6；编码器转一圈，舵轴转 60°。
舵轴分辨率约 0.000916°/count。这是计数量化分辨率，不包含啮合背隙，也不等于绝对精度。

有限舵角的零偏计算、源间比较和停放变化采用实际线性差值。不能将多圈读数取模，
也不能用 wrapPi 把 +179° 和 −179° 当作相邻 2°来验证机械路径。

Kinco 速度内部单位：

```text
raw_velocity = rpm × 512 × Rm / 1875
wheel_speed_mps = raw_velocity × 1875/(512×Rm) × 2π/60 × sd/Gd × r
```

当前驱动使用这些换算。若设备配置了其他电子齿轮/用户单位，需先核实设备读数与换算
一致，不能再用轮径“补偿”整倍数的单位错误。

## 5. EtherCAT 方案的整定位置

| 项目 | 当前整定位置 |
|---|---|
| 有限角分支、斜率、门控、几何和打滑 | 上位机公共算法参数 |
| 转向/驱动伺服环 | KincoServo 或已核准的厂家对象 |
| 位置环前馈 | 可选 PREOP SDO 写 0x60FB.02/.03 |
| 标零 | 记录电机参考和外置编码器安装偏移 |
| 通信底层停车 | PDO watchdog、驱动器停止模式 |

Kinco 速度环 PI 与位置环参数在厂家工具中标定并保存。当前没有将 0x60F9 全套参数
做成 ROS 配置项。0x60FB 的速度前馈字段为 `kinco.position_velocity_feedforward_raw`；
raw=256 是所提供 ESI 的默认值，不能将“100%”直接写成 100。默认
`kinco.write_position_feedforward=false`，核准具体固件单位后才选择软件写入。
当前 CST 尚未接成运行闭环，因此不安排 Kinco 上位机 ks/kv/ka 标定。

## 6. 参数、标定文件与生效时机

| 载体 | 保存什么 | 当前生效规则 |
|---|---|---|
| 现场 YAML | 硬件身份、尺寸、齿比、方向、限位、阈值、运动和协方差参数 | configure 时读取；推荐改完停机重启并重新 configure |
| `snapshot_path` | 四编码器完整读数、分辨率/圈数信息及校验 | 启动比较；标零或可用的停机采样更新 |
| `snapshot_path + ".calibration"` | 外置安装偏移、电机零偏及部分配置指纹 | configure 时加载，覆盖 YAML 中对应两组零偏 |
| 驱动器非易失参数 | 厂家伺服环、电子齿轮和停止设置等 | 由厂家工具/协议确定保存及掉电生效规则 |
| 原始记录 | ROS bag、PDO/CAN 记录、外部测量、计算过程 | 用于复算与验收，必须与配置版本关联 |

当前没有“在线改参数立即重建控制环”的保证。`ros2 param set` 的成功回复不代表
运行控制器已采用新值；标零服务更新的内部偏移也不会自动回写成 ROS 参数服务器的新值。
应以标定文件、实际读数和重启后的表现为准。

改变电机减速比、分辨率、invert、编码器安装或齿数后重新标零。现有标定文件仅检查
部分外置编码器配置，不能自动识别所有电机侧配置变更。

## 7. 验收记录的边界

软件单测/fake 证明计算和控制分支，不能证明真实齿数、零点、背隙、DC 周期或拔线停止。
阈值需要健康数据与异常数据共同验证；最小二乘残差只能识别轮子之间不一致的差模异常。
四轮一致的共模打滑需要上层跨传感器融合。

本方案不具备独立命令单个转向角的 ROS 服务，也未直接发布每轮原始残差和两绝对源差值。
需要这些量时，使用标定记录中的 CAN/PDO/伺服 trace 和离线计算，或先补充采集能力，
不能把缺失证据填成“通过”。
