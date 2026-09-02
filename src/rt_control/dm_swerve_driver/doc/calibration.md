# dm_swerve_driver 标定流程

本文是 `config/swerve_params.yaml` 的实车标定规程。仓库中的机械尺寸、限幅和前馈系数只是 vcan 占位值，未完成本流程前不得把它们当作实车参数。

## 0. 记录与通用约束

每次标定记录日期、电机序列号、ESC_ID/MST_ID、固件版本、电源电压、轮胎状态、载荷和参数 Git commit。修改参数后保存原始数据与拟合脚本输出，不只记录最后一个数字。

软件没有温度停车、故障锁存或人工复位门。电机固件负责自身保护；测试人员仍必须准备可直接断开主电源的物理开关，并确保台架能承受最大输出。

标定前执行：

```bash
source /opt/ros/humble/setup.bash
ros2 run dm_swerve_driver dm_swerve_bringup_check \
  --interface can0 \
  --steering 1:17 --steering 2:18 --steering 3:19 --steering 4:20 \
  --motor 5:21 --motor 6:22 --motor 7:23 --motor 8:24 \
  --pmax 12.5 --vmax 30 --tmax 10
```

退出码 `0` 表示全部读回且无差异，`1` 表示审计有缺失或告警，`2` 表示参数/接口致命错误。该工具不使能电机。

## 1. 分配 CAN ID 与审计映射限幅

在刷写或连接 gs_usb 固件前，用达妙官方 GUI 逐台连接：

1. 设置唯一 ESC_ID 和 MST_ID，记录 FL、FR、RL、RR 的转向/驱动对应关系。
2. 确认全部电机为 MIT 模式、CAN 2.0 标准帧、1 Mbps。
3. 记录 PMAX、VMAX、TMAX、固件版本和齿轮减速比寄存器。
4. 配置电机 TIMEOUT；驱动内写入 `0x09` 仍保持为可选项。
5. 接上整条总线后运行只读审计，确保 8 个 MST_ID 都有反馈。

验收：ID 无重复；驱动读回的映射限幅与 GUI 一致。读回与 YAML 不同时驱动会使用读回值并 WARN，不会拒绝使能。

## 2. 转向机械零点

1. 架起车辆并失能驱动：`ros2 service call /swerve_driver/disable std_srvs/srv/Trigger {}`。
2. 用治具把四个轮面严格平行于车体 +x 方向。
3. 调用 `ros2 service call /swerve_driver/rezero_steering std_srvs/srv/Trigger {}`。
4. 断电、等待母线完全放电、重新上电，再次只读审计 p_m 与主反馈。
5. 启动节点，确认 diagnostics 中四个转向电机 `seeded_from_multi_turn=true`。

验收：断电重启后轮向误差满足机械要求；`wrapPi(p_m)-wrapPi(raw)` 小于 0.1 rad。若 G_s>1 且 p_m 不持久，停止实车落地测试并追加 homing/索引方案。

## 3. 轮径与驱动齿比

1. 在低速、直线、无打滑地面标记起点，手推车辆 5 m 以上。
2. 记录四轮累计电机转角和实际移动距离，正反方向各三次。
3. 对每轮计算 `r_eff = distance * G_d / motor_angle`。
4. 检查左右/前后差异；若差异来自轮胎或机构，不要用方向系数掩盖。
5. 将统一值或经过机械确认的值写入 `chassis.wheel_radius_m` / `drive.gear_ratio`。

验收：再次推行 5 m，四轮里程与实测距离误差满足项目指标，直线横漂无系统性方向。

## 4. 静摩擦前馈 ks

1. 架起单模块，先把 `drive.kd=0`、`kv=0`、`ka=0`。
2. 从零缓慢增加轮侧 t_ff，记录轮开始连续转动的正向和反向力矩。
3. 每个方向至少重复五次，剔除明显卡滞异常点。
4. `ks = (abs(T_start_positive)+abs(T_start_negative))/2`。

验收：给定略高于 ks 的低速命令能稳定起转，略低于 ks 时不会持续加速。重新启用 `drive.kd`。

## 5. 速度前馈 kv

1. 保持轮离地或使用可控滚筒台，选择多个稳态轮速平台。
2. 每个平台等待加速度接近零，记录稳态轮速和维持力矩。
3. 分别拟合正/负方向 `T-ks*sign(v)=kv*v`，检查残差和方向不对称。
4. 使用适合两方向的系数写入 `drive.kv`。

验收：速度平台主要由前馈维持，硬件 kd 只修正残差，不长期饱和。

## 6. 加速度前馈 ka（可选）

在负载和供电稳定的台架上执行多组限斜率阶跃，记录轮速、目标加速度和力矩。固定已标定的 ks/kv 后拟合剩余项 `T-ks*sign(v)-kv*v=ka*a`。若数据噪声或轮胎弹性使拟合不稳定，保留 `ka=0` 并记录原因。

验收：加入 ka 后阶跃跟踪改善且不过冲恶化，不以放大噪声换取单次漂亮曲线。

## 7. 转向 kp/kd/kff 与前馈上限

1. 架起单模块，先令 `kff_omega=0`，执行 90° 阶跃。
2. 增加 kp 到响应足够快，再增加 kd 抑制过冲，目标过冲小于 5%。
3. 使用低幅正弦角度目标，逐步增加 `kff_omega` 以减小相位滞后。
4. 设置 `steering.max_ff_speed_radps` 为机构可接受的轴侧目标角速度；它在乘 G_s 前钳位。
5. 人工制造接近 0.9·PMAX 的回中条件，确认回中周期 steering v_des=0、驱动速度=0，随后由 20° 对齐门控接力。

验收：无满 VMAX 速度前馈尖峰、无明显踢腿，±π 等效翻转后可自动恢复行驶。

## 8. 整车功能验证

按 `hardware_bringup.md` 的四个阶段执行。重点验证：

- 8 条 MIT 指令同周期批发，反馈无异常 MST_ID。
- `/cmd_vel` 超时自动零速，新命令立即恢复。
- 拔掉 IMU 后不停车，yaw 切到轮式估计；恢复 IMU 时 odom 无跳变。
- 单电机反馈丢失时其余 7 电机继续，恢复后自动归队。
- 总线整体静默时驱动速度被置零，恢复后自动解除。
- 电机 ERR 触发限频 clear+enable，但没有上位机故障锁存。
- `disable`、SIGINT 和进程退出均发送零速后批量失能。

完成后把实测结果填入 `hardware_validation_status.md`，并由测试人员签字。
