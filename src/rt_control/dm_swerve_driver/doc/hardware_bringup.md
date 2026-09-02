# 硬件 Bring-up 顺序

必须按单电机、单模块、架起整车、落地四级推进。上一级未满足验收项时不得进入下一级。

## 共用准备

- 主电源物理断路器可触达；CAN 与逻辑电源分开检查。
- 车辆可靠架设，轮子高速旋转范围无人和线束。
- 终端电阻、总线波特率和电源极性已测量。
- `config/swerve_params.yaml` 已复制一份带日期的现场版本。
- 开两个终端观察 `/diagnostics` 和 `/swerve_driver/odom`。

启动命令：

```bash
source install/setup.bash
ros2 launch dm_swerve_driver swerve_driver.launch.py \
  params_file:=/absolute/path/to/site_swerve_params.yaml
```

停止命令：

```bash
ros2 service call /swerve_driver/disable std_srvs/srv/Trigger {}
```

## A. 单电机台架

1. 总线上只连接一台电机，运行 `dm_swerve_bringup_check --motor ESC:MST`；转向电机用 `--steering`。
2. 核对 PMAX/VMAX/TMAX、ERR、温度和 p_m。
3. 使用现场 YAML 启动节点，但只给极低速度/角度目标。
4. 验证 enable、正常反馈、cmd 超时零速、disable 和硬件 TIMEOUT。
5. 断电复验 p_m 持久性；记录 `0x09` 写入是否易失。

验收：方向、单位、反馈 ID 正确；无意外使能；通信中断后固件按 TIMEOUT 失能。

## B. 单模块台架

1. 连接一台转向和一台驱动电机，确认两条 MST_ID 独立。
2. 轮子悬空，从 +x、+y、170° 目标验证 optimize 和反速。
3. 验证 20° 对齐门控：转向误差大时驱动为零，收敛后自动恢复。
4. 触发接近 PMAX 的等效回中，检查无速度前馈尖峰。
5. 分别拔掉转向/驱动反馈，确认 diagnostics 和自动重使能。

验收：转向/驱动符号与 YAML 一致，回中和门控平滑，恢复不需要重启节点。

## C. 架起整车

1. 接入 8 电机和 IMU，先运行只读审计。
2. 启动节点，确认四转向 `seeded_from_multi_turn=true`，全部电机 ERR=1。
3. 依次发送纯 +x、纯 +y、纯自旋和对角命令，目视核对四轮方向。
4. 记录 CAN 抓包，确认每控制周期 8 帧集中发送，目标散布小于 1 ms。
5. 断开一个电机反馈、IMU、整条 CAN，逐项验证降级和恢复。
6. 调用 clear_faults、disable、rezero（仅失能时）、enable。

验收：所有接口和降级路径与设计一致；无锁存态；恢复不造成 odom 跳变。

## D. 落地低速

1. 清空场地，速度/加速度限制降到保守值，旁站人员持物理断路器。
2. 先直行 0.5 m，再后退、横移、原地小角度旋转。
3. 执行半径已知的低速圆弧，对比 odom、IMU 和地面实测。
4. 逐步提高速度，验证去饱和、门控阈值和 cmd timeout。
5. 按 `calibration.md` 完成轮径、ks/kv/ka、转向参数拟合。

验收：实测轨迹误差满足项目指标，电流/温度处于电机规格内，所有现场参数和原始数据归档。

## 现场记录模板

| 项目 | 结果 | 数据/日志路径 | 测试人 | 日期 |
|---|---|---|---|---|
| 单电机协议与 TIMEOUT | 待执行 |  |  |  |
| p_m 断电持久性 | 待执行 |  |  |  |
| 单模块方向与门控 | 待执行 |  |  |  |
| 8 帧发送散布 | 待执行 |  |  |  |
| 单电机丢帧恢复 | 待执行 |  |  |  |
| IMU 降级/恢复 | 待执行 |  |  |  |
| 总线静默/恢复 | 待执行 |  |  |  |
| 低速直线/圆弧 | 待执行 |  |  |  |
