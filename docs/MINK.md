# Mink 直接仿真控制（对照入口）

本页描述 `script/start.sh --backend mink` 的原直接仿真入口。当前默认后端已经改为 `mink-rolling`，见 [Mink → Rolling](MINK-ROLLING.md)。两个后端都支持网页实际模型及 `--viewer` 原生窗口，默认端口均为 18131；同一进程的网页与窗口才共享仿真状态。

完整控制链路图、各层频率及数学量见 [Mink 链路图](MINK-CHAIN.md)。

## 模型和依赖

`--real2sim` 指向用户提供的 `demo_mink.sh` 所在目录，默认相对于本项目定位 `../../sim/real2sim_v3`。复用其中的 `MinkFollower`，包括双臂 FrameTask、PostureTask、ConfigurationLimit、VelocityLimit、DisplacementCollisionLimit 和 150 ms 物理制动预测，没有复制修改外部工程。

浏览器读取同一工程的便携 `urdf/robot.urdf` 与 `meshes/legacy`，由 `ModelAssets` 限制资源路径在显式根目录内。启动核对双臂关节名称、关节限位及初始位姿的 TCP 位置/朝向。Mink 1.3.0、MuJoCo 3.15.0、DAQP 和 qpsolvers 在 `pyproject.toml` 固定版本；更换模型或升级求解器需重新验证。

## 输入及执行

`POST /api/jog/velocity` 接受 `client`、`epoch`、单调递增 `seq`、`robot_direction` 和 `speed_mm_s`。方向使用机器人本体 `base_link`；服务端从同一 URDF 映射到 `arm_carriage`，再转换到 MuJoCo 世界系。连续模式不接受步长或 `/step` 分段命令。按下仅发送一次启动，100 ms 保活独立续期。

速度档位为 0.5、1、2、5、10、20、30、50 mm/s，最高 50 mm/s；100 mm/s 已从页面和后端允许列表移除。20、30、50 mm/s 已通过初始姿态向前移动的实际跟随验证。选择的档位表示目标推进速度，实际速度仍受跟随能力和现有限制约束。

单独的物理线程每 10 ms 推进一次，目标速度变化限制为 50 mm/s²；Mink 的关节速度限制沿用参考配置。每次求解后，MuJoCo 以 2 ms 步长推进五次。实际末端可能存在跟随误差和短暂速度超调；界面明确显示实际速度。控制循环长期落后超过 100 ms 时退出接管，不用追赶积压时间批量补发目标。

松键 `/halt` 保留控制权并旋转代次，先丢弃积累的目标滞后，再减速停止；等待实际关节速度降低后允许新输入。Esc、失焦、失联、400 ms 输入保活超时、控制权过期、碰撞预测阻挡或跟随误差超过 3 mm 均取消方向并退出接管。求解异常使反馈状态不可用。恢复后需要新的主动接管，旧代次和过期请求不能恢复运动。

HTTP 线程与物理线程共用锁；HTTP 只更新输入状态或读取实际状态，不推进物理步长。原生窗口显示同一份 MuJoCo 数据，鼠标拖动不会替代键盘目标。窗口初始化完成后才启动物理与 HTTP 线程，避免初始化读取与物理更新并发。原生窗口启动前检查活动显示输出；没有显示器或远程桌面的虚拟显示输出时明确报错，可用 `--headless` 继续使用网页。关闭原生窗口或 Ctrl+C 会停止网页服务和本次仿真线程。

## 反馈和边界

状态中的 `input_mode: continuous`、`publication.mode: mink` 和 `feedback_source: mujoco_actual` 驱动网页模式显示。`model_q_rad`、`q_rad`、`tcp_mm`、`tcp_actual_speed_mm_s` 来自实际 `qpos/qvel` 与配套模型 FK/Jacobian；网页同时独立检查渲染模型的 TCP 对齐误差。运动卡片报告累计实际位移、速度和实际模型路径偏差，无固定段终点。

本页的直接模式仅为本地仿真。新增 `RollingStream` 已提供 Mink 未来轨迹到 Rolling 传输的适配，默认用本地协议模拟器验证，未连接真实电机。避碰基于参考示例已有几何和凸包近似，不保证实机间距、动态制动或绕障。历史 `manual_sim.py` 仍含 FJT 启动准备，不在默认启动链路中。
