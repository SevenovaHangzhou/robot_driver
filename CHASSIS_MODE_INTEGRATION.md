# 本地底盘模式集成（2026-10-25）

本目录替换自用户提供的 chassis-mode-runtime 压缩源码。启动默认值改为
`steer_while_driving`；对齐模式通过 `/control/set_chassis_align_first` 请求，
服务确认来自控制周期，实际状态为 `/rt_control/chassis/align_first`。
保留原版运行切换实现及停止检查，不改舵角速度、加速度、jerk、轮径或标定。
测试夹具显式选择 align_first，另有默认连续模式断言。

`src/vendor/robot_interfaces` 从本机已有 Git 对象导出，版本严格为 deps.repos 指定的
`ea582c8a711874813d2ad7f0801b366bd32ab67e`。Ruckig 的现有 ROS Humble 依赖包复制到
`/home/robot/wheel_driver/deps/ruckig`，实机驱动构建不依赖 Gazebo 目录。
编译 CMake 参数为
`-Druckig_DIR=/home/robot/wheel_driver/deps/ruckig/opt/ros/humble/lib/x86_64-linux-gnu/cmake/ruckig`。

只加载实机驱动环境：

```bash
source /home/robot/wheel_driver/driver_env.bash
```

该命令不启动硬件，也不强制设置 ROS_DOMAIN_ID。实际设备启用/运动未在本次执行。
旧源码及导航、仿真修改前快照保存在
`/home/robot/wheel_nav/log/chassis-modes-20261025/before/`。
本地构建与模拟反馈测试日志在同级 log 目录，六段导航验收单独进行。

导航预测统一订阅 `/rt_control/chassis/joint_states`，由驱动以控制轴坐标发布。
它独立于 `/joint_states` 的 URDF 显示符号和广播接口裁剪；失效反馈不续发。
生产模块编号为 1=RL、2=FL、3=RR、4=FR，导航按 2、4、1、3 对应 FL、FR、RL、RR。

The integration also fixes a production setpoint discontinuity at zero yaw rate:
already aligned wheel axes keep equivalent mixed forward/reverse branches when
entering pure translation. Initial unaligned crab targets still choose a common
feasible branch. The regression reproduces the old gate/180-degree target jump;
all driver test groups pass after the change. Simulation and the navigation
snapshot use the same patched implementation and source hashes.
