# peripheral-control — PLC、真空、灯带、超声波、电池与IMU

**范围**：PLC、真空、灯带、超声波、电池与IMU。本区对应一个同名职责agent。

**Owner路径**：plc_io_modbus；modbus_tcp_rtu485；bms_node；lpms_nav3_can；control_api_adapter中的vacuum_adapter及其专属测试。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：各包config/machines归hw-config；公共IDL归integration；宿主设备配置归platform。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。源码位置本轮不变。

## 冻结事实

下列事实保留原始记录、日期和验证边界，不将旧实验平台结果外推到新主机。ID中的旧区名仅为历史来源坐标，不表示当前owner。已退役/过期条目仅留历史区，清单见划分decision。

| 历史来源坐标 | 事实 | 原始记录 | 原验证状态 |
| --- | --- | --- | --- |
| canopen-chassis/F5 | LPMS-NAV3 已迁入当前 main 源码基线并纳入构建；默认静态启动、私有 topic、只接收 CAN；不启用三代机共线或正式导航融合 | [LPMS-NAV3 源码接入](../../areas-archive/canopen-chassis/records/2026-09-17-lpms-nav3-main-source-integration.md)#F1-F4 | PARTIAL（T1 无接口/解码测试；连接路径、跨进程 DDS 与实机待验） |
| canopen-chassis/F6 | LPMS 在 node 构造完成至 spin 建立窗口收到 SIGINT 时按正常停止返回 0；context 仍有效的配置/构造异常继续返回 1 | [LPMS 启动窗口 SIGINT 纠错](../../areas-archive/canopen-chassis/records/2026-09-20-lpms-startup-sigint-exit.md)#F1-F2 | PARTIAL（T1 Native；远端完整闭包待确认） |
| canopen-chassis/F7 | LPMS 姿态与角速度协方差支持启动时 YAML 配置，每项九个有限数、默认零，按行写入消息 | [LPMS 协方差配置](../../areas-archive/canopen-chassis/records/2026-10-08-lpms-configurable-covariance.md)#F1-F2 | PARTIAL（T1 参数/转换测试；标定与现场应用待验） |
| contract/14#F1 | 新真空职责为 Autonomy 管泵、Motion 管左右阀，阀输出确认不等于吸牢 | [V3 公共接口实现](../../areas-archive/contract/records/2026-10-02-v3-interface-contract-implementation.md)#F1 | PARTIAL（IDL/源码/Mock；实物待验） |
| io-power/04#F3 | LED YAML 是用户配置而非现场验证证据，设备身份/地址/寄存器映射待验。 | [io-power-20260916-01](../../areas-archive/io-power/records/2026-09-16-modbus-led-driver.md)#F3 | PARTIAL |
| io-power/05#F1 | `modbus_tcp_rtu485` 同包拥有独立 LED 写节点与 E08 超声波只读节点，网络 IO 均不进入 ros2_control 实时环。 | [io-power-20260917-01](../../areas-archive/io-power/records/2026-09-17-modbus-ultrasonic-driver.md)#F1 | 有效 |
| io-power/06#F1 | 四路话题改用 `sensor_msgs/Range`，固定 3.5 m、60 度元数据并使用同一批次完成时间。 | [io-power-20260919-01](../../areas-archive/io-power/records/2026-09-19-ultrasonic-standard-range.md)#F1 | 已由 08#F2/F3 扩展为八路 |
| io-power/06#F2 | `0xFFFD` 无目标发布 `+Inf`，诊断为 `OK/no_target`。 | [io-power-20260919-01](../../areas-archive/io-power/records/2026-09-19-ultrasonic-standard-range.md)#F2 | PARTIAL（离线通过；实机待验） |
| io-power/06#F3 | 默认四个 frame 仅表示 E08 通道，装车后必须用实测外参与 `base_link` TF 替换。 | [io-power-20260919-01](../../areas-archive/io-power/records/2026-09-19-ultrasonic-standard-range.md)#F3 | 已由 08#F2 扩展为八路 |
| io-power/06#F4 | E084F 一次 FC03 连读四个通道，按已确认配置同时测量；驱动不做逐路发射轮询。 | [io-power-20260919-01](../../areas-archive/io-power/records/2026-09-19-ultrasonic-standard-range.md)#F4 | 已由 08#F2/F3 扩展为双 E08 串行轮询 |
| io-power/06#F5 | V3 安装独立 `rt_control_ultrasonic.launch.py`，双臂 runtime 不隐式启动采集，Mock 禁用硬件进程。 | [io-power-20260919-01](../../areas-archive/io-power/records/2026-09-19-ultrasonic-standard-range.md)#F5 | PARTIAL（离线/安装后 Mock 通过；实机待验） |
| io-power/07#F1 | LED 驱动配置为六个控制器，共享 TCP 502，RTU 站号依次为 1..6。 | [io-power-20260920-01](../../areas-archive/io-power/records/2026-09-20-six-led-controllers.md)#F1 | PARTIAL（用户确认配置；实机未验证） |
| io-power/08#F1 | 两台 E08 共用 TCP 504，驱动站号为 unit 1 和 unit 6；站号 2..5 为 E08 保留地址。 | [io-power-20260921-01](../../areas-archive/io-power/records/2026-09-21-dual-e08-eight-ultrasonic.md)#F1 | PARTIAL（协议/离线；第二台实机待验） |
| io-power/08#F2/F4 | 八路 A22 映射为 unit 1 和 unit 6 各四路，发布 `channel1..8`；ROS 视场角固定为 40 度，A22 角度等级 2 待实机配置。 | [io-power-20260921-01](../../areas-archive/io-power/records/2026-09-21-dual-e08-eight-ultrasonic.md)#F2-F4 | PARTIAL（离线通过；实机配置待验） |
| io-power/10#F2 | 当前 CAN 卡接口由用户确认为 `can0`；旧 `can1` USB 命名脚本不适用。 | [双电池流程](../../areas-archive/io-power/records/2026-10-06-dual-golden-phoenix-power-flow.md) | PARTIAL（宿主配置待验） |
| io-power/11#F1 | V3 真空软件按共用泵、左右独立阀和左右独立压力建模；映射缺失时 fail closed | [双侧真空与传感器接口](../../areas-archive/io-power/records/2026-10-02-dual-vacuum-and-sensor-interfaces.md)#F1 | PARTIAL（T1；BQ-152） |
| io-power/11#F2 | 两台 E08 分别发布，单台失败不阻止另一台；Range 年龄 >2.5 s 过期 | [双侧真空与传感器接口](../../areas-archive/io-power/records/2026-10-02-dual-vacuum-and-sensor-interfaces.md)#F2 | PASS（源码/无设备；实机时序待验） |
| io-power/11#F3 | LED 公共入口为完整 RGBW 服务，写确认不等于实际发光 | [双侧真空与传感器接口](../../areas-archive/io-power/records/2026-10-02-dual-vacuum-and-sensor-interfaces.md)#F3 | PARTIAL（源码/协议；实机写待验） |
| io-power/12#F1 | LED 节点正常 SIGINT/SIGTERM 退出时六路写无色，捕获到致命异常时六路尽力写红色；SIGKILL、断电和通信失效不作保证。 | [LED 退出颜色策略](../../areas-archive/io-power/records/2026-10-05-led-exit-colors.md)#F1 | PARTIAL（源码/离线；实机待验） |
| peripheral-control-20261013-01#F1-F3 | 四个外设包的七个协议/传输库头位于 include/<pkg>/protocol，无 ROS include；ROS 转换独立，运行接口及控制行为未修改。 | [外设协议库拆分](records/2026-10-13-peripheral-protocol-library-split.md)#F1-F3 | PARTIAL（T0/T1；109 项四包测试通过，整机闭包与实机未验） |
