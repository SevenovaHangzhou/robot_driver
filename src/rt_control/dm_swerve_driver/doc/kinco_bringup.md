# Phase 8：Kinco FD / IgH / BRT 编码器

完整标定资料：[标定说明](swerve_calibration_overview.md)、
[现场手册](swerve_calibration_manual.md)、[记录模板](swerve_calibration_record_template.md)。

节点入口为 `swerve_driver_node`，运行路径固定为 Kinco EtherCAT。
模板在 `config/kinco_params.yaml`；硬件事实字段的 0 值会拒绝配置。
总线分工已确认：8 台电机使用 EtherCAT，4 个外置 BRT 编码器保留 CANopen。
真实 EtherCAT 电机总线及 CANopen 编码器台架验收尚未执行。

## 构建与启动

目标 Linux 主机安装与内核主站一致的 IgH 1.6 用户库和头文件，配置专用网卡。
默认构建不链接 IgH；选择 Kinco 激活时会明确报缺少后端，绝不会自动改用 fake。

```bash
colcon build --packages-select dm_swerve_driver --cmake-args -DDM_SWERVE_ENABLE_IGH=ON
source install/setup.bash
ros2 launch dm_swerve_driver swerve_driver.launch.py params_file:=/absolute/path/kinco_params.yaml
```

先填写真实轮径、轴齿比、方向、8 轴电机分辨率、slave position 顺序、4 个编码器
`0x6501/0x6502` 期望值、精确整数齿数、主备差异阈值和允许停放位移。
`dc_cycle_ns` 必须等于 `1e9/control.rate_hz`；FD 支持的周期范围及 watchdog
停止时间需要台架确认。设置带写权限的快照路径，提前创建父目录。

## 首次标零

1. configure 后保持 inactive，架起底盘，用治具将四轮平行车体 +x。
2. 调用 `~/rezero_steering`。服务在禁能状态读取电机及编码器绝对位置，生成安装偏移。
3. 成功后写入快照及同路径加 `.calibration` 的标定文件；以后启动自动加载。
   该服务仅记录软件偏移，保持 BRT 0x6003 预置值。
4. 激活节点。首次启动缺少快照、两绝对源不同意、机械范围不可表示或编码器规格
   不一致时均拒使能；修正后重新激活。

两源有分歧时不可用标零掩盖机械故障。更换编码器、改变齿数或安装后重新治具标定。
标定文件与现场 YAML 一起归档；故障停机时不覆盖可信编码器快照。

## 实际周期链路

启动：IgH 配置 PDO/DC/watchdog、禁能采样 → BRT 身份与持久值/两绝对源检查
→ CSP 当前角预载 → DS402 0x06、0x07、0x0F → 全八轴状态/模式确认。
BRT 启动检查 TPDO2 映射为 0x6004:00/32 位且关闭缩放，配置同步传输和 100 ms 心跳，
收到 SDO ACK 后发送 NMT Operational。

每周期一次八轴 EtherCAT exchange、一次 CANopen SYNC。IgH 使用 0x1601/0x1A01
显式重映射：转向写 CSP 位置，驱动写 CSV 速度，反馈含 0x6041、0x6061、0x6064、
0x606C、0x2601、0x2602。0x2602 暂按未知硬故障锁存。
前馈由驱动器位置环处理；上位机 CSP 不发送转向速度前馈。

驱动/逻辑欠压允许按既定次数和间隔恢复，其余错误锁存。任一轴离线或 WKC/AL
异常使全车驱动目标归零。复位脉冲和重使能嵌入正常 PDO 批，不额外发送恢复批。
外置编码器掉线可用已对齐的电机备份；两者都不可用或两源分歧则锁存。
恢复外置源时首周期连续，然后按参数步长回归其绝对角；原始角也参与机械限位检查。

`/diagnostics` 提供 8 轴、4 编码器及汇总状态，包含原始错误字、WKC、AL/链路、
恢复次数、心跳、主备来源及打滑模块。缺失 IMU 延续轮式 yaw 降级。
差模打滑会剔除不一致轮并放大 odom 协方差；共模四轮同滑的残差可能为零，
须由上层 EKF 融合其他传感器识别。

## 台架待验收

- FD 最小/最大 DC 周期、watchdog 实测停机时间；拔 EtherCAT 线必须验证从站自行停止。
- 真实 PDO 重映射、模式显示、0x2601/0x2602 故障注入与复位效果。
- 位置环 0x60FB 参数的工程量/内部值换算；ESI 默认速度前馈 raw=256 与手册工程量
  100% 不同，参数明确命名为 `position_velocity_feedforward_raw`，直接写原始值，
  不将百分数当作内部值。未核准前保持 `write_position_feedforward=false`。
- 0x605E 停止模式和驱动器默认保存值；可显式配置 0、1、2，现场验证实际减速/制动。
- BRT 6501/6502 实际值、PDO/心跳、断电多圈保持、齿轮两侧逼近背隙及主备切换。
- +179° 到 -179° 的区间内转向轨迹与全车驱动对齐门控。
- CST 仅保留底层模式/数据类型，当前运行路径为 CSP/CSV；需实测确认 CSV 低速不足后
  再接入闭速度环与经核准的力矩单位，不能把 CST 类型声明当作可运行 CST。

IgH API 依据：[EtherLab 官方 1.6 接口](https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html)。
本地依据为桌面的 Kinco FD ESI、低压伺服手册及 BRT 多圈 EDS/协议手册。
