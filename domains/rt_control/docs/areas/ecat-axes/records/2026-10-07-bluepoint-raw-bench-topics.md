---
id: ecat-axes-20261007-01
area: ecat-axes
title: 双 P140000100 台架原始话题实际通信
date: 2026-10-07
type: feature
trigger: 用户要求修改代码确保实际力传感器话题通信成功
commits: []
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence:
  - "左右位置2/4，Vendor0xA1/Product0x8081/Revision1，6 responder"
  - "两台SDO Info确认六通道/温度int32、状态/计数uint32；Rx额外字段float零字编码"
  - "两台0x1C32:04=0x401F，0x1C32:05=100000 ns"
  - "初轮8包构建通过；可复现构建脚本9包构建通过，101项聚焦测试通过"
  - "Mock独立进程5秒订阅：left4636/right4634条，wrench0条，左右诊断可读"
  - "DC/1ms实机独立进程5秒订阅：left4956/right4955条，wrench0条"
  - "采集期间传感器2/4为OP，ZeroErr1/3保持PREOP；主设备0为OP，子设备5为PREOP"
  - "Ctrl-C退出0，主站恢复Idle/Inactive，全部6从站回到PREOP"
supersedes: []
related: [BQ-149, ecat-axes-20260917-01]
---

## 行为

新增 `rt_control_force_sensors.launch.py` 和 `force_sensor_runtime.py`。独立台架入口仅配置
左右两台GenericEcSlave及诊断状态接口，不载入电机模块/enable_manager。使用同一IgH
主站与同一controller_manager加载两个已有C++ broadcaster，不建立第二硬件访问旁路。

真实启动前只读检查master Idle/Inactive、Link UP、6 responders，以及位置2/4的
PREOP/+、Vendor/Product/Revision和完整PDO。错误立即拒绝。默认Mock；真实模式必须
显式选择sm/dc。本次实机使用dc，1ms周期，固定零RxPDO，保留默认PDO，无启动SDO列表。
本次验证只覆盖DC模式，SM模式尚未实机验证。

参数从草案转换为标准controller参数，补齐startup_id、link/AL接口。raw-only模式使用
传感器局部_raw标签满足插件要求，既不发布TF，也不声明机器人物理坐标系。
calibration_valid固定false，故不发布有效wrench；诊断ERROR表示工程单位未确认，不能
据此认定设备故障。raw数据排列Fx/Fy/Fz/Mx/My/Mz，不含温度或采样计数。

mock和实机都由另一rclpy进程订阅左右Int32MultiArray及transient-local诊断；验证每条
数据含6项、双侧持续收流、左右诊断可读且wrench没有数据。测试ROS_DOMAIN_ID=73。

实机最后样本：left=[-137359159,118927553,173392644,3435711,-2165001,41045636]，
right=[70052,213906,53955,-766,181,1794]。左侧量级明显较大，本次不认定其标定/量程/单位
正确，不直接乘参考比例作为可信物理力值。运行期数据新鲜度和StatusCode语义未闭合。

## 构建与启动

从仓库根执行：

```bash
bash tools/build_force_sensor_bench.sh
source /opt/ros/humble/setup.bash
source /tmp/rt-control-force-bench/install/setup.bash
ros2 launch rt_control_bringup rt_control_force_sensors.launch.py use_mock_hardware:=true
```

构建脚本在独立目录复制冻结EtherCAT依赖，并按顺序应用现有13个补丁，避免使用未补丁
上游驱动（后者缺少固定PDO保留和link/AL状态导出）或修改用户vendor工作树。
支持src/vendor与当前导入的src/src/vendor布局。需要ROS2 Humble、colcon以及已导入的
冻结ecat_icube/robot_interfaces依赖。路径参数可替换默认临时构建根。

已确认本台架接线及主站空闲后启动真实raw采集：

```bash
ros2 launch rt_control_bringup rt_control_force_sensors.launch.py use_mock_hardware:=false sync_mode:=dc
```

其他终端source相同环境（ROS_DOMAIN_ID也必须一致）后：

```bash
ros2 topic echo /rt_control/left_wrist/raw
ros2 topic echo /rt_control/right_wrist/raw
ros2 control list_controllers -c /controller_manager
ros2 topic echo /rt_control/bluepoint/calibration --qos-durability transient_local
```

Ctrl-C停止采集。不可与另一个使用master0的实时控制进程同时运行。

## 限制

只有6从站台架和左2/右4映射得到验证，不自动替换完整V3双臂拓扑，不解除其draft门禁。
单位比例、物理TF、计量标定、状态码、新鲜度、长期DC和负载下稳定性仍需验证。
停机控制器/硬件停用成功，主站已回Idle且所有从站PREOP；退出时上游pluginlib报告
library对象仍存活的卸载警告，进程退出0。本次未更改上游插件析构流程。


## YAML 对应的九路命名状态 — 2026-10-07

配置新增可选 `sensor_state_topic`：左 `/rt_control/left_wrist/state`，右
`/rt_control/right_wrist/state`。现有 broadcaster 从同一实时读取样本发布
`control_msgs/msg/DynamicJointState`；不创建另一个硬件读取节点。`joint_names` 为
YAML 的 `sensor_name`，`interface_values[0].interface_names` 按 `value_interfaces`
再 `auxiliary_interfaces` 的顺序保留完整配置名称，`values` 为对应值：六个通道、
状态码、采样计数、温度，共九项。预分配消息并使用 realtime publisher trylock；
uint32 辅助值以 double 无损传递。原 `/raw` 仍只含六路 int32，工程单位和标定门禁不变。

重建并正常停止旧台架进程后，在原 ROS_DOMAIN_ID=12 恢复真实 DC 采集。
另一进程订阅两侧新话题，并逐条验证名称/长度与 YAML 一致。3 秒收到左2635、
右2634条；分别观察到1323、1320个不同采样计数（表明短窗口内计数持续变化，
不替代闭环新鲜度门限验证）。末次原始样本：

| 接口尾名 | 左 | 右 |
|---|---:|---:|
| channel_1_raw | -137358183 | 67240 |
| channel_2_raw | 118931187 | 231762 |
| channel_3_raw | 173395650 | 70211 |
| channel_4_raw | 3435708 | -931 |
| channel_5_raw | -2164927 | 245 |
| channel_6_raw | 41045982 | 1885 |
| status_code_raw | 0 | 0 |
| sample_counter_raw | 3290120 | 3290106 |
| temperature_raw | 0 | 0 |

状态位与温度缩放未确认，不能把温度 raw=0 解释为0℃。采集在验证结束后继续运行。
查看当前数据：

```bash
source /opt/ros/humble/setup.bash
source /tmp/rt-control-force-bench/install/setup.bash
export ROS_DOMAIN_ID=12
ros2 topic echo /rt_control/left_wrist/state --once
ros2 topic echo /rt_control/right_wrist/state --once
```

验证：两个包增量构建成功；13项C++测试与6项参数转换测试通过；质量门禁207项
通过、13项跳过；C++格式检查通过。独立mock订阅3秒收到3001/3000条命名消息。
C++覆盖名称顺序、时间戳及4294967295辅助值无截断；恢复链路测试按连续更新驱动，
允许实时发布锁竞争时丢弃单帧。


## 用户选择 PDF V1.1 地址 — 2026-10-07

左右现有配置已按 PDF 改为 RxPDO 0x1600 单项0x2000:00固定零控制字；TxPDO
0x1A00为0x4000～4005:00六路int32、4007:00计数uint32、4008:00温度int32。
PDF未定义第七项状态码，已从profile、family及broadcaster配置移除；state共八项。
保留实机扫描身份，不捏造P140000107身份。当前布局来源改为参考PDF，运行验证标记
false。上述历史通信证据对应旧0x6000/7000映射，不能用于当前配置准入。

启动前检查改为从profile生成期望PDO列表，保留固定PDO/无启动SDO条件，确保旧实机
映射不能通过当前PDF配置的检查。12项配置/启动转换测试通过，包括旧映射拒绝用例。
重新对位置2/4的八个PDF输入地址逐项upload，全部返回0x06020000（对象不存在）；
未启动不匹配配置、未写SDO、未重映射设备。参考PDF布局与当前固件不兼容，实际
温度为零的原因仍未闭合。曾通过子进程调用遇到/dev/EtherCAT0不可见，随后直接命令
已读到上述设备abort结果，不把环境错误当设备返回值。


## PDF 配置节点启动核验 — 2026-10-07

用户要求节点采集具体值。隔离构建9包通过，随后实际执行
`ROS_DOMAIN_ID=12 ros2 launch rt_control_bringup rt_control_force_sensors.launch.py use_mock_hardware:=false sync_mode:=dc`。
launch退出1：`Force sensor 2 PDO layout mismatch`，硬件节点未启动，未产生新实机话题数据。
左右只读`ethercat pdos -p 2/4`均确认Rx7000:01..08、Tx6000:01..09（含StatusCode/Temper）。
当前PDF选择布局为Rx2000:00、Tx4000..4005/4007/4008:00，与实机不匹配；不绕过检查。
首次沙箱读取提示/dev/EtherCAT0不可见，获准的沙箱外只读检查成功，故实际阻断原因
已确认为布局不匹配。配置保持用户指定PDF版本，实际恢复6000/7000采集需用户选择。


## 新 DOCX 协议配置 — 2026-10-07

用户改为指定`三代机电气件通讯协议/力传感器/三代机/前置产品信息/六维力传感器-EtherCAT通讯协议V1.1(1).docx`，
日期2026-09-10，SHA256 9c275c21bb171a99cd814c2a041b8bc9b03774ef1034cac523104c1e63ab8cdf。
该文档映射对象与此前实机一致：Rx7000:01..08、Tx6000:01..09，映射容器保留1600/1A00。
左右profile、family、broadcaster九路接口已同步恢复，保留输出固定零。
REAL保留输出仍为uint32零字编码，不导出命令。启动检查继续按profile生成期望列表。

力/力矩比例改为100000 raw每N/N·m（scale0.00001、decimals5）。StatusCode定义为保留；
Temper为保留，文档写内部未接PT1000时为满量程，但没有定义满量程数值/温度比例。
移除此前温度raw/10的依据，不认定raw=0代表未接温度器件。固件注明3.6.1.3-ECG，与
此前实机3.6.1.0不同，因此calibration_valid及verified继续false，不发布有效wrench。
本次12项profile/启动转换测试通过，旧PDF映射拒绝用例通过；未重新启动实机或验证比例。
历史PDF/SDO abort结果保留作为过程记录，SDO失败不能单独代替PDO映射证据。


## DOCX 配置实机话题复测 — 2026-10-07

当前配置重新构建9包成功，ROS_DOMAIN_ID=12真实DC/1ms启动，两台传感器OP且完整working counter，电机1/3保持PREOP。独立rclpy进程预热后订阅5秒，检查九项名称与YAML一致、整数/有限值、时间戳/计数顺序和wrench门禁。

left: state=5005，raw=5004，wrench=0；名称错误/非有限值/计数倒退/时间戳倒退均0，消息约1000.00Hz，最大消息时间戳间隔1.0107ms，计数变化2502次（约500次/秒），从3096770到3101774。发布1kHz不等于独立新样本1kHz，重复计数不代表新样本。

末条原始九项：[-137359099.0, 118953426.0, 173473178.0, 3436116.0, -2164148.0, 41050758.0, 0.0, 3101774.0, 0.0]；六通道观察范围min=[-137362026.0, 118950330.0, 173468559.0, 3436031.0, -2164247.0, 41050071.0], max=[-137357019.0, 118957686.0, 173479210.0, 3436181.0, -2163957.0, 41052052.0]。

right: state=5005，raw=5005，wrench=0；名称错误/非有限值/计数倒退/时间戳倒退均0，消息约1000.00Hz，最大消息时间戳间隔1.0107ms，计数变化2502次（约500次/秒），从3096766到3101770。发布1kHz不等于独立新样本1kHz，重复计数不代表新样本。

末条原始九项：[66141.0, 243502.0, 85604.0, -1482.0, 268.0, 2832.0, 0.0, 3101770.0, 0.0]；六通道观察范围min=[64162.0, 239497.0, 81768.0, -1546.0, 234.0, 2726.0], max=[70082.0, 245674.0, 88179.0, -1426.0, 349.0, 2877.0]。

按新DOCX比例暂算：左Fx=-1373.59099N、Fy=1189.53426N、Fz=1734.73178N、Mx=34.36116Nm、My=-21.64148Nm、Mz=410.50758Nm；右Fx=0.66141N、Fy=2.43502N、Fz=0.85604N、Mx=-0.01482Nm、My=0.00268Nm、Mz=0.02832Nm。此为文档比例算例，不是有效wrench或已验证计量结果。左Mz超过选型资料120Nm，当前负载/机械型号/零点及固件比例适用性未确认，不能判定物理数据正常；未执行硬件清零。

StatusCode和Temper均持续0，按新文档为保留字段，不认定无故障/0℃/未接PT1000。测试后SIGINT正常退出0，硬件停用/关闭成功，主站恢复Idle且全部PREOP；仍有既有class_loader卸载警告。通信/原始收流通过，标定、温度有效性与1kHz独立新样本未通过验收。
