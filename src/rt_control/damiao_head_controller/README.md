# damiao_head_controller：达妙头部通信、运行与调参

本包管理三代机两台达妙头部电机的使能、失能和故障复位，是 ros2_control 控制器插件，不能作为独立电机节点直接运行。只启动头部专用入口即可，不需要启动全机器人控制系统。

控制链为：ROS2 位置话题 → ForwardCommandController → robot_hw_can/DamiaoSystem → SocketCAN → 电机。位置速度闭环在电机内部执行；ROS2 不计算电机位置/速度 PI。

## 1. 本次台架与实机事实

本节记录 2026-10-09 会话整理时的事实，现场身份和实验结果不能直接套用于其他机器人。

| 项目 | 本次设置/结果 |
| --- | --- |
| 电机 | DM-J4310-2EC V1.1，两台，已安装头部负载 |
| USB-CAN | gs_usb，USB ID 1d50:606f；实际不是 PCIe CAN |
| 接口/波特率 | can2 / 1 Mbit/s |
| 电机1 CAN ID / Master ID | 1 / 17（0x11） |
| 电机2 CAN ID / Master ID | 2 / 18（0x12） |
| 工作模式 | CTRL_MODE=2，位置速度模式 |
| ACC / DEC / MAX_SPD | 两台读回 2 / -2 / 600 |
| 减速比 | 两台读回 10 |
| 反馈 PMAX / VMAX / TMAX | 12.5 rad / 50 rad/s / 10 Nm |
| 电机 TIMEOUT | 两台设为10000计数，即500 ms，保存Flash得到应答 |
| 正转方向 | 用户报告两台正角度均逆时针，观察侧未记录 |
| 零位 | 用户确认姿态后两台设置零位，重启读回约0.00008637 / 0.00003925 rad |
| 当前配置速度 | 用户要求两台 velocity_limit=0.2 rad/s，约11.46°/s |
| PI对比速度 | 已分析的历史实验为0.1 rad/s，不是0.2 rad/s |

项目中 ID1 对应 head_joint、ID2 对应 head_pitch_joint；物理轴映射、模型正轴和完整机械范围仍需现场核对。模型 ±1.57 rad 是参考值，不代表完整范围已实测。CAN 寄存器通信、零位重启保持、ROS2 使能及用户操作的小幅运动已分别得到证据，但不是完整生产验收。

## 2. CAN、CANopen 与串口的区别

头部达妙使用经典 CAN 上的厂家协议，没有采用 IMU 的 PDO 解码方式。CAN 是总线，CANopen 是建立在 CAN 上的一种上层协议；不能用一个设备的协议判断另一个设备。IMU 相关驱动参见 lpms_nav3_can 的 README。

工控机 COM 通常对应 /dev/ttyS*，USB串口通常对应 /dev/ttyUSB* 或 /dev/ttyACM*。串口不能只靠换接线变成 CAN；连接电机 CAN_H/CAN_L 应使用 CAN 控制器/收发器。DB9只是接插件形状，不能据此认定接口协议。电机UART调参接口与CAN控制接口也不是同一条链路。

## 3. CAN 接口准备

```bash
lsusb
ip -details -statistics link show type can
command -v candump
```

插拔或重启后，USB-CAN可能重新枚举为 can0。先确认设备，再配置；不要修改其他总线。

```bash
# 仅适用于确认目标USB-CAN目前叫can0的情况
sudo ip link set can0 down
sudo ip link set can0 name can2
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up
ip -details -statistics link show can2
```

若已经叫 can2，直接对 can2 配置。状态应为 UP、bitrate 1000000；ERROR-ACTIVE 是正常CAN状态名，不等于已经发生错误。Network is down 表示接口未启动；No such device 表示名称不存在。

```bash
candump -t a can2
```

## 4. 硬件运行配置

本次临时文件为 /tmp/damiao-head-runtime/head_can.draft.yaml。/tmp中的配置、脚本和数据不是永久部署资产，请在需要长期使用时另行归档。以下为本次配置示例，位置范围和时序仍需要现场验证：

```yaml
can_interface: can2
configure_timeout_ms: 100
feedback_timeout_ms: 50
transition_timeout_ms: 250
command_rate_hz: 250.0
disabled_poll_interval_ms: 20
max_rx_frames_per_cycle: 32
joints:
- name: head_joint
  can_id: 1
  master_id: 17
  min: -1.57
  max: 1.57
  velocity_limit: 0.2
  acceleration_krad_s2: 2.0
  deceleration_krad_s2: -2.0
  maximum_speed_rad_s: 600.0
- name: head_pitch_joint
  can_id: 2
  master_id: 18
  min: -1.57
  max: 1.57
  velocity_limit: 0.2
  acceleration_krad_s2: 2.0
  deceleration_krad_s2: -2.0
  maximum_speed_rad_s: 600.0
```

| 参数 | 含义 |
| --- | --- |
| configure_timeout_ms | 初始化阶段寄存器/反馈等待超时 |
| feedback_timeout_ms | 工控机判定反馈过期的时间 |
| transition_timeout_ms | 硬件状态转换等待超时 |
| command_rate_hz | 控制帧发送频率，250Hz约每4ms一次 |
| disabled_poll_interval_ms | 失能状态轮询间隔 |
| max_rx_frames_per_cycle | 每周期最多处理帧数 |
| min / max | 以电机当前坐标解释的目标范围，单位rad；越界拒绝而非截断 |
| velocity_limit | 发送给电机的运动最高速度幅值，输出轴rad/s |
| acceleration_krad_s2 / deceleration_krad_s2 | 电机内部转子加减速度，krad/s²，ACC正、DEC负 |
| maximum_speed_rad_s | 电机内部MAX_SPD，减速前转子rad/s，不是输出轴测试速度 |

所有字段必须通过schema校验，TBD不能启动，Mock也一样。时序示例来自现有集成测试；command_rate_hz=250也对应已记录设计。启动驱动读回模式、ID、超时保护、映射范围和内部运动参数，并校验与配置匹配；不会自动改模式或写这些参数。

修改YAML不会热更新已经运行的硬件实例。先失能，再退出并重启唯一的头部实例，才能加载0.2 rad/s；旧的PI对比结论不因此变成0.2 rad/s测试结论。

## 5. 编译

在 robot_driver 根目录的新终端中，仅加载ROS基础环境，再构建以下包。当前工作区vendor实际位于 src/src/vendor；其他工作区需按实际路径调整。

```bash
# 从你的 robot_driver 仓库根目录执行。
source /opt/ros/humble/setup.bash
colcon --log-base log_head build \
  --build-base build_head --install-base install_head \
  --symlink-install --executor sequential \
  --base-paths \
    src/interfaces/rt_control_interfaces \
    src/src/vendor/robot_interfaces/qos \
    src/src/vendor/robot_interfaces/robot_system_interfaces \
    src/src/vendor/robot_interfaces/robot_rt_control_interfaces \
    src/description/robot_description \
    src/rt_control/robot_hw_can \
    src/rt_control/damiao_head_controller \
    src/rt_control/rt_control_bringup \
  --packages-select robot_interfaces_qos robot_system_interfaces \
    robot_rt_control_interfaces rt_control_interfaces robot_description \
    robot_hw_can damiao_head_controller rt_control_bringup
```

依赖controller_manager、forward_command_controller、joint_state_broadcaster、robot_state_publisher等已安装的Humble包。本例使用独立输出目录；不要把build_head/install_head/log_head等生成物提交到Git。它们目前会被全仓质量检查扫描，应在归档/提交前处理生成物忽略策略。

## 6. ROS2环境与启动

启动和操作终端必须处于相同ROS环境。本次实际控制节点使用域12：

```bash
source /opt/ros/humble/setup.bash
source install_head/setup.bash
export ROS_DOMAIN_ID=12
export ROS_LOCALHOST_ONLY=0
unset RMW_IMPLEMENTATION
```

ROS_DOMAIN_ID隔离发现域；ROS_LOCALHOST_ONLY=1仅本机通信、0允许网络通信；RMW_IMPLEMENTATION=rmw_fastrtps_cpp显式选择Fast DDS，unset使用安装默认实现。省略export不会清除已有变量。可用 printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION 检查。

仅保留一个控制实例，使用带有退出失能流程的入口：

```bash
ros2 run rt_control_bringup rt_control_start --head-runtime \
  head_can_config:=/tmp/damiao-head-runtime/head_can.draft.yaml \
  use_mock_hardware:=false
```

Mock使用 use_mock_hardware:=true，不代表实机通过。启动时管理器和状态发布器ACTIVE、位置控制器INACTIVE，不会自动使能；硬件初始化会发送失能并以当前反馈位置初始化保持命令。

另一个已加载相同环境的终端检查：

```bash
ros2 service list --no-daemon | grep /rt/head
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

## 7. 使能、位置话题、清错和退出

三个生命周期服务都是 rt_control_interfaces/srv/RtEnable，空请求，返回ok、failed_batch、failed_joint、status_word、stage。

```bash
ros2 service call /rt/head/enable rt_control_interfaces/srv/RtEnable '{}'
```

等待ok:true，管理器确认两台使能后激活位置控制器。仅使能不会执行额外角度目标。

```bash
# 示例：确认范围、方向和保持位置后才能发送
ros2 topic pub --once /head_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.11, 0.0]}'

# 两台返回零位
ros2 topic pub --once /head_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0]}'
```

数组顺序[head_joint, head_pitch_joint]，即项目配置的ID1/ID2；单位rad，绝对位置，不是增量。0.11 rad约6.30°。另一台若应保持不动，应填其当前保持位置而非随意填0。发布成功只说明消息已发送，不等于到位。

该控制器不是joint_trajectory_controller，没有动作时长或插补轨迹接口。--once只发布一次，节点继续执行最后目标；新目标覆盖旧目标。0.1 rad/s下走0.11 rad至少约1.1秒，0.2 rad/s下至少约0.55秒，不包含完整加减速和收敛时间。

```bash
ros2 topic echo /joint_states
ros2 topic echo /diagnostics --once

# 清错：不是话题；停止位置控制、请求失能，再清除并等待失能终态
ros2 service call /rt/head/reset_fault rt_control_interfaces/srv/RtEnable '{}'

# 结束控制：先失能，再在启动终端Ctrl+C
ros2 service call /rt/head/disable rt_control_interfaces/srv/RtEnable '{}'
```

到位后仍使能保持。查看反馈终端的Ctrl+C只停止查看。清错不会自动重新使能，故障根因未消除可能再报错。诊断名称为 /robot/rt_control/damiao_head/summary，发布在 /diagnostics；重点查看phase、fault_latched、motion_allowed及两台feedback_age_ms。

## 8. CAN ID、Master ID及报文示例

CAN ID标识电机控制地址；Master ID决定该电机发回报文所用的CAN帧ID。方向和报文类型不同，ID并非必须相同。0x7FF只用于厂家寄存器操作，不是所有控制帧都使用它。

| 报文 | CAN帧ID |
| --- | --- |
| 寄存器读写/保存 | 0x7FF，数据内携带目标电机ID |
| 位置速度控制/特殊控制 | 0x100+电机ID，本次0x101/0x102 |
| 电机反馈 | Master ID，本次0x011/0x012 |

### 参数读取

数据为[电机ID低字节，高字节，33，寄存器编号，四字节占位]，读回值为uint32或float32小端，按寄存器类型解释。

```bash
# 电机1模式，预期011#0100330A02000000，值2
cansend can2 7FF#0100330A00000000
# 电机1 Master ID、CAN ID、TIMEOUT
cansend can2 7FF#0100330700000000
cansend can2 7FF#0100330800000000
cansend can2 7FF#0100330900000000
# 电机2模式
cansend can2 7FF#0200330A00000000
```

### 特殊控制与零位

当前模式使用101/102；七个FF加末字节FC使能、FD失能、FE保存当前输出轴零点、FB清错。零点命令也将位置给定置0。以下直接操作只能在ROS硬件实例停止、状态明确时使用，避免与控制器争用总线。

```bash
# 失能
cansend can2 101#FFFFFFFFFFFFFFFD
cansend can2 102#FFFFFFFFFFFFFFFD
# 确认姿态且失能后设零；不要在不确定姿态下执行
cansend can2 101#FFFFFFFFFFFFFFFE
cansend can2 102#FFFFFFFFFFFFFFFE
# 清错
cansend can2 101#FFFFFFFFFFFFFFFB
cansend can2 102#FFFFFFFFFFFFFFFB
```

零位对齐模型意味着机械零姿态与电机0 rad一致，不仅是数字置0。用户本次确认当前姿态后设零；断电后小偏差的近零读回证明此次保持，但不证明模型轴向或整个机械范围均正确。

### TIMEOUT与Flash

TIMEOUT寄存器0x09为uint32，每计数50us=0.05ms，仅使能时生效；0表示未启用，本驱动拒绝0。500ms×20=10000=0x2710，小端10 27 00 00。它不同于工控机feedback_timeout_ms。

```bash
# 两台已失能后设置500ms
cansend can2 7FF#0100550910270000
cansend can2 7FF#0200550910270000
# 读回，预期末四字节10270000
cansend can2 7FF#0100330900000000
cansend can2 7FF#0200330900000000
# 两台已失能时保存全部当前电机内部参数
cansend can2 7FF#0100AA0100000000
cansend can2 7FF#0200AA0100000000
```

应核验写入/保存应答与读回；保存命令可能返回4字节前缀。Flash保存会保存全部当前内部参数，不只TIMEOUT。YAML的限位、velocity_limit和ROS通信参数不写入电机Flash。Flash写次数有限，试调不要反复保存。本次500ms保存已应答，但保存后断电保持尚未在本记录中验证。

### 反馈解析

八字节反馈：[ID|ERR<<4, POS高8位, POS低8位, VEL高8位, VEL低4位|转矩高4位, 转矩低8位, MOS温度, 线圈温度]。

ERR=0失能、1使能；3输出轴校准异常、4传感器异常、5电机编码器校准异常、8过压、9欠压、A过流、B MOS过温、C线圈过温、D通信丢失、E过载。

本次首字节01/02代表两台失能，11/12代表两台使能。温度字节按摄氏度整数。POS16位，速度/转矩12位：实际值=原始值×2×映射上限/(2^位数−1)−映射上限。必须使用电机实际PMAX/VMAX/TMAX；映射范围不是机械限位或安全测试速度。本次速度量化步长约0.02442 rad/s，零速可能解码为±0.01221 rad/s，不能把单个反馈值认定为真实爬行。

## 9. 服务不可发现与重复实例排查

waiting for service to become available表示尚未到达服务，不是电机返回失败。先看启动终端是否还运行，再检查：

```bash
pgrep -af '[r]os2_control_node'
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
ros2 daemon stop
ros2 service list --no-daemon
```

本次明确发现过控制节点域12、调用端域0，导致服务不可发现；统一环境后恢复。也发现两个控制节点同时连接can2，一个使能另一个持续失能，导致ok:true之后电机又失能。不同ROS域不会隔离同一CAN硬件：同一总线只能有一个本项目控制实例。

使能成功后若失能，检查诊断、反馈状态和发送帧，不能只看服务返回。本次另修复switch_position_controller中临时ListControllers响应生命周期问题：保留shared_ptr再遍历。编译安装、2个CTest目标和20次Mock失能通过；此前段错误与后续正常退出需区分。

## 10. PI调参实验与结论

当前电机内部读回基线：

| 参数 | 寄存器 | 两台基线 |
| --- | --- | --- |
| 电流带宽I_BW | 0x18 | 1000 |
| 速度P KP_ASR | 0x19 | 约0.00372 |
| 速度I KI_ASR | 0x1A | 约0.002 |
| 位置P KP_APR | 0x1B | 54 |
| 位置I KI_APR | 0x1C | 0 |
| 阻尼Deta | 0x1F | 4 |
| 速度滤波带宽V_BW | 0x20 | 40 |

依据协议第10页，位置速度模式采用内部串级环；第18～19页列出增益寄存器。MIT控制帧的Kp/Kd与本模式内部PI不是一回事；MIT位置控制Kd不能为0的说明不能直接套用。Deta不能直接当成通用PID的D。文档第10页与后部参数描述对阻尼作用有不同表述，应结合固件和厂家说明，不猜测效果。

用户报告带负载运动中抖动；记录显示位置总体能到位、保持较稳定，运动中速度与转矩明显波动。速度反馈有量化/估计误差，摩擦、机械间隙也可能参与，曲线不能唯一归因于某个增益。

本次只在电机1试调位置P 54→48.6（降低10%），其余参数不变，未保存Flash。随后按用户要求恢复54。恢复后同一0.5→-0.2rad路径的公共0.45→0.05rad区间对比：

| 指标 | P=54恢复后 | P=48.6 |
| --- | --- | --- |
| 速度波动标准差 | 0.0638rad/s | 0.0634rad/s |
| 去除负载趋势后转矩波动 | 0.0609Nm | 0.0623Nm |

差异很小，单轮不足以证明位置P降低有改善。下一步优先排查速度环和机械低速特性；内环到外环顺序为保留厂家电流环→速度环→位置环。速度P 0.00372→0.003348曾计划，但操作中断，之后读回仍为原值0.00372；不要写成已完成速度环试验。

A/B测试要求：相同负载、方向、范围和速度，每次只改一个参数，备份原始float字节，失能后写入并读回，再由现场使能测试。比较速度波动、转矩波动、位置平顺性、超调、到位误差、温度和实际体感；效果确认前不保存Flash。当前配置已请求0.2rad/s，不能与0.1rad/s历史数据直接作为等条件A/B比较。

## 11. 实时曲线脚本和采样数据

本次脚本 /tmp/damiao-pi-review/live_can_plot.py 仅监听，绝不发送CAN帧；它没有安装进ROS包。两列分别显示两台的位置/目标、速度/限速、转矩，标题显示状态、温度、反馈年龄。需保持唯一的控制节点运行，以便持续产生反馈。

工控机桌面终端运行：

```bash
MPLCONFIGDIR=/tmp/damiao-pi-review/mpl \
python3 /tmp/damiao-pi-review/live_can_plot.py --interface can2 --window 20
```

非桌面环境录60秒并输出最后60秒图像：

```bash
python3 /tmp/damiao-pi-review/live_can_plot.py \
  --headless --duration 60 --window 60
```

默认输出 /tmp/damiao-pi-review/plots/ 下的CSV和PNG，CSV保留全部接收样本；窗口仅显示最近window秒。关闭窗口或Ctrl+C停止监听，不会失能电机。映射默认12.5/50/10来自本次实机，其他设备用 --pmax/--vmax/--tmax指定实测值。脚本不读取或修改PI。依赖Python3、numpy、matplotlib及桌面绘图后端；当前环境验证了无窗口收流与输出，未验证桌面窗口。

临时证据包括：baseline.json（PI备份）、motion-live-round2.jsonl（原值）、position-p-trial-transactions.json、position-p-restore-transactions.json、position-p-restored-comparison.png/json。零位与Flash操作分别记录在 /tmp/damiao-head-zero/transactions.json、/tmp/damiao-flash-save-500ms/transactions.json。原始日志时间遵循当时宿主时钟，不据此推断用户日期或绝对时间准确性。
