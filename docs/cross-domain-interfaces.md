# RT-Control 域间接口实现视图

> 对齐公共契约：`robot_interfaces` 0.6.0 @ `e19d1450339d6bce598f664eb18fb093e02097ff`
> 适用实现：`robot_driver main`
> 状态：RT-Control 下游实现说明，不是第二份域间事实源

## 1. 权威关系

域间 endpoint、类型、生产者、消费者和 QoS 的唯一事实源是
[`robot_interfaces`](https://github.com/SevenovaHangzhou/robot_interfaces) 仓库的
`contract/endpoints.yaml`。本仓库只实现 RT-Control，并保存两类接口包：

- `src/interfaces/robot_rt_control_interfaces`、`robot_system_interfaces`：公共契约 0.6.0
  的构建镜像，字段和常量不得在本仓库单独修改；
- `src/interfaces/rt_control_interfaces`：仅供 RT-Control 域内使用，其他域不得依赖。

公共类型发生破坏性变化时，必须先修改契约，再同步本仓库镜像与全部生产者/消费者，
按同一个接口仓库 SHA 原子发布。ROS 2 同名 endpoint 使用不同类型时不会自动转换。

## 2. RT-Control 接收的公共命令

| ID | ROS 名称 | 形式 / 类型 | 生产方 | 当前实现约束 |
| --- | --- | --- | --- | --- |
| R-IN-01 | `/cmd_vel_safe` | Topic / `geometry_msgs/msg/Twist` | Motion | 唯一生产者；20～50 Hz；只使用 `linear.x`、`angular.z`；本地接收超时 0.5 s 后停车 |
| R-IN-02 | `/whole_body_jtc/follow_joint_trajectory` | Action / `control_msgs/action/FollowJointTrajectory` | Motion | 完整且唯一的 14 个机械轴；`allow_partial_joints_goal=false`；整组取消 |
| R-IN-03 | `/control/set_enabled` | Service / `robot_rt_control_interfaces/srv/SetControlEnabled` | 维护/生命周期工具 | `enabled=true` 等待 enable manager 就绪，必要时只自动复位一次，再调用 `/rt/enable`；`enabled=false` 只调用 `/rt/disable` |
| R-IN-04 | `/vacuum/pump/set_enabled` | Service / `robot_rt_control_interfaces/srv/SetPumpEnabled` | 维护工具或 RT 内部管理器 | 活动真空命令、状态不新鲜、阀仍开或可能持箱时拒绝普通停泵 |
| R-IN-05 | `/vacuum/grip` | Action / `robot_rt_control_interfaces/action/VacuumGrip` | Motion | 通道固定 `left/right`；当前只接受 `grip_profile_id=default`；GRIP 要求每通道新鲜 `attached=true`；RELEASE 保持 `UNVERIFIED` |

Autonomy 不得直接调用以上五个控制入口。外部任务必须经过 Autonomy 和 Motion，不得绕过
任务与运动域直达 RT-Control。

## 3. RT-Control 发布的公共状态

| ID | ROS 名称 | 形式 / 类型 | 消费方 | 当前实现约束 |
| --- | --- | --- | --- | --- |
| R-OUT-01 | `/tf` | Topic / `tf2_msgs/msg/TFMessage` | Perception、Motion、Autonomy | `robot_state_publisher` 发布本体动态坐标边；不发布 `map→odom` 或 `odom→base_footprint` |
| R-OUT-01S | `/tf_static` | Topic / `tf2_msgs/msg/TFMessage` | Perception、Motion、Autonomy | 独立 Topic；transient-local；发布本体固定坐标边 |
| R-OUT-02 | `/wheel/odom` | Topic / `nav_msgs/msg/Odometry` | Perception | 50 Hz；`frame_id=odom`、`child_frame_id=base_footprint`；不作为最终到站或停稳证据 |
| R-OUT-03 | `/joint_states` | Topic / `sensor_msgs/msg/JointState` | Motion、Perception、Autonomy | 125 Hz（BQ-135：配置标称 100 Hz 被 250 Hz 控制环量化为 125 Hz，裁决以实测 125 为准）；只含 14 个 EtherCAT 机械轴；仅 position；不含履带控制关节 |
| R-OUT-04 | `/battery_state` | Topic / `sensor_msgs/msg/BatteryState` | Autonomy | 5 s 周期；只读；一键 native 与 Docker 默认启动 BMS 节点 |
| R-OUT-05 | `/vacuum/state` | Topic / `robot_rt_control_interfaces/msg/VacuumState` | Autonomy | 20 Hz；`left/right` 离散 `attached` 状态；Motion 不订阅，其他域不得自行重算 GRIP 成功 |
| R-OUT-06 | `/control/safety_state` | Topic / `robot_rt_control_interfaces/msg/SafetyState` | Perception、Motion、Autonomy | 10 Hz；最大年龄 200 ms；软件可观测摘要，不代表急停、安全继电器或 STO 的真实状态 |
| R-OUT-09 | `/rt_control/readiness` | Topic / `robot_system_interfaces/msg/DomainReadiness` | Autonomy | transient-local；状态变化立即发布，稳定时 1 Hz |
| R-OUT-10 | `/diagnostics` | Topic / `diagnostic_msgs/msg/DiagnosticArray` | 诊断工具 | 只作诊断，不替代 Action Result、SafetyState、readiness 或硬安全链证据 |

RT-Control 不订阅 Perception 发布的 `map→odom` `/tf`。本域运行 `robot_state_publisher`
只是本体 TF 的发布责任，不构成对 Perception TF 的消费依赖。

## 4. 公共 IDL 裁决

### 4.1 使能与真空服务

- `SetControlEnabled.Request`：`enabled`、`reason`；Response：`accepted`、`enabled`、
  `robot_system_interfaces/ErrorInfo error`。
- `SetPumpEnabled.Request`：`enabled`、`reason`；Response：`accepted`、`enabled`、
  `robot_system_interfaces/ErrorInfo error`。公共接口没有绕过持箱保护的 `force` 字段。
- `VacuumGrip.Result`：`accepted`、`overall_verification_level`、
  `robot_system_interfaces/ErrorInfo error`、`channel_results`。
- 三个入口的成功结果必须返回 `error.code=SUCCESS` 且 `retryable=false`；失败的恢复性
  只由 `ErrorInfo.retryable` 表达，消费方不得解析 `code` 或 `message` 控制流程。
- 结构化错误只在非实时 `control_api_adapter` 中从域内结果映射；不改变
  `enable_manager`、PLC 输出服务或 250 Hz 控制环的内部接口。

### 4.2 真空 Action 与状态

- `VacuumGrip` 的 `GRIP=1`、`RELEASE=2`，通道是字符串 `left/right`；
- 现场输入是 PLC 数字量吸附判定，不对域外发布压力值；
- GRIP 只有所有目标通道新鲜且 `attached=true` 才是 `ATTACHED_VERIFIED`；
- RELEASE 只证明阀命令执行完成，物理释放结论始终为 `UNVERIFIED`；
- `/vacuum/state` 的契约消费者只有 Autonomy；Motion 以 Action Result 获取业务终态。

### 4.3 SafetyState 与 DomainReadiness

`SafetyState` 当前字段为：

```text
header
safe_to_start_motion
control_enabled
enable_manager_ok
ethercat_ok
canopen_ok
plc_ok
bms_ok
state
active_faults
```

`DomainReadiness` 当前字段为：

```text
header
ready
state
reason
version
config_summary
```

这些字段以当前 main 的可执行生产者为最终裁决。契约不得引入生产者无法填写的压力、
UUID、schema hash 或硬安全链状态。

## 5. 域内接口

以下 endpoint 在同一 ROS graph 中可见，但只属于 RT-Control 内部实现，不是域间 API：

| ROS 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/rt/enable`、`/rt/disable`、`/rt/reset_fault` | `rt_control_interfaces/srv/RtEnable` | enable manager 内部生命周期事务 |
| `/plc/io_state` | `rt_control_interfaces/msg/PlcIoState` | PLC 原始状态投影 |
| `/plc/left_solenoid`、`/plc/right_solenoid`、`/plc/vacuum_pump` | `std_srvs/srv/SetBool` | 真空适配器调用的 PLC 输出服务 |
| `/rt_internal_state_broadcaster/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | RT diagnostics 内部输入 |
| `/controller_manager/*` | ROS 2 control 标准服务/状态 | 控制器装载、切换和内部观测 |

其他域不得以这些 endpoint 建立依赖。ROS_DOMAIN_ID 和 DDS discovery 不提供域级封装；
“能发现/能调用”不等于“属于公共契约”。

## 6. 已删除接口

以下 endpoint 不再存在，出现在 ROS graph 中即为违约：

- `/robot_model/info`
- `/calibration/info`
- `/navigation/set_base_motion_inhibit`
- `/navigation/base_motion_gate_state`
- `/motion/base_travel_readiness`

Robot Model 与标定版本一致性改由发布清单固定同一 `robot_description` 仓库提交保证，
特殊版本通过独立分支和独立引用处理。

## 7. 实现位置

- endpoint 装配与 remap：`src/rt_control/rt_control_bringup/launch/rt_control.launch.py`
- 控制器频率、14轴集合和速度看门狗：`src/rt_control/rt_control_bringup/config/controllers.yaml`
- PLC/BMS、真空、安全和 readiness 周期：`src/rt_control/rt_control_bringup/config/rt_io.yaml`
- 公共适配器：`src/rt_control/control_api_adapter`
- 公共 IDL 构建镜像：`src/interfaces/robot_rt_control_interfaces`、
  `src/interfaces/robot_system_interfaces`
- 域内 IDL：`src/interfaces/rt_control_interfaces`

修改上述边界后，至少运行公共契约门禁、`tools/quality_gate.sh`、受影响接口包及全部
RT-Control 消费包的 build/test。公共 wire 变更还必须由 Motion、Perception、Autonomy
使用同一契约版本完成联合验证。
