# rt-control 开发进度与联调准入

> **状态：允许进入同机工程联调，尚未达到整机生产发布。** motion 等域可以使用
> 当前接口验证发现、消息格式、轨迹/速度链路和取消/失败流程；所有接口仍属于
> “当前工程接口”，公共域间契约冻结前不承诺名称、QoS 或错误语义长期兼容。

当前目标机硬件运行基线仍为 `4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d`，目标机仅为
`ar@192.168.0.40`。T-020 的新 TF 根链已完成源码和容器 Mock 验证，但需要构建、部署新镜像后才会替换该运行基线。

## 1. 当前已完成

### 控制执行

- 12 个手臂关节、Turn、EtherCAT Updown 组成完整 14 轴 JTC/FJT；
- Updown 已从 CANopen Node 1 迁移为 EtherCAT position 15 的 CSP；
- 左右履带保持 CANopen Node 2/3，由 diff-drive 接收速度命令；
- 14 轴当前位置预装载、分批使能、失能、整组复位和故障整组处理已实现；
- FJT 会拒绝 partial goal、第一点超差或 EtherCAT 反馈过旧的目标；
- 履带在无新命令时按 controller 限制回到零速。

### 状态与生命周期

- 输出关节状态、FJT feedback/result、里程计、导航/视觉共用 TF 和诊断；
- 新 TF 合同为 `odom → base_footprint → base_link → 本体/传感器连杆`，不再由模型发布 `world → base_link`；
- `/rt/enable`、`/rt/disable`、`/rt/reset_fault` 返回明确的阶段、失败批次、关节和
  原始 statusword；
- 正常退出会先失能并释放 EtherCAT，再清理 CANopen，避免粗暴释放主站；
- 仓库已新增当前工控机专用的一键启动入口，在容器 ready 后自动调用 `/rt/enable`；
  交给联调同事前仍需由 rt-control 负责人部署并完成一次组合命令实机首跑。

### 验证结论

- Docker 镜像、控制器加载、Mock、迁移差异和仓库策略门禁通过；
- 16-position EtherCAT 与两节点 CANopen 完成实机通信和重复有序启停；
- 14 轴使能/失能以及 28 段逐轴最小低速 FJT 通过；
- 当前负载下 CPU 和内存余量足够，详细数值见
  [Docker 部署与性能验证](docker-deployment-performance-summary.md)。

## 2. 当前可用于联调的接口

| 方向 | 当前接口与类型 | 当前语义 | 联调注意事项 |
| --- | --- | --- | --- |
| motion → rt-control | `/dual_arm_jtc/follow_joint_trajectory`，`control_msgs/action/FollowJointTrajectory` | 执行双臂、Turn、Updown 完整 14 轴轨迹 | 必须包含固定顺序全部 14 轴；仅在 `/rt/enable` 成功后接收 |
| motion → rt-control | `/cmd_vel`，`geometry_msgs/msg/Twist` | 履带线速度/角速度 | 0.5 s 无命令超时；只能有一个有效 publisher |
| 运维 → rt-control | `/rt/enable`、`/rt/disable`、`/rt/reset_fault`，`robot_interfaces/srv/RtEnable` | 14 个 EtherCAT 轴整组生命周期 | 不得当作急停；履带不受 `/rt/enable` 门控 |
| rt-control → motion/状态消费者 | `/joint_states`，`sensor_msgs/msg/JointState` | 14 个 EtherCAT 轴和两条履带的控制状态 | 当前约 50 Hz；接口集合需要公共契约再次确认 |
| rt-control → motion/Nav2 | `/diff_drive_controller/odom`，`nav_msgs/msg/Odometry` | 履带里程计 | 约 50 Hz；`header.frame_id=odom`，新 `child_frame_id=base_footprint` |
| rt-control → 导航/视觉 | `/tf`，`tf2_msgs/msg/TFMessage` | diff-drive 发布 `odom → base_footprint`；RSP 发布活动关节边 | 底盘边约 50 Hz，RSP 上限 50 Hz；同一边只允许一个发布者 |
| rt-control → 导航/视觉 | `/tf_static`，`tf2_msgs/msg/TFMessage` | RSP 发布 `base_footprint → base_link` 和固定本体/传感器边 | transient-local 静态树；`map → odom` 仍由 perception/定位负责 |
| rt-control → 运维/上层 | `/diagnostics`，`diagnostic_msgs/msg/DiagnosticArray` | EtherCAT、CANopen、使能和故障状态 | 当前约 1 Hz、多发布者；按 `status.name` 读取，结构尚未冻结 |

兼容性注意：TF 客户端仍可直接查询组合后的 `odom → base_link`，但它不再是一条直接边。
任何依赖 `/diff_drive_controller/odom.child_frame_id == base_link`、`world` frame 或重复发布本体
TF 的节点都必须迁移；否则会重新制造多父树或产生坐标语义不一致。

`/controller_manager/*`、`/dynamic_joint_states`、PDO/SDO、裸 CAN 帧和驱动内部状态
接口不属于跨域契约，其他域不得直接依赖。gateway、autonomy 和 perception 也不应绕过
motion 直接发送运动命令。

## 3. 联调准入边界

当前允许：

- 同机 ROS 2 发现和消息连通性；
- 接口字段、单位、关节顺序、feedback/result、取消和失败流程联调；
- 在现场批准范围内，由 motion 发送小幅低速完整 14 轴轨迹；
- 履带软件命令链和零命令检查。

当前不能据此宣布：

- 域间接口已经冻结或未来不会调整；
- 软件诊断等同于急停、STO 或安全 PLC；
- 履带方向、比例、停止距离以及 heartbeat/EMCY 机械停车已经完整验收；
- 多轴生产轨迹、GPU/motion 联合满负载或长时间实时性能已经验收；
- EtherCAT 偶发丢包已经修复。

## 4. 剩余主线任务

### A. 域间公共契约重设

当前 ROS 接口足够开始工程联调，但仍要由相关域共同冻结：

- 命令和状态由哪个域生产/消费，禁止的旁路调用；
- 关节集合、顺序、单位、坐标系和时间戳；
- Action 的取消、抢占、超时、重复目标和重启语义；
- Topic 的 QoS、发布频率、单 publisher 所有权和断流语义；
- `map/odom/base_footprint/base_link` 以及相机、雷达 frame 的唯一发布者和模型版本；
- Service 的幂等、错误码、版本兼容和操作者权限；
- ROS middleware、网卡暴露和跨容器发现范围。

当前容器设置了 `ROS_DOMAIN_ID=42`、host network 和 CycloneDDS 配置路径，但没有显式
固定 `RMW_IMPLEMENTATION`。因此正式公共契约还需要记录实扫 RMW，并决定其他域是
共享同一 DDS 配置还是通过受控桥接暴露；不能只凭 XML 文件名推断运行中的 middleware。

### B. 安全诊断公共输出

现有 `/diagnostics` 能报告 EtherCAT link/responders/WC、14 轴 CiA402 状态、CANopen
heartbeat/EMCY 以及 enable manager 状态，适合工程排障。联调后仍需形成上层可稳定
消费的公共安全诊断契约，至少明确：

- ready、warning、fault、latched fault 和 recovering 的含义；
- 时间戳、新鲜度、故障来源、首故障轴、原始码和是否需要人工复位；
- 哪些 WARN 只上报，哪些故障由驱动/rt-control 自动整组停车；
- gateway/autonomy 能看到的摘要与 rt-control 内部细节边界；
- 重启后故障是否保留，以及诊断消息丢失时上层怎样处理。

当前没有独立 `rt_watchdog`，也没有 motion/autonomy heartbeat 自动停机功能；不得让
其他域根据旧设计文档假定这些能力存在。软件诊断始终不能替代实体急停、STO、驱动器
保护或机械限位。

## 5. 剩余支线任务

| 分类 | 当前状态 | 对当前联调的影响 | 后续动作 |
| --- | --- | --- | --- |
| EtherCAT 偶发丢包 | 已定位到 slave 1→2 物理段，当前未影响 action/使能 | 按已批准风险不阻塞低速联调 | 检查线缆/接头/端口，完成 30 分钟 CRC/Lost/WC 零增量空跑 |
| 履带实机验收 | 控制链存在，持续零速和 CAN 状态已验证 | 暂不做无监护的底盘运动 | 验证方向、比例、停止距离、heartbeat/EMCY 和断链停车 |
| 故障注入 | 正常使能/失能/退出已验证 | 不阻塞接口开发，阻塞生产验收 | 分别验证 EtherCAT Fault、CAN EMCY、断链和恢复 |
| 多轴与联合负载 | 逐轴最小 FJT 与资源采样已完成 | 可做小范围 motion 联调 | 验证生产轨迹、多轴同时运动和 GPU/motion 代表负载 jitter |
| Ti5 对象宽度 | 当前实机能写入/读回 250，ESI 与实机宽度仍不一致 | 不阻塞当前固定硬件 | 固件/驱动器变更前取得供应商确认 |
| 启动依赖策略 | 当前主机两只 CANable 均在场 | 不阻塞当前主机 | 后续决定缺少 BMS 适配器时 rt-control 是否允许独立启动 |
| 新 TF 生产部署 | 源码合同、容器 Mock 和三段查询已通过 | 旧锁定镜像尚不含新根链 | 构建新镜像、更新 release/launcher 锁并在目标机只读复核 TF；无需电机运动 |

## 6. 联调期间的版本约束

- rt-control owner 提供并启动锁定镜像，其他域不要自行重建或替换 tag；
- motion 等域暂时按本页“当前接口”接入，并把接口依赖集中封装，便于公共契约调整；
- 联调问题必须同时记录 rt-control SHA、其他域 SHA、ROS domain、实际 RMW 和时间窗口；
- 发现接口缺项先形成公共契约裁决，不直接依赖 controller_manager、PDO 或裸 CAN 绕过。

日常启动见 [rt-control 一键启动](one-command-start.md)。
