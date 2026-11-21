# 急停接收与双臂停机

`ctrl_enable_manager` 统一订阅 `/plc/emergency_stop/state`（域内 `EmergencyStopState`，
reliable / transient_local / depth 1），在自己的非实时 callback group 接收；实时状态机只读原子状态。
控制器与外设包之间不直接依赖。

## 覆盖范围

| 控制范围 | 接收与诊断 | 本次停机响应 |
| --- | --- | --- |
| 左臂、右臂、双臂，包括 stationary_enable | 有 | 启用 |
| 双臂与头部组合 | 有 | 停整个现有使能组，包括头部伴随控制器 |
| 双臂重力前馈组合 | 有 | 停止当前轨迹控制器及重力前馈伴随控制器 |
| 底盘、独立头部 | 有 | 仅接收，尚未启用动作 |
| 升降、主动悬挂 | 公共管理器具备接收能力 | 机型中尚无可运行 scope，未形成独立接收实例 |

机型 enable policy 的 `emergency_stop_response: true` 选择响应。
未选中的控制范围默认仅观察，不能据此宣称已有急停停机功能。

## 部署参数

用户于 2026-11-19 决定：接收端失联/样本过期阈值在部署时配置。
接收阈值唯一来自 `hwcfg_machines/units/<unit>/deployment.yaml` 的
`emergency_stop.receiver_timeout_ms`；输入节点的映射与 poll/request/stale 时间也在同一台套的
`emergency_stop.input` 中。支持正整数毫秒值 1..2147483647；未知值显式保留 `TBD`。
开启响应的实机范围在四个时间或映射未填写时拒绝生成 artifact。修改台套源配置后须重新安装
`hwcfg_machines` 并重新生成 artifact，不手改生成的 controllers.yaml。
mock 不连接物理输入节点；draft mock 可离线组合，但缺少接收阈值时控制器仍拒绝 configure，
运行测试必须显式提供测试台套的时间值，不能把测试数值作为现场默认值。

`emergency_stop_topic` 可在配置时指定，默认上述域内话题。
`emergency_stop_topic`、`emergency_stop_response`、`emergency_stop_timeout_ms` 成功配置后不可动态更改。
广播节点的 poll/request/stale 参数另由其部署配置显式给出，不复用或猜测接收端阈值。
实机 artifact 的 input_peripherals 由 scope launch 启动；节点与 launch 共生命周期，不自动重启。
节点故障或退出由本接收端的原有新鲜度和锁存停机路径处理，不能通过立即杀死控制器跳过停机收敛。

接收端检查有效性、状态、stop_requested、producer UUID、样本序号、样本年龄和消息传输年龄。
生产者与消费者须使用一致的 ROS 时间基准；来自未来的时间戳拒绝作为释放证据。
有效 RELEASED 的剩余新鲜期转成接收端单调时钟 deadline，重复同一序号不会延长 deadline。
PRESSED 同样推进序号水位；旧序号或与按下相同序号的 RELEASED 不能恢复释放证据。
同周期按下与硬件离线分别锁存，人工复位按钮及同一控制器实例重新 activate 都不能清除
离线造成的 restart_required；恢复仍由操作台按完整 RT-Control 重启流程执行。
新实例、倒退序号、未知/无效数据、按下、样本过期、进程退出或网络失联均不能自动恢复许可。
QoS 缓存的旧 RELEASED 也必须通过年龄检查。

## 响应与恢复

- 初次没有新鲜释放样本时保持禁止使能。收到停机事件即锁存；两个 RT 周期之间已接收的按下再释放也不会丢掉锁存。
- 运行或使能/复位途中触发：RT 进入现有 EmergencyQuickStop；OperationEnabled/QuickStopActive
  轴写控制字 `0x0002`，其他轴沿原有状态机向失能收敛，然后进入 EmergencyDisable。
  沿用已存在的停止阶段超时与驱动反馈规则，不新增减速或运动阈值。
- 非实时 worker 停止已有 managed/companion controller 集合；等待在途 controller switch 后再执行停用。
  包括当前 JTC/rolling、组合头部和被选中的重力前馈。停用失败会要求控制栈重启。
- 停机锁存期间拒绝使能和模式切换。按钮释放只更新观测，不恢复旧轨迹，也不自动使能。
- 操作台确认按钮释放、收到新鲜输入、停止过程结束并收到控制器停用确认后，显式执行现有
  `/rt/reset_fault`，成功清除锁存后才能单独执行 `/rt/enable`。输入仍不可信时复位会被拒绝。
  硬件/总线故障引起的 restart_required 继续走原有完整重启要求。

诊断增加 `emergency_stop_reported_state`（上次广播值）、`emergency_stop_effective_state`
（本地新鲜度判断）、`emergency_stop_response`、`emergency_stop_latched`。状态数值为 0 UNKNOWN、
1 RELEASED、2 PRESSED。观察模式未配置超时时 effective 保持 UNKNOWN，reported 只表示最后收到的值，
不能用于运动许可。服务拒绝 stage 为 `emergency_stop`。

## 验证边界

T0 包构建、状态机/握手/模式切换测试及真实本机 DDS 接收测试，不连接硬件。
测试时间常量不属于部署配置。现场 T3/T4、驱动器实际停止时间、物理制动与断动力效果尚未验证，
由操作台按分级流程执行。本功能是网络软件停机路径，不替代安全继电器、STO 和硬件保护。
