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
`config/emergency_stop.yaml` 的 `receiver.emergency_stop_timeout_ms: 0` 只是未填写标记，
不是运行阈值。部署人员在该源配置中填入经过现场确认的正整数毫秒值，重新构建安装
`ctrl_enable_manager` 并重新生成 artifact；不要手改生成的 controllers.yaml。
支持范围为 1..2147483647 ms。开启响应而值为 0 时 configure 返回 ERROR，不会静默关闭保护。
同一部署的响应控制范围共用此值。独立头部当前仅用默认观察模式。

`emergency_stop_topic` 可在配置时指定，默认上述域内话题。
`emergency_stop_topic`、`emergency_stop_response`、`emergency_stop_timeout_ms` 成功配置后不可动态更改。
广播节点的 poll/request/stale 参数另由其部署配置显式给出，不复用或猜测接收端阈值。
生产 launch 尚未自动启动广播节点；部署应由操作台启动已配置的广播节点，再启动控制范围。

接收端检查有效性、状态、stop_requested、producer UUID、样本序号、样本年龄和消息传输年龄。
生产者与消费者须使用一致的 ROS 时间基准；来自未来的时间戳拒绝作为释放证据。
有效 RELEASED 的剩余新鲜期转成接收端单调时钟 deadline，重复同一序号不会延长 deadline。
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
