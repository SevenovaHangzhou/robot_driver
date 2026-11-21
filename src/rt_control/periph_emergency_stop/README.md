# 头部与双臂软急停输入广播

独立外设节点 `emergency_stop_node` 只通过 `infra_modbus` 发送 FC02 读请求，向域内
`/plc/emergency_stop/state` 发布 `rt_control_interfaces/msg/EmergencyStopState`。
不写 DO，不调用失能、复位或使能服务，不自动启动控制栈，不修改宿主路由。

## 状态契约

- 有效输入等于 `pressed_level`：`PRESSED`；其余有效输入：`RELEASED`。
- 首次读取前、请求失败、协议异常或样本年龄达到 `stale_timeout_ms`：`UNKNOWN`。
- `stop_requested = state != RELEASED`。UNKNOWN 不能解释为按钮释放。
- 发布 QoS：reliable、transient_local、depth=1。每个成功/失败读取后发布，另以
  `poll_period_ms` 周期广播；TCP 等待与广播使用独立非实时 callback group / 两个 executor 线程。
- `header.stamp` 是发布时间，`sample_stamp` 是最后成功读取完成时间；样本年龄使用 steady clock。
  `sample_sequence` 只在成功读取时递增，心跳不刷新样本。每次进程启动生成新的 producer UUID。
- 输入恢复会如实广播 RELEASED，但不代表复位许可。此节点只报告输入，不持有停机锁存状态。

消费者必须锁存停机并管理人工复位；收到 PRESSED、UNKNOWN 或本节点失联都不能继续许可运动。
消费者须用本地单调时钟设置消息到达超时，并检查样本年龄，不能只看 `header.stamp` 或
transient_local 缓存的最后一条 RELEASED。进程被杀、DDS 失联时不能依赖最后一次退出广播。
任何数据恢复/按钮释放均不得触发自动使能或恢复旧任务。

这是普通网络 IO 的软件观测，不是安全继电器/STO 或硬件急停执行确认。
当前包由已配置实机范围的 scope launch 启动；接收与头臂响应见 [ctrl_enable_manager](../ctrl_enable_manager/README.md)。
仅运行本节点不会使机器人停机；头部/双臂消费者须配置并运行。
跨域消费需先在独立 robot_interfaces 冻结公共契约，不直接把本域消息作为公共接口。

## 配置与启动

必须显式配置 `host`、`port`、`unit_id`、`input_address`、`pressed_level`、
`poll_period_ms`、`request_timeout_ms`、`stale_timeout_ms`；缺失/越界拒绝启动。
`interface` 可选，空值遵循宿主路由；有值时复用 infra_modbus 的网卡绑定，绑定失败不回退。
所有参数仅在启动读取，修改后需重启节点。时间参数是部署输入，不提供猜测性安全默认值。
轮询受请求耗时限制，实际最坏检测时间还包含设备响应、网络、调度和消费者处理；本次手工
采样约 1 s 间隔不能作为急停响应时间保证。

台套配置位于 `hwcfg_machines/units/<unit>/deployment.yaml` 的 `emergency_stop` 段，
包含接收超时与输入节点参数；机型消费者选择响应，组合器生成节点配置及启动清单。
实机配置有 TBD 时拒绝生成 artifact；mock 不启动物理输入节点。通用包不内置 IP 或极性。
用户于 2026-11-21 将本次交付限定为头部和双臂的软急停，沿用普通 IO → Modbus → 软件停机链。
四个时间参数由软件响应预算及现场测量确认，未知值保留 TBD。硬件安全回路建设与认证另行处理；
本节点不提供安全继电器或 STO 的替代能力，也不能套用其 15/20 ms 器件反应规格。
部署者确认所需参数后，可单独诊断节点（以下变量必须先赋值，实机操作由操作台执行）：

```bash
ros2 run periph_emergency_stop emergency_stop_node --ros-args \
  -p host:="$ESTOP_HOST" -p port:="$ESTOP_PORT" -p unit_id:="$ESTOP_UNIT" \
  -p interface:="$ESTOP_INTERFACE" -p input_address:="$ESTOP_ADDRESS" \
  -p pressed_level:="$ESTOP_PRESSED_LEVEL" \
  -p poll_period_ms:="$ESTOP_POLL_MS" -p request_timeout_ms:="$ESTOP_REQUEST_MS" \
  -p stale_timeout_ms:="$ESTOP_STALE_MS"
ros2 topic echo /plc/emergency_stop/state rt_control_interfaces/msg/EmergencyStopState \
  --qos-reliability reliable --qos-durability transient_local
```

## 离线验证

构建 `infra_modbus`、`rt_control_interfaces`、`periph_emergency_stop` 后，运行：

```bash
colcon test --packages-select infra_modbus periph_emergency_stop --event-handlers console_direct+
colcon test-result --verbose
python3 tools/arch_check.py
```

节点测试使用真正的 ROS 进程、DDS 消息和本地 TCP 测试模块，覆盖未知初态、释放/按下、
晚加入订阅者、TCP 等待中的过期广播、超时/异常/错误数据/断线、恢复重连、只读报文和 SIGTERM。
测试时间参数仅供本地测试，不是现场安全阈值。
