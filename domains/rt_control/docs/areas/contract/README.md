# contract — 公共契约与适配层

**范围**：公共契约 vendor（`robot_rt_control_interfaces`、`robot_system_interfaces`、
`robot_interfaces_qos`）与 `source-lock.yaml`、域私有接口（`rt_control_interfaces`）、
公共适配器（enable/vacuum/status）、诊断归一化及 vendored RT-Control 契约视图。
**Owner 包/资产**：`src/rt_control/control_api_adapter`、`src/interfaces/*`、
`src/rt_control/rt_diagnostics`、`src/rt_control/rt_control_operator_ui`。

不属于本区：契约 schema 本身的修改（权威源在 `robot_interfaces` 仓库，本区只做
锁定升级）、PLC/BMS 节点实现（→ io-power）。

## 冻结事实（当前有效）

| # | 事实 | 来源 | 状态 |
| --- | --- | --- | --- |
| 01#F1 | /joint_states 契约视图频率=125 Hz（旧本地视图来源） | [contract-20260813-01](records/2026-08-13-joint-states-125hz.md)#F1 | 已由 02#F1 取代来源指针 |
| 02#F1 | vendored 权威 RT-Control 视图固定 /joint_states=125 Hz | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F1 | 有效 |
| 02#F2 | src/interfaces 只保留域内私有接口，公共 schema 从 vendor 构建 | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F2 | 有效 |
| 02#F3 | 跨域 Topic 使用 robot_interfaces_qos 命名 profile | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F3 | 有效 |
| 02#F4 | /cmd_vel_safe QoS 由 Motion 与 RT-Control 原子升级 | [contract-20260814-01](records/2026-08-14-robot-interfaces-vendoring-and-qos.md)#F4 | 有效 |
| 03#F1 | ErrorInfo 是跨域公共载荷，code 唯一采用 uint32 DREE | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F1 | 有效 |
| 03#F2 | 当前 DomainReadiness 为最终公共字段集并遵守一致性规则 | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F2 | 有效 |
| 03#F3 | 全部生产/消费域必须锁定同一接口 SHA 原子升级 | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F3 | 有效 |
| 03#F4 | d8236bd 是 PR #6 验证 SHA；发布仍须回填最终 main SHA | [contract-20260816-01](records/2026-08-16-bq137-shared-error-readiness.md)#F4 | 已由 04#F1 闭环 |
| 04#F1 | RT-Control 固定 robot_interfaces 最终 main SHA f699f45 | [contract-20260817-01](records/2026-08-17-robot-interfaces-final-main-pin.md)#F1 | 已由 06#F1 取代 |
| 04#F2 | f699f45 包含 BQ-137 与 Perception completeness，契约版本仍为 0.7.0 | [contract-20260817-01](records/2026-08-17-robot-interfaces-final-main-pin.md)#F2 | 有效 |
| 04#F3 | 最终 SHA 回填不解除各域同 SHA 原子升级与跨域 smoke 门禁 | [contract-20260817-01](records/2026-08-17-robot-interfaces-final-main-pin.md)#F3 | 有效 |
| 05#F1 | Native 完整构建必须安装并验证 QoS 五 profile 运行闭包 | [contract-20260817-02](records/2026-08-17-native-qos-runtime-closure.md)#F1 | 有效 |
| 05#F2 | Native 入口在硬件访问前 fail-closed 检查公共适配器依赖闭包 | [contract-20260817-02](records/2026-08-17-native-qos-runtime-closure.md)#F2 | 有效 |
| 05#F3 | Native READY 需要 live controller-manager、控制器状态与 EtherCAT OP | [contract-20260817-02](records/2026-08-17-native-qos-runtime-closure.md)#F3 | 有效 |
| 06#F1 | RT-Control 权威 robot_interfaces SHA 固定为 92d6ff2 | [contract-20260827-01](records/2026-08-27-robot-interfaces-main-pin.md)#F1 | 有效 |
| 06#F2 | f699f45→92d6ff2 未改变 RT 三个公共包，只改变 Motion M-08 Action | [contract-20260827-01](records/2026-08-27-robot-interfaces-main-pin.md)#F2 | 有效 |
| 07#F1 | ROS Domain 由部署显式配置为十进制 `0..232`，默认 `0` | [contract-20260828-01](records/2026-08-28-configurable-ros-domain.md)#F1 | 有效；取代 BQ-128 固定 Domain 0 |
| 07#F2 | 同一机器实例的五域必须使用同一 Domain，Domain 不是安全隔离 | [contract-20260828-01](records/2026-08-28-configurable-ros-domain.md)#F2 | 有效 |
| 07#F3 | RMW 仍固定 `rmw_fastrtps_cpp`，传输仍为 Fast DDS 默认 UDP+SHM | [contract-20260828-01](records/2026-08-28-configurable-ros-domain.md)#F3 | 有效 |
| 08#F1 | 共享模型新增固定边 `base_link → lidar_main`，位姿为 `xyz=(0.382364228640, 0.133500000000, 0.121820508080)`、`rpy=(0, 0.523598775598, 0)` | [contract-20260902-01](records/2026-09-02-lidar-main-tf.md)#F1 | 有效；消费者联合验证待完成 |
| 08#F2 | `lidar_main.STL` 使用 `base_link` 坐标和毫米单位，消费时必须使用 `0.001` 缩放 | [contract-20260902-01](records/2026-09-02-lidar-main-tf.md)#F2 | 有效 |
| 08#F3 | `lidar_main` 变更不新增公共接口、不改变 odom TF 所有权或实时控制参数 | [contract-20260902-01](records/2026-09-02-lidar-main-tf.md)#F3 | 有效 |
| 09#F1 | 操作员 UI 只消费现有标准/公共状态面，不新增 enable-manager 私有状态 topic | [contract-20260903-01](records/2026-09-03-operator-console.md)#F1 | 有效 |
| 09#F2 | 厂商错误目录必须携带手册版本、页码和源文件 SHA；未知码禁止猜测或自动复位 | [contract-20260903-01](records/2026-09-03-operator-console.md)#F2 | 有效；0x603F 自动采集受 BQ-142 阻塞 |
| 09#F3 | 操作员命令单次发起且无自动重试；软件停机不代表硬急停/STO | [contract-20260903-01](records/2026-09-03-operator-console.md)#F3 | 有效 |

## 记录索引（倒序）

- 2026-09-03 [Qt 操作员故障控制台首版](records/2026-09-03-operator-console.md) — feature，PARTIAL（T1；0x603F 与实机验收待完成）
- 2026-09-02 [新增 base_link 下 lidar_main 固定 TF 与毫米制 STL](records/2026-09-02-lidar-main-tf.md) — feature，PARTIAL（T1；生产切换与消费者联合验证待完成）
- 2026-08-28 [ROS Domain 由部署显式配置](records/2026-08-28-configurable-ros-domain.md) — decision，PASS（T1；native/Compose/Mock）
- 2026-08-27 [robot_interfaces 权威 SHA 同步至 92d6ff2](records/2026-08-27-robot-interfaces-main-pin.md) — corrective，PASS（T1；RT schema 不变）
- 2026-08-17 [Native QoS 运行依赖闭包与 fail-closed 启动门禁](records/2026-08-17-native-qos-runtime-closure.md) — corrective，PASS（T3；最终 stopped/Idle/PREOP）
- 2026-08-17 [robot_interfaces 回填最终 main SHA](records/2026-08-17-robot-interfaces-final-main-pin.md) — corrective，PASS（T0；发布仍待镜像与全域同 SHA smoke）
- 2026-08-16 [BQ-137 跨域错误与 readiness 语义裁决落地](records/2026-08-16-bq137-shared-error-readiness.md) — decision，PASS（T1；发布待 PR #6 最终 SHA 与全域迁移）
- 2026-08-14 [公共接口改为固定 SHA vendoring 并采用命名 QoS](records/2026-08-14-robot-interfaces-vendoring-and-qos.md) — decision，PASS（T1；当时的 BQ-137 语义阻塞已由 03#F1～F4 闭环）
- 2026-08-13 [/joint_states 频率裁决为实测 125 Hz（BQ-135）](records/2026-08-13-joint-states-125hz.md) — decision，PASS（T1）

## 历史锚点（2026-08-13 前，未迁移）

- PROGRESS.md 历史段：T-IF-RT-001..005（契约 0.5.0/0.6.0、公共适配器、
  ErrorInfo 采用）
- 已删除的本地跨域接口视图（历史 main：契约 0.6.0 @ `e19d1450`）
- 相关 BQ（不完全）：见 BLOCKED-questions.md 中 T-IF-RT 关联裁决
