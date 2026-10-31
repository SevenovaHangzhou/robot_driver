# 操作网页合并与诊断节点设计稿

状态：**已定稿**（用户 2026-10-31 评审第 7 节全部事项）。对应迁移待办 §5a 的 `rt_control_operator_web + rt_control_arm_console → integ_operator_web` 与 `rt_diagnostics → integ_diagnostics` 两行。实现按 §4.5、§5.7 的顺序逐个 PR 进行。

## 0. 已定事项（用户 2026-10-31）

| 事项 | 结论 |
| --- | --- |
| 网页形态 | 一个包、一个进程、一个端口，多个页面 |
| 传输 | aiohttp + WebSocket（连接断开即视为操作员离开，用于"断线即停"） |
| 使用范围 | 只给台架与调试，不上量产 |
| 操作权 | 同一时间一人操作，其他会话只读 |
| 诊断定位 | 启动时自检**一次**，输出故障位置 + 公司统一 DREE 码 |
| 诊断与网页 | 诊断逻辑在独立节点；网页只是读者 |
| 故障码 | 按厂家/型号建表，与设备事实放一起（#153）；`ErrorInfo.code` 只放 DREE 事件类，厂家原始码放 `detail` |
| DREE 新码 | `RT_DEVICE_OFFLINE`1001、`RT_DEVICE_IDENTITY_MISMATCH`1010、`RT_DEVICE_FAULT`1028、`RT_STO_ACTIVE`1070、`RT_DEVICE_FAULT_RESETTABLE`1128（可复位算可自动恢复）、`RT_DEVICE_FEEDBACK_STALE`1141（robot_interfaces#12 待评审） |

## 1. 现状

### 1.1 两个网页

| | `rt_control_arm_console`（手臂） | `rt_control_operator_web`（底盘） |
| --- | --- | --- |
| 功能 | 单臂/双臂关节 S 曲线运动、位姿与序列、整车模型 3D 视图、使能/失能、故障复位、临时真空开关与压力显示 | 使能/失能、摇杆/键盘遥控、软件停止、故障复位、转向参数现场调参 |
| 服务端 | 标准库 `ThreadingHTTPServer`：JSON POST 命令 + SSE 状态流 | aiohttp + WebSocket，状态 20 Hz 推送 |
| 鉴权 | 令牌登录换会话 cookie、同源检查、审计日志 `audit.jsonl` | 连接后首条消息带令牌；调参等操作再次输入令牌确认 |
| 操作权 | 显式取得/释放控制权 | 单终端控制权；遥控租约，心跳 50 ms，命令超时 200 ms 即停 |
| ROS 接口 | `/rt/capabilities`、`/control/set_enabled`、`/rt/reset_fault`、FollowJointTrajectory、`/joint_states`、`/diagnostics`、控制器参数服务 | `/rt/capabilities`、`/control/set_enabled`、`/rt/reset_fault`、`list_hardware_components`、控制器参数服务、`/diagnostics`、速度话题 |
| 规模 | Python 约 2100 行，前端约 1200 行（另有离线 three.js） | Python 约 1600 行，前端约 950 行 |

共同部分：令牌与会话、同源检查、`/rt/capabilities` 能力发现、使能与故障复位、`/diagnostics` 读取、控制权。两套实现各写一遍。

### 1.2 诊断

- 运行栈启动前，`rt_control_scope_runtime.launch.py` 调用各协议包的 `preflight(expected_topology)`：
  - EtherCAT：经 `ethercat` 命令行只读检查主站、从站数量、身份（厂家/产品/版本）、PREOP、分支接线、各轴 0x6041/0x603F 是否为允许的启动码；
  - CANopen：接口 UP 且 error-active、USB 序列号、总线包是否与源一致；
  - 达妙头部：没有预检（`preflight` 为空）。

  预检**遇到第一个问题就抛异常**，运行栈不启动，但没有逐设备的报告。
- 运行中：`enable_manager` 在 `/diagnostics` 发一行汇总（阶段、失败批次/关节/状态字）；底盘、力传感器广播器各发自己的行。
- 公共接口 R-OUT-09 `/rt_control/readiness`（`DomainReadiness`，消费方 Autonomy）在注册表中由 RT 生产，但 #117 删除草稿适配器后**目前没有发布者**。
- 原 `rt_diagnostics` 已删除（#118），只解析二代履带。

### 1.3 可用的设备数据

| 设备 | 周期数据中的故障信息 | 故障码表 |
| --- | --- | --- |
| 零差 eRob 手臂驱动 | `status_word`、`error_code`（0x603F） | `devices/faults/zeroerr_erob.yaml`（草稿） |
| Kinco 舵轮驱动 | `status_word`；**插件未导出 0x2601/0x2602 错误字** | `devices/faults/kinco_fd.yaml`（草稿），需插件导出后才能用 |
| 达妙头部电机 | `fault_code`（反馈帧故障半字节）、温度 | 未建（手册摘录 V3-DOC-2026-012 已有） |
| BRT 舵角编码器（CANopen） | `ready`、`feedback_age_ms`；EMCY 未导出 | 只能判断在线/超时 |
| EtherCAT 主站与从站 | `link_up`、`slaves_responding`、`wc_error_count`、各从站 `al_state`、`process_data_age_ms` | 不需要 |

## 2. 目标与边界

- 网页：台架/调试人员在一个页面完成使能、手臂与底盘操作、查看自检与故障；不新增运动能力。
- 诊断：启动时一次性给出"哪个设备、什么问题、DREE 码、厂家原始码与说明"；运行中设备自己报出故障时，同样给出位置与码。
- 不做：替代急停/STO/安全链；自动使能或自动复位；参与或改变 `enable_manager` 的使能准入（准入只由 `enable_manager` 决定）；访问未在制品中声明的设备。

## 3. 包与进程

```text
integ_operator_web   一个 aiohttp 进程：网页 + 一个 rclpy 节点（只调用公共接口）
integ_diagnostics    一个 rclpy 节点：启动自检 + 被动故障上报，发布 /rt_control/readiness
integ_composer       在制品中输出 diagnostics 设备清单（唯一来源）
hw_*                 preflight 改为返回逐设备结果；故障码表读取方式在表中声明
```

两个包都是非实时 Python 进程，绑定非实时 CPU（沿用底盘页 `launcher.py` 的 CPU 绑定）。诊断节点作为运行栈的一部分由 `rt_control_scope_runtime.launch.py` 启动；网页由操作员单独启动，可以在运行栈之前或之后启动。

## 4. `integ_operator_web`

### 4.1 进程结构

- aiohttp 事件循环处理 HTTP 与 WebSocket；rclpy 执行器在独立线程运行；两者之间只通过线程安全队列与带锁的状态快照交换数据。
- 页面由 `/rt/capabilities` 决定是否可用：有 `arm_joint_trajectory` 显示手臂页，有 `chassis_velocity` 显示底盘页，诊断页始终可用（无发布者时显示"诊断节点未运行"）。

### 4.2 鉴权与操作权

- 令牌文件权限必须为 0600。`POST /api/login` 校验令牌，返回会话 cookie（`HttpOnly`、`SameSite=Strict`），失败延迟 1 s。
- WebSocket 握手时校验会话 cookie 与 `Origin`；不在消息里传令牌。
- 操作权：任一会话可显式"取得操作权"；同一时间只有一个持有者，其他会话只收状态、命令一律拒绝。持有者可释放；断开连接即释放。
- 断线即停：持有者 WebSocket 断开或心跳超时（沿用底盘页：心跳 50 ms，超时 200 ms）时，停止当前手臂运动（平滑停车）与底盘遥控（速度归零）；不失能、不复位。
- 再次确认：转向参数修改、故障复位、恢复默认需要持有者重新输入令牌（沿用底盘页做法）。
- 审计：所有命令（含拒绝）写一份 `audit.jsonl`，格式沿用手臂控制台（时间、ROS_DOMAIN_ID、操作、参数、结果、会话摘要、客户端地址；不写令牌与会话值）。

### 4.3 WebSocket 消息

- 客户端 → 服务端：`{"type": ..., "id": <请求号>, ...}`；服务端对每个请求回 `{"type": "result", "id": ..., "ok": ..., "code": ..., "message": ...}`。
- 服务端 → 客户端：`status`（20 Hz，按页面分段：使能、手臂、底盘、诊断摘要）、`event`（运动完成、序列进度、操作权变化、自检完成）。
- 消息大小上限、未知类型拒绝、数值必须有限，沿用底盘页 `protocol.py` 的入口校验（只在入口校验一次）。

### 4.4 页面与功能来源

| 页面 | 功能 | 来源 |
| --- | --- | --- |
| 公共栏 | 登录、操作权、使能/失能、故障复位、使能状态与失败位置 | 两者合并 |
| 手臂 | 关节目标、S 曲线执行与停止、位姿与序列（含逐段速度与停留）、整车模型视图、真空开关与压力 | 手臂控制台，SSE 改为 WebSocket |
| 底盘 | 摇杆/键盘遥控、档位、软件停止、转向参数调参 | 底盘页原样迁入 |
| 诊断 | 自检结果表（位置、DREE 码与名称、厂家码与说明、建议）、运行中故障、`/diagnostics` 原始行 | 新增，只读 `/rt_control/readiness` 与 `/diagnostics` |

### 4.5 迁移顺序（每步一个 PR）

1. 建 `integ_operator_web`：公共核心（鉴权、会话、操作权、WebSocket、ROS 桥、审计）+ 底盘页（底盘页已是 aiohttp/WebSocket，迁入改动最小）。
2. 迁入手臂页（SSE/POST 改为 WebSocket 消息；运动规则与测试原样保留）。
3. 加诊断页（依赖 §5 的 `/rt_control/readiness`）。
4. 删除两个旧包，更新台架启动脚本与文档。第 2、4 步改变访问地址与端口，按行为变更 PR 处理。

## 5. `integ_diagnostics`

### 5.1 设备清单（组合器输出）

组合器在制品中增加 `diagnostics` 段，诊断节点只读它，不另维护设备清单。每台设备一项：

```yaml
- source: arms/left_joint3/drive        # <模块>/<关节或设备名>/<角色>
  transport: {owner: hw_ethercat, master: 0, ring_position: 3}
  identity: {vendor_id: 0x5A65726F, product_code: 0x29252, revision_number: 1}
  state: {status_word: left_joint3/status_word, error_code: left_joint3/error_code, al_state: ethercat_slave_3/al_state}
  allowed_startup_codes: ['0xA000']     # 来自现有 allowed_errors
  fault_table: <devices/faults/zeroerr_erob.yaml 的内容>
```

总线级条目：`ethercat/master0`（`link_up`、`slaves_responding`、`wc_error_count`、`process_data_age_ms`）、`can/<接口名>`。来源字段全部取自现有模块配置、设备事实与机器人描述。

### 5.2 启动自检（一次）

| 阶段 | 时机 | 检查 | 失败时 |
| --- | --- | --- | --- |
| A 总线前 | 现有 preflight 位置（ros2_control 启动前） | 主站/接口在线、从站数与身份、AL 状态、接线、各轴 0x603F（头部 CAN 现无预检，第一版只在阶段 B 检查） | **保持现状：运行栈不启动**；改为先跑完全部检查，按设备输出结果文件后再退出 |
| B 激活后 | 硬件激活完成、使能前 | 各从站 OP、`link_up`、`wc_error_count` 不增长、`process_data_age_ms` 在时限内、各轴 `error_code`/头部 `fault_code` 无故障、编码器 `ready` 且未超时 | `ready=false`，结果写入 `/rt_control/readiness` |

阶段 A 的结果文件写在本次运行目录，诊断节点启动后读入并一并发布；运行栈没起来时，网页直接读该文件显示原因（§7 第 4 项）。

码的选择：从站缺失、未到 OP、接口未起、编码器无数据 → `RT_DEVICE_OFFLINE`；身份不符 → `RT_DEVICE_IDENTITY_MISMATCH`；设备报故障码 → 查该设备的故障码表，未登记的码 → `RT_DEVICE_FAULT`，`detail` 写"未登记"；反馈超时 → `RT_DEVICE_FEEDBACK_STALE`。允许的启动码（如零差 `0xA000`）照样列出，但按 `RT_DEVICE_FAULT_RESETTABLE` 标为可复位，不作为阻断项。

### 5.3 运行中（被动上报）

不重复执行自检。只在设备自己通过周期数据报出问题时上报：从站 AL 状态离开 OP、驱动状态字故障位置位（附 0x603F 查表结果）、头部 `fault_code` 非零、反馈超时。上报即刻发布，故障清除后移除该条目。

### 5.4 输出

- R-OUT-09 `/rt_control/readiness`（`DomainReadiness`，`Q_LATCHED`，变化立即发、稳定 1 Hz）。`readiness_name` 为本次作用域名；`errors[]` 每项一个 `ErrorInfo`：

  | 字段 | 内容 | 例 |
  | --- | --- | --- |
  | `code` | DREE | `1028` |
  | `severity` | FAULT / WARN | FAULT |
  | `source` | 设备位置 | `arms/left_joint3/drive` |
  | `detail` | 厂家原始码与说明、出处 | `0x603F=0x2214 电机电流过流（ZeroErr §7.2.2 p120）` |
  | `retryable` | 由 DREE 的 R 位决定 | false |

  `blockers` 用稳定标记：`self_check_failed`、`device_fault`、`bus_offline`。
- 本次运行目录写 `self_check.json`（阶段 A 与 B 的全部结果），供审计与 Runtime-LOG 归档。
- `/diagnostics`（R-OUT-10）不变，各控制器照旧发布。

### 5.5 实现语言与位置

Python rclpy 节点，订阅内部状态广播 `/rt_internal_state_broadcaster/dynamic_joint_states`（实现时确认其包含传感器类状态接口），不进入实时进程，不声明控制接口。故障码查表与"读取方式"（整码 / 位域 / 达妙半字节）在诊断节点内实现，含义全部来自表。

### 5.6 缺口（需单独 PR）

1. Kinco 插件导出 0x2601/0x2602 错误字：改 `hw_ethercat`（实时路径，只开 PR，台架验证后合并）。
2. 达妙头部故障码表：从 V3-DOC-2026-012 摘录后登记。
3. BRT 编码器 EMCY 未导出：第一版只判断在线/超时。
4. 外设（BMS、IMU、PLC、LED/超声波）是独立进程，第一版不纳入；后续按同一 `source` 规则接入。

### 5.7 实施顺序（每步一个 PR）

1. 组合器输出 `diagnostics` 设备清单（制品新增一段，组合快照会变化，单独说明）。
2. 各协议包 preflight 改为逐设备结果 + 结果文件（失败仍阻止启动）。
3. `integ_diagnostics` 节点与 `/rt_control/readiness` 发布（行为变更：新增发布者，恢复 R-OUT-09）。
4. §5.6 缺口逐项补齐。
5. 网页诊断页（§4.5 第 3 步）。

## 6. 验证

- 网页：保留两个旧包的全部服务端测试（运动规则、序列、遥控租约、调参、协议校验），迁移后在新包中通过；新增操作权与断线即停测试（真实 aiohttp 服务器 + 测试客户端，不 mock 业务逻辑）。
- 诊断：阶段 A 用现有 preflight 测试的命令输出样本；阶段 B 用 mock 硬件运行栈注入状态值；查表用 #153 的两张表。
- 台架：用户在场时，按现有静止使能流程启动 `bench_dual_arm` 与 `head_only`，确认自检全部通过；断开一个从站电源或拔网线前须单独授权。

## 7. 已定事项（用户 2026-10-31）

| # | 问题 | 结论 |
| --- | --- | --- |
| 1 | 自检结果是否参与使能准入 | **不参与**。`enable_manager` 是唯一准入方；诊断只报告 |
| 2 | 运行中设备自己报出的故障是否上报（不重复自检） | **上报**（§5.3） |
| 3 | 诊断节点是否作为 R-OUT-09 `/rt_control/readiness` 的生产者 | **是**，恢复该公共接口，Autonomy 可消费 |
| 4 | 阶段 A 失败、运行栈没起来时，网页如何显示原因 | 网页读本次运行目录的 `self_check.json` |
| 5 | 端口 | 统一 18120；台架启动脚本同步 |
| 6 | 手臂页临时真空开关（调用现场 `~/.local/bin/vacuum` 脚本） | 第 2 步原样迁入并保留"临时"标注；改用 `periph_plc_io` 公共服务另开 PR |
