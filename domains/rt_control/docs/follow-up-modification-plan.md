# rt-control 后续修改计划

状态：**两项修改均已进入源码，尚未构建发布镜像或完成目标机验收**

记录日期：2026-07-29

本文记录后续修改范围、约束、实现状态和验收方向。当前目标工控机仍运行修改前的锁定镜像；本文两项修改均尚未部署到生产 release，恢复入口也未执行掉电—复电实机测试。

## 1. 主接触器急停断电后的交互式恢复入口

### 1.1 场景与目标

现场硬件无法向工控机提供独立、可信的急停状态；急停通过切断控制主接触器使电机和抱闸侧掉电，而工控机及旧 rt-control 容器可能继续运行。

新增一个显式、人工确认的恢复入口，建议命令形态为：

```bash
./tools/rt_control_ipc.sh recover-power-loss
```

该入口不是急停检测器，不监听复电后自动恢复，也不改变普通一键启动的“不得自动复位 Fault”策略。

### 1.2 已批准的恢复顺序

1. 最佳努力调用 `/rt/disable`；电机已掉电、通信不可用时允许该步骤无法确认 `0x0040`，但不得伪报有序失能成功。
2. 停止旧容器并要求确认容器已退出、EtherCAT 主站 `Idle / Active: no`，销毁旧 JTC、PDO 缓存和使能会话。
3. 暂停并要求操作者明确确认主接触器已经恢复、机械区域安全且允许执行器上电。
4. 启动全新的锁定控制栈，但不立即调用 `/rt/enable`。
5. 核对冻结拓扑和身份，等待 14 个配置轴进入 OP、Working Counter 完整、控制服务就绪。
6. 调用一次全组 `/rt/reset_fault`；无 Fault 时接受无副作用的 `already_clear`。
7. 按已冻结 BQ-115 检查非激磁终态：其余 10 轴严格要求 `0x0040`；
   `right_joint2/right_joint3/left_joint2/left_joint3` 四个 Ti5 允许 `0x0040` 或已实机确认的
   `Ready To Switch On (0x0021)`。原计划“14 轴一律 `0x0040`”与 BQ-115 冲突，现按不得推翻的冻结裁决修正。
8. 调用一次 `/rt/enable`，要求服务返回成功、14 轴均为 `0x0027`、JTC 为 ACTIVE。
9. 任一步失败都停止恢复，不循环复位、不重复使能、不自动忽略失败轴。

### 1.3 实现约束

- 不新增 ROS 包或常驻恢复节点；优先在现有受控操作入口中实现独立子命令。
- 普通启动路径不得自动调用 `/rt/reset_fault`；恢复路径必须由专用命令和人工确认显式进入。
- 不增加 `NET_ADMIN`、`privileged`、Docker socket 或其他容器权限。
- 不把软件流程描述成急停、STO 或硬件安全功能。
- 驱动器返回未知 Fault 时必须报告首个失败轴、状态字和阶段，不得假定都是主接触器断电所致。
- 实机掉电—复电验收需要单独的 L3 授权、现场急停条件和完整日志记录。

### 1.4 收益与代价

收益：操作者只需一个受控入口和一次复电确认；每次恢复均使用新控制会话，避免旧轨迹、旧预装载或缓存 PDO 跨越掉电—复电。

代价：流程仍依赖人工确认，且完整控制栈重启会增加约一分钟恢复时间；已掉电期间无法证明驱动器收到过软件失能控制字。

### 1.5 当前源码实现状态

- 在现有 `tools/rt_control_ipc.sh` 中增加独立 `recover-power-loss` 子命令；普通 `start` 仍不调用 reset。
- 使用现有 `/dynamic_joint_states` 的 `status_word` 做逐轴掩码检查，不增加 ROS 包或常驻节点。
- 对旧会话的 disable 只做最佳努力并如实记录；新会话失败会执行失能/停止清理，不做 reset/enable 循环。
- 本阶段只完成离线合同、Shell 和 Mock/镜像验证；真实主接触器掉电—复电仍需单独现场授权。

## 2. 原始轮速里程计统一为 `/wheel/odom`，并移交 `odom` TF 所有权

### 2.1 目标

为避免 rt-control 原始轮速里程计与导航域最终里程计命名及所有权冲突，将 rt-control 对外轮速里程计 topic 统一为：

```text
/wheel/odom
```

`/wheel/odom` 只是 `nav_msgs/msg/Odometry` topic 名称，不是 TF frame 名称。rt-control 同时停止发布动态
`odom -> base_footprint` TF；导航域成为最终 `/odom` 里程计数据和 `odom -> base_footprint` TF 的唯一发布者。

本项已修改当前源码中的 Launch、控制器配置和活动接口文档，但尚未构建/部署新的锁定镜像，导航域也尚未联合启动验证。

### 2.2 当前实机证据与不一致

2026-07-29 对当前锁定镜像运行时执行 `ros2 topic list` 和 `ros2 topic info`：

- 存在 `/diff_drive_controller/odom`，类型为 `nav_msgs/msg/Odometry`，唯一发布者为 `diff_drive_controller`；
- `/odom` 不存在。

因此“rt-control 当前发布 `/odom` topic”与实机运行事实不一致。不过当前 diff-drive 配置包含
`odom_frame_id: odom` 和 `enable_odom_tf: true`，所以 rt-control 确实占有并发布动态
`odom -> base_footprint` TF；这正是本次需要从 rt-control 移除的 TF 所有权。

### 2.3 冻结前提与影响范围

- rt-control 将 diff-drive 的 `nav_msgs/msg/Odometry` 输出从 `/diff_drive_controller/odom` 显式 remap 为唯一
  `/wheel/odom`；不保留 `/odom` 或 `/diff_drive_controller/odom` 别名。
- rt-control 将 diff-drive 的 `enable_odom_tf` 设置为 `false`，不再发布任何以 `odom` 为父 frame 的 TF；不得把
  `wheel/odom` 或 `/wheel/odom` 写成 TF frame。
- `/wheel/odom.header.frame_id` 仍为 `odom`，`child_frame_id` 仍为 `base_footprint`，使其能作为导航融合输入；topic
  命名和消息内部 frame 语义是两套独立契约。
- 导航域必须发布最终 `/odom` topic，并成为动态 `odom -> base_footprint` TF 的唯一权威发布者。rt-control 的
  `robot_state_publisher` 继续发布 `base_footprint -> base_link -> ...` 本体 TF，不接管 `map -> odom` 或
  `odom -> base_footprint`。
- 必须同步检查 motion/Nav2、可视化、录包、监控和联调脚本的订阅名称。
- 修改后需要验证 `/wheel/odom` 唯一发布者、消息频率、时间戳、frame_id/child_frame_id；在只启动 rt-control 时应
  明确不存在 `odom -> base_footprint`，联合启动导航后该边必须只有导航域一个发布者。

### 2.4 收益与代价

收益：明确区分轮速原始测量与导航融合结果，并消除 rt-control 与导航同时广播 `odom -> base_footprint` 造成的 TF
竞争、跳变和多权威问题。

代价：这是跨域可见名称和 TF 所有权变更；现有订阅 `/diff_drive_controller/odom` 的消费者必须迁移到
`/wheel/odom`。导航未启动或启动失败时，TF 树将有意缺少 `odom -> base_footprint`，此时依赖 odom 连通性的导航、可视化
和感知消费者不可用，不能由 rt-control 自动补发一个备用 TF。

### 2.5 当前源码实现状态

- diff-drive 配置已设 `enable_odom_tf: false`；Launch 将其 odometry 发布端显式 remap 到 `/wheel/odom`。
- 无设备 Mock 实扫只有 `/wheel/odom` 一个发布者，消息 frame 保持 `odom/base_footprint`，频率约 50 Hz；旧
  `/diff_drive_controller/odom` 与 `/odom` 均不存在。
- 同一 Mock 在 6 秒过滤观察内没有 `odom` 父 TF；RSP 的静态 `base_footprint -> base_link` 仍为
  `z=0.202094 m`。
- 目标机当前锁定镜像尚未切换；导航域的 `/odom` 和唯一 `odom -> base_footprint` 仍需在联合联调中验证。

## 3. 当前状态

| 项目 | 状态 | 当前运行版本是否包含 |
| --- | --- | --- |
| 主接触器掉电恢复入口 | 源码已实现，待镜像和实机掉电—复电验收 | 否 |
| `/wheel/odom` topic 与 `odom` TF 所有权移交 | 源码和无设备 Mock 已完成，待镜像部署和导航联合验证 | 否 |
