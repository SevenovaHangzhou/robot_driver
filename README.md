# robot

Robot 是半人形拆码垛机器人的机上软件 monorepo。仓库按五个逻辑域分工，但以一台机器人、一组共享契约和一份 release manifest 集成与发布。

## 当前状态

仓库目前已完整导入 **rt-control** 域代码、容器、宿主配置和验收记录。perception、motion、autonomy 和 gateway 将按同一目录与契约逐步并入。“当前只有 rt-control 实现”不改变根目录是整机公共仓库的定位。

rt-control 的构建、Docker、实机和质量门禁说明见 [domains/rt_control/README.md](domains/rt_control/README.md)。

## 域划分

| 域 | 核心责任 | 不负责 |
| --- | --- | --- |
| `rt-control` | EtherCAT/CANopen、ros2_control、轨迹/速度/IO 执行、硬件状态和实时诊断 | 任务顺序、导航/机械臂规划、感知决策 |
| `perception` | 传感器、点云/视觉、定位、箱墙任务计划和近拍 6D pose | 运动执行、BT/FSM |
| `motion` | MoveIt2/cuMotion/OMPL、碰撞世界、Nav2 和任务级运动 Action | Demo 流程编排、传感器推理 |
| `autonomy` | 顶层 FSM、任务级 BT、顺序/取消/恢复、SQLite 任务账本和健康汇总 | Nav2/MoveIt、感知推理、硬件实时控制 |
| `gateway` | 外部指令接入、MQTT/mTLS、状态/告警上报、离线缓存和 OTA | 绕过 autonomy 向 motion/rt-control 下发业务指令 |

机内调用方向为：

```text
external/fleet ↔ gateway ↔ autonomy → perception
                                      └→ motion → rt-control
```

## 共享资产

- `src/description/robot_description`：机器人物理/运动学权威模型源，属于 Robot Model/平台基础层，不属于任一业务域。
- `src/interfaces`：跨包/跨域 ROS 2 msg/srv/action 契约，不放业务实现。
- `config`（待并入）：按机器人 serial 和场景版本化的配置/标定资产。
- `docker` / `deploy`：五域镜像、整机 Compose、systemd 和 release manifest。当前 Compose 仅含 rt-control。

共享包不得依赖业务域实现。域间只通过冻结接口交互，不得跨域引用对方的内部库、私有配置或可写数据目录。

## 仓库结构

```text
robot/
├─ domains/                  # 各域说明、AI 契约、进度与阻塞记录
│  └─ rt_control/
├─ src/
│  ├─ description/          # 共享 Robot Model
│  ├─ interfaces/           # 共享 ROS 契约
│  └─ rt_control/           # 当前已导入的域实现
├─ docker/                   # 镜像与 Compose
├─ hostsetup/                # 当前为 rt-control 宿主安装与验收
├─ patches/                  # 当前为 rt-control 冻结上游窄补丁
├─ tools/                    # 仓库通用和域级工具
└─ docs/                     # 整机/部署/验收文档
```

新域并入时应同时建立 `domains/<domain>/README.md`、`AGENTS.md`、`PROGRESS.md` 和 `BLOCKED-questions.md`，不将域内事实堆回根目录。

## 开发与提交

1. 先阅读根 [AGENTS.md](AGENTS.md)。
2. 根据变更路径，再阅读 `domains/<domain>/AGENTS.md`。
3. 公共模型、接口或发布配置变更必须列出所有生产者、消费者和需联合验证的域。
4. 提交前运行 `tools/quality_gate.sh` 及受影响域契约要求的附加门禁。

仓库级质量门禁目前与 rt-control 首个落地域一起实现。后续新域加入时，在保留公共门禁的前提下增加域级 CI job，不得用新域的需求削弱已有安全检查。
