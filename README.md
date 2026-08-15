# robot_driver

半人形拆码垛机器人的 **RT-Control 实时控制域仓库**。本仓库只保存实时域实现、
构建封装、宿主配置和验收证据，不接收 Perception、Motion、Autonomy 或 Gateway 的
业务实现代码。

其他域只通过版本锁定的公共 ROS 2 契约与 RT-Control 交互。域间 endpoint 和 wire
schema 的权威源是独立
[`robot_interfaces`](https://github.com/SevenovaHangzhou/robot_interfaces) 仓库；
本仓库通过 `deps.repos` 固定并导入 RT-Control 需要的公共包，只在树内保存本域私有接口。

## 责任边界

RT-Control 负责：

- EtherCAT/CANopen、ros2_control 和 250 Hz 实时控制环；
- 完整 14 轴轨迹、履带安全速度和 PLC/BMS IO 执行；
- 驱动使能、失能、故障复位、停机收敛和硬件诊断；
- 发布机械轴、轮速里程计、TF、真空、安全摘要和域就绪状态。

RT-Control 不负责：

- 任务顺序、行为树、导航与机械臂规划、感知推理；
- `map -> odom`、`odom -> base_footprint` 等外部坐标边；
- 用软件状态替代急停、安全继电器、STO 或驱动器保护；
- 收纳或直接引用其他四域的内部库、配置和业务代码。

## 接口所有权

- `src/vendor/robot_interfaces`：由 `deps.repos` 固定 SHA 导入的公共契约、接口包与
  `robot_interfaces_qos`；`src/interfaces/source-lock.yaml` 保存同一身份元数据。
- `src/interfaces/rt_control_interfaces`：仅供 RT-Control 域内使用的
  `PlcIoState`、`RtEnable` 等 msg/srv。
- `src/vendor/robot_interfaces/contract/views/rt_control.md`：随 vendor 导入的 RT-Control
  契约视图，不是本仓库的第二份事实源。

公共接口发生破坏性变化时，必须先修改 `robot_interfaces`，再由全部生产者和消费者
锁定同一提交原子升级。ROS 2 两侧类型不一致时可能都正常启动但完全无法通信。

## 仓库结构

```text
robot_driver/
├─ domains/rt_control/      # 实时域规则、进度、阻塞问题和验收记录
├─ src/
│  ├─ description/          # robot_description 构建副本
│  ├─ interfaces/           # RT-Control 私有接口 + 公共契约 source-lock
│  ├─ vendor/               # 构建时从 deps.repos 导入，不入库
│  └─ rt_control/           # 硬件、控制器、诊断和 bringup
├─ docker/rt-control/       # RT-Control 镜像
├─ docker/compose.yaml      # RT-Control 部署，不是其他域通用模板
├─ hostsetup/               # 实时域宿主安装与验收
├─ patches/                 # 冻结上游窄补丁
├─ tools/                   # 构建、启动、门禁和运维工具
└─ collaboration-and-commit-standards.md
```

## 分支模型

| 分支 | 定位 | Docker 要求 |
| --- | --- | --- |
| `main` | RT-Control 稳定集成与交付载体 | 必须提供可复现镜像和容器验证 |
| `native` | 原生增量开发主线 | 可免容器封装，但实时、安全和质量要求不降低 |

所有共享分支禁止直接 push，变更经 feature/bugfix 分支和 PR 合并。完整规则见
[AGENTS.md](AGENTS.md) 与
[协作提交规范](collaboration-and-commit-standards.md)。

## 文档入口

| 文档 | 内容 |
| --- | --- |
| [domains/rt_control/README.md](domains/rt_control/README.md) | RT-Control 构建、运行与验收入口 |
| `src/vendor/robot_interfaces/contract/views/rt_control.md` | 固定 SHA 中的 RT-Control 公共契约视图 |
| [domains/rt_control/docs/one-command-start.md](domains/rt_control/docs/one-command-start.md) | 一键启动与接口速查 |
| [domains/rt_control/PROGRESS.md](domains/rt_control/PROGRESS.md) | 已完成工作及验证证据 |
| [domains/rt_control/BLOCKED-questions.md](domains/rt_control/BLOCKED-questions.md) | 已裁决和待裁决问题 |

## 本地质量门禁

```bash
tools/quality_gate.sh
```

接口、硬件包或启动路径变更还必须按
[RT-Control AGENTS.md](domains/rt_control/AGENTS.md) 构建和测试全部受影响包。没有容器、
目标机或实机证据时必须明确记录为未验证，不得从源码检查推导运行结论。
