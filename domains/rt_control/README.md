# RT-Control 域地图

开工必读只有根 [`AGENTS.md`](../../AGENTS.md) 与 [`ARCHITECTURE.md`](../../ARCHITECTURE.md)。本页是一屏导航，
不是事实源；硬件值、裁决和验证结论以下表"记录"列出的文件为准。

## 包与层（`src/rt_control/`）

| 层 | 包 | 一句话 |
| --- | --- | --- |
| infra | `infra_modbus` `infra_shm_ring` `infra_socketcan` | 不依赖任何域包的共享叶子库（Modbus TCP 长连接客户端、Runtime-LOG 共享内存环、SocketCAN 收发） |
| hw_config | `rt_control_machine_config` | 机型 manifest、模块、设备、台套（零位/标定/部署）数据，无代码 |
| protocol | `robot_hw_ethercat` `robot_hw_canopen` `robot_hw_can` `rt_control_semantic_components` | 总线/设备适配、renderer、CiA402 语义封装 |
| control | `ctrl_enable_manager` `ctrl_rolling_trajectory` `ctrl_swerve` `ctrl_force_torque_broadcaster` | 使能状态机与各控制器/广播器 |
| periph | `periph_bms` `periph_imu_lpms` `periph_led_ultrasonic` `periph_plc_io` | 独立外设节点（BMS、IMU、LED/超声、PLC IO） |
| integration | `rt_control_composer` `rt_control_bringup` `control_api_adapter` `rt_control_operator_web` `rt_control_arm_console` | 离线组合、启动入口、公共适配、诊断、操作网页 |

共享：`src/description/robot_description`（Robot Model 构建副本）、`src/interfaces/rt_control_interfaces`（域内 IDL）、
`src/vendor/robot_interfaces`（由 `deps.repos` 导入的公共契约，不入库）。包的去留与改名计划见
[迁移待办](docs/migration-backlog.md) §5a。

## 数据流

```text
rt_control_machine_config ──> rt_control_composer ──> 各 protocol 包 renderer ──> artifact 目录 ──> rt_control_start
   (manifest/module/unit)       (校验 + 组合，纯函数)     (ros2_control 片段)       (compose_artifact)      (launch + PID 1)
```

```bash
ros2 run rt_control_composer compose_artifact --output /abs/dir --robot-variant Alfa_V3 \
  --machine-unit <unit> --physical-profile <profile> --control-scope <scope> [--mock]
```

台套：`Alfa_V3-01`、`Alfa_V3-Advantech-01`、`Alfa_V3-bench-01`。scope 定义在
`rt_control_machine_config/machines/Alfa_V3/manifest.yaml`。

## 启动入口（`rt_control_start`，容器 PID 1）

除默认的静态校验外，所有运行入口都是通用分阶段 scope 入口 `rt_control_scope_runtime.launch.py`：
先加总线锁、跑 owner preflight，再按顺序激活硬件、加载控制器；停机统一走
`scope_shutdown`（停运动控制器 → `/rt/disable` → 停其余控制器 → 硬件退出）。

| 参数 | 等同于 | 说明 |
| --- | --- | --- |
| （无） | `rt_control_module.launch.py` | 只做 machine-profile 校验，不创建硬件节点 |
| `--scope <scope>` | — | 通用入口；scope 需声明 `staged_hardware_activation` |
| `--arm-runtime` | `--scope arms_only` | 双臂运动（JTC + rolling），发布 RSP |
| `--arm-jtc` | `--scope arms_only jtc_only:=true` | 双臂仅 JTC |
| `--enable-only` | `--scope arms_stationary_enable physical_profile:=arms_only` | 双臂原位使能（250 Hz） |
| `--head-runtime` | `--scope head_only` | 头部云台，使能由 `enable_manager` 负责，见 [达妙头部](../../src/rt_control/robot_hw_can/docs/damiao_head.md) |
| `--force-sensors` | `--scope force_bench` | 腕部力传感器台架，实机需 `sync_mode:=sm|dc` |

任何非 mock 启动都会访问真实总线，需按根 `AGENTS.md` 第 5 节单独授权。容器与发布见 [`docker/README.md`](../../docker/README.md)。

## 改什么去哪

改动类型与唯一位置见 [`ARCHITECTURE.md`](../../ARCHITECTURE.md) §2；验证命令见 [`tools/README.md`](../../tools/README.md)。

## 记录

| 文件 | 内容 |
| --- | --- |
| [BLOCKED-questions.md](BLOCKED-questions.md) | 阻塞问题与用户裁决（2026-10-29 重新开始，写入前须用户确认；旧条目见 git 历史） |
| [PROGRESS.md](PROGRESS.md) | 验证时间线索引 |
| [docs/areas/](docs/areas/README.md) | 7 个功能区的冻结事实表与开发记录 |
| [docs/areas-archive/](docs/areas-archive/) | 2026-10-13 前 9 个旧功能区的记录（只读归档） |
| [testing/](testing/README.md) | 发布前测试分层、用例目录与 runner |
| [docs/v3-arm-motion-integration.md](docs/v3-arm-motion-integration.md) | V3 双臂与运控对接方案 |
| [docs/hardware-configuration-layering-and-motor-variants.md](docs/hardware-configuration-layering-and-motor-variants.md) | 硬件配置分层与电机变体（部分内容描述二代 descriptor） |

### 历史资料（不作为当前操作依据）

二代/旧机记录，仅用于追溯：
[接手知识图谱](docs/onboarding-knowledge-map.md)、
[一键启动](docs/one-command-start.md)、
[原生开发与运行](docs/native-development-workflow.md)、
[新机部署与运行手册](docs/deployment-operations-runbook.md)、
[宿主安装记录](docs/host-setup-record.md)、
[联调准入摘要](docs/integration-readiness-summary.md)、
[后续修改计划](docs/follow-up-modification-plan.md)、
[PLC/BMS 合并交接](docs/plc-bms-merge-hardware-handoff.md)、
[迁移基线报告](docs/baseline_report.md)，
以及 `docs/` 下带日期的调试/验收记录。
