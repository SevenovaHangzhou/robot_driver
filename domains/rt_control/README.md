# RT-Control 域地图

开工必读只有根 [`AGENTS.md`](../../AGENTS.md) 与 [`ARCHITECTURE.md`](../../ARCHITECTURE.md)。本页是一屏导航，
不是事实源；硬件值、裁决和验证结论以下表"记录"列出的文件为准。

## 包与层（`src/rt_control/`）

| 层 | 包 | 一句话 |
| --- | --- | --- |
| infra | `infra_modbus` `infra_renderer` `infra_shm_ring` `infra_socketcan` | 不依赖任何域包的共享叶子库（Modbus TCP 长连接客户端、协议渲染器的 schema 校验与 XML 参数、Runtime-LOG 共享内存环、SocketCAN 收发） |
| hw_config | `hwcfg_machines` | 机型 manifest、模块、设备、台套（零位/标定/部署）数据，无代码 |
| protocol | `hw_ethercat` `hw_canopen` `hw_can_damiao` `hw_semantic_components` | 总线/设备适配、renderer、CiA402 语义封装 |
| control | `ctrl_enable_manager` `ctrl_rolling_trajectory` `ctrl_swerve` `ctrl_force_torque_broadcaster` | 使能状态机与各控制器/广播器 |
| periph | `periph_bms` `periph_imu_lpms` `periph_led_ultrasonic` `periph_plc_io` | 独立外设节点（BMS、IMU、LED/超声、PLC IO） |
| integration | `integ_composer` `integ_bringup` `integ_diagnostics` `integ_operator_web` | 离线组合、启动入口、诊断、操作网页 |

共享：`src/description/robot_description`（Robot Model 构建副本）、`src/interfaces/rt_control_interfaces`（域内 IDL）、
`src/vendor/robot_interfaces`（由 `deps.repos` 导入的公共契约，不入库）。

## 数据流

```text
hwcfg_machines ──> integ_composer ──> 各 protocol 包 renderer ──> artifact 目录 ──> rt_control_start
   (manifest/module/unit)       (校验 + 组合，纯函数)     (ros2_control 片段)       (compose_artifact)      (launch + PID 1)
```

```bash
ros2 run integ_composer compose_artifact --output /abs/dir --robot-variant Alfa_V3 \
  --machine-unit <unit> --physical-profile <profile> --control-scope <scope> [--mock]
```

台套：`Alfa_V3-01`、`Alfa_V3-Advantech-01`、`Alfa_V3-bench-01`。scope 定义在
`hwcfg_machines/machines/Alfa_V3/manifest.yaml`。

## 启动入口（`rt_control_start`，容器 PID 1）

除默认的静态校验外，所有运行入口都是通用分阶段 scope 入口 `rt_control_scope_runtime.launch.py`：
先加总线锁、跑 owner preflight，再按顺序激活硬件、加载控制器；停机统一走
`scope_shutdown`（停运动控制器 → `/rt/disable` → 停其余控制器 → 硬件退出）。
每个入口都必须显式传入 `robot_variant:=` 与 `machine_unit:=`，并与制品记录的选择一致；不传即拒绝启动
（用户 2026-11-06：不再用代码默认值猜台套，容器默认命令同样需要写明）。

| 参数 | 等同于 | 说明 |
| --- | --- | --- |
| （无） | `rt_control_module.launch.py` | 只做 machine-profile 校验，不创建硬件节点 |
| `--scope <scope>` | — | 通用入口；scope 需声明 `staged_hardware_activation` |
| `--arm-runtime` | `--scope arms_only` | 双臂运动（JTC + rolling），发布 RSP |
| `--arm-jtc` | `--scope arms_only jtc_only:=true` | 双臂仅 JTC |
| `--enable-only` | `--scope arms_stationary_enable physical_profile:=arms_only` | 双臂原位使能（250 Hz） |
| `--head-runtime` | `--scope head_only` | 头部云台，使能由 `enable_manager` 负责，见 [达妙头部](../../src/rt_control/hw_can_damiao/docs/damiao_head.md) |
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

### 历史证据

已退役架构的导航、启动/部署指南和完成的交接计划不再保留；需要追溯时查 Git 历史。
实机记录与能力审查报告仍作为证据保留，例如 [宿主验收记录](docs/host-setup-record.md)、
[迁移基线报告](docs/baseline_report.md) 和 `docs/` 下带日期的调试/验收记录。
它们只说明记录时点、对应主机和配置，不能作为当前 V3 操作步骤；当前启动入口见本页，
构建、部署与授权分别见 `tools/README.md`、`docker/README.md`、`hostsetup/README.md` 和根 `AGENTS.md`。
