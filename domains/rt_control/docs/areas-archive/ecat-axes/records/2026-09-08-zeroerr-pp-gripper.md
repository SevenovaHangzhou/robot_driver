---
id: ecat-axes-20260908-02
area: ecat-axes
title: 零差 PP 夹爪硬件适配与限力配置骨架
date: 2026-09-08
type: feature
trigger: ELECTRI-118
commits: []
env: both
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-147, motion-20260908-01, release-deploy-20260907-01]
---

## 背景

用户批准复用官方 ROS 2 夹爪 controller，同时补齐 PP 与最大力下发。
本记录主要归属硬件适配；控制器行为见 motion-20260908-01。

## 改动

- `robot_hw_ethercat/ZeroErrPpSlave` 复用 GenericEcSlave 的注册及配置解析。
- 新增独立可测的 PP 握手/停止状态机，不新增 master，不改已有 CSP 族和旧机 profile。
- `6072` 使用额定电流千分比，开口 m、请求最大力 N，均依赖明确标定。整数限值向下取整。
- EtherCAT 窄补丁提供周期开始/读取完整性回调，沿用已有 WC 判定，无新增独立超时监视器；
  原编号 `0008`，同步 main 后顺延为 `0013-coherent-pdo-cycle-hooks.patch`。
- native bootstrap 与生产 Dockerfile 同步补丁顺序；draft 配置安装但不进入生产 launch。

## 验证

- RED: 新状态机/模块引用先编译失败；新增边界用例先失败，再实现修正。
- Native: `colcon build` 构建 gripper_controllers、EtherCAT 依赖和 robot_hw_ethercat；
  GTest 覆盖握手、原始 PDO、力限制、失能优先、故障、取消、配置拒绝和 Action 闭环。
  最终新增 30 个 C++ 用例；`colcon test-result` 汇总 98 条记录，0 错误/失败/跳过。
- 现有 EtherCAT profile/variant 的四个 pytest 测试目标通过；三代机 43 个配置测试通过，
  5 个物理 profile、10 个 profile/scope 组合静态校验通过。
- 独立覆盖率构建：PP 状态机行覆盖率 99.12%，适配器初轮行覆盖率 89.36%；后续增加配置拒绝用例。
- PP 状态机 ASan/UBSan 检查：13 个用例通过。
- Docker: 相同固定依赖 SHA 的本地基线镜像上完成 6 包重新构建和包内测试；
  `--network none --cap-drop ALL` 的独立测试容器运行通过。未映射任何 EtherCAT/CAN 设备。
  最终增量镜像 `rt-control:electri-118-pp-offline` 内 83 条测试记录通过，manifest digest 为
  `cfddfcb50391d8f1903242c146793bff9bcfde002ad0274694a5ad5a34b56443`。
- 从生产 Dockerfile 的全量构建在源码下载阶段停滞后终止，不能声称全量交付镜像已经验证。
- 自审：域边界/公共接口/旧配置兼容性 PASS；离线生命周期与停止语义 PASS；实机与完整交付
  镜像 UNVERIFIED；原有未提交配置改动保留，未暂存、提交或推送。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 夹爪 controller 不占有 control_word；enable_manager 的失能/复位指令优先，适配器仅负责 PP 位。
- F2: 新目标与力限制先进入已发送 PDO，再触发 bit 4；必须完成 bit 12 握手才允许报告到位。
- F3: 真实 profile、firmware、标定及 PP 限矩效果仍由 BQ-147 阻塞；draft/runtime gate 未放开。
- F4: EcSlave 虚接口增加回调，所有 EtherCAT 依赖必须同批重构建，禁止混用旧 ABI 二进制。

## 遗留

实际电机身份、PDO assignment、机械标定、驱动 PP 限矩和停止验证、整机 scope 生命周期、
真实 Robot Model 与完整 main 交付镜像仍待闭合。电机反馈换算的力是估算值，不是传感器测量。
