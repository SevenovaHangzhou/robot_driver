# ELECTRI-102 软件／mock 完成说明

> 日期：2026-08-19
> 结论：rolling trajectory transport 已达到 Motion mock 联调门；真实视觉伺服与硬件运动未验收。

## 1. 交付结果

本期核心问题不是“驱动能不能收一条 Twist”，而是：Motion 能否仅凭公共接口和文档，持续
提交一条安全可替换、可停车、可诊断的 14 轴 future，并在 FJT 与 rolling 之间保持唯一
command writer。

这一软件目标已经完成：

- 公共 IDL 按 provider 拆到 `robot_interfaces`，Motion batch 与 RT session/state 没有私有
  反向依赖；
- 新 rolling controller 在 CSP mode 8 下接收完整 14 轴 cubic-Hermite suffix，RT 以 250 Hz
  采样；不修改 PDO、CiA402 mode 或普通 FJT 限值；
- Motion 采用 100 ms knot、30 Hz 完整 suffix update；一次一臂，其他轴从已接受轨迹的对应
  future time 填值；
- FJT/JTC 与 rolling 使用 verified STRICT mode switch，同一时刻至多一个 writer；
- provisional 包络、超时、horizon、capacity、splice、状态、RejectCode、close 生命周期均
  参数化并有 fail-closed 测试；
- public-IDL-only producer 和 DDS peer 能完成 mode→open→prime→30 Hz update→两阶段 close→
  回 FJT；
- Docker 与 Native 入口在硬件访问前显式检查 Motion 接口、rolling controller 和 rolling QoS
  的安装态运行闭包；
- 14 项异常／长稳矩阵与当前干净实现提交的真实墙钟 10 分钟 fake loop 通过。

“完成”在本文只表示上述 software/mock checkpoint。它不表示相机图像、手眼 TF、视觉控制律、
真实关节响应、精度或 production safety envelope 已通过。

## 2. 仓库与交付分支

| 仓库 | 分支／身份 | 状态 |
| --- | --- | --- |
| `SevenovaHangzhou/robot_interfaces` | `功能/视觉伺服-ELECTRI-102` @ `9cc937970736cd19fd3bf5283de8cc5c15926967` | 已推送；54 项门禁单测与 Humble 全 6 包构建通过；未提 PR。 |
| `SevenovaHangzhou/robot_driver` | `功能/视觉伺服-ELECTRI-102`，功能／镜像提交 `185a343b08be877cc96941e6187a102c270847ff` | 原子提交完成并推送；锁定上述接口 SHA；不提 PR。 |
| `SevenovaHangzhou/robot_description` | 无可用权威工作树／无外参 artifact | 未修改；T-01 明确未完成，见 BQ-139。 |

接口仓保持 `[Unreleased]`／包版本 0.7.0；没有伪造 0.8.0、tag 或 release。按用户裁决，Motion
实际接入并确认 schema 可用后再单独提接口 PR。

## 3. 冻结的生产参数

| 项 | 当前值 |
| --- | ---: |
| Motion knot period | 100 ms（10 Hz） |
| Motion batch publish | 30 Hz |
| RT update | 250 Hz |
| planned / required initial horizon | 500 / 500 ms |
| max horizon | 600 ms |
| buffer capacity | 64 active points |
| update timeout | 200 ms |
| replace lead | 16 ms provisional |
| rolling state period | 20 ms |
| prime timeout | 100 ms |
| rotary rolling velocity | 0.2617993878 rad/s（15 deg/s） |
| rotary acceleration / stop | 0.75 rad/s² |
| Updown velocity | 0.09 m/s |
| Updown acceleration / stop | 0.5 m/s² |

这些限值只作用于 rolling visual-servo path，不改变 normal FJT/JTC。大 splice 容差按用户批准
保留并登记为 `RISK-E102-001`；Motion 仍负责真正平滑，不应使用满容差。

## 4. T-01～T-21 完成矩阵

| 任务 | 状态 | 结果／边界 |
| --- | --- | --- |
| T-01 手眼外参与描述副本 | **外部阻塞** | 没有权威 robot_description 仓或 calibration artifact；未猜值、未只改副本。BQ-139。 |
| T-02 关闭 JTC topic 旁路 | **PASS** | 首点准入启用时不创建 `~/joint_trajectory`；Action 回归与 mock 图验证通过。 |
| T-03 公共 rolling IDL | **PASS** | provider-owned IDL、固定 14 轴、独立 Reject/Stop/Service result，接口分支已推。 |
| T-04 命名 QoS | **PASS** | C++/Python `rolling_command`、`rolling_state`，合同与运行图检查通过。 |
| T-05 rolling controller 移植 | **PASS** | public contract overlay 构建、插件加载及完整核心测试通过。 |
| T-06 enable_manager 注册表 | **PASS** | 两 writer 注册、非法集合 fail configure、disable 全停用。 |
| T-07 配置／命名／inactive bringup | **PASS** | `whole_body_jtc` 与 rolling mandatory INACTIVE，JSB 配置 100 未改、实测 125 Hz。 |
| T-08 文档基线纠正 | **PASS** | 旧原型只作历史输入；当前决策、交接、性能和 area records 以 main 名称／125 Hz 事实重写。 |
| T-09 删除重复验证 | **PASS** | 每段 direct validation 一次，RejectCode 顺序不变。 |
| T-10 增量后缀验证 | **PASS** | 证明前缀验证结论不变；Prime 全量、replacement 只验变化 suffix，并保留 full oracle。 |
| T-11 有效点复制 | **PASS** | wire ceiling 256 不变；capacity 64 最大有效复制 14,936 bytes，RT 零 allocation。 |
| T-12 单调采样游标 | **PASS** | generation-aware O(1) 命中，逐拍与 reference bit-exact。 |
| T-13 validation CPU 亲和 | **PARTIAL** | 启动机制证明普通 executor 留 housekeeping；当前目标 TID/PSR 动态证据待 BQ-140。 |
| T-14 provisional source | **PASS** | production/test-only/provisional 三态分离，public state 持续暴露。 |
| T-15 provisional envelope | **PASS** | 14 轴严格 schema、raw SHA version、无 URDF/PLC 隐式 fallback；台架实测待 BQ-138。 |
| T-16 参数外置 | **PASS** | YAML configure-time 冻结，越界失败，Open 回报实际生效值。 |
| T-17 FJT↔rolling 快切 | **PASS（mock）** | 46 项 mode state-machine 测试、STRICT 结果复核、安全抢占；目标耗时待 BQ-140。 |
| T-18 mock 矩阵 | **PASS** | 14/14；真实墙钟 600 秒，150,000 周期，18,001 batch 全接受，零容忍计数全 0。 |
| T-19 lead／切换标定 | **PARTIAL** | 桌面分位数已记录；没有目标 DDS/generation/switch 分布，16 ms 不升级为 production。 |
| T-20 Motion 说明／示例 | **PASS** | 完整指南、21 个 RejectCode 响应、dry-run producer、public DDS peer；1 秒 31/31 batch。 |
| T-21 契约／BQ／进度 | **PASS** | 决策 D01～D47、area records、BQ-138～141、PROGRESS 和精确接口 pin 已更新；门禁强制每项含问题、至少两个选项、推荐项与最终采用项。 |

T-01、T-13 动态部分和 T-19 目标部分不阻塞“Motion mock transport”完成口径，但明确阻塞
真实视觉伺服／production 声明。

## 5. 最终验证摘要

- `robot_interfaces`：生成视图、contract、error-code、changelog 门禁 PASS；54 tests PASS；
  rosdep 无缺项；ROS 2 Humble 全 6 包 clean build PASS。
- 两仓真实源码联合构建：13 packages PASS。硬件 driver 依赖来自已验证 underlay；本机完整
  `packages-up-to` 仍受既有 `lely_core_libraries` 本地缺失限制，没有把该环境缺口写成代码失败。
- driver package test result：276 tests，0 errors，0 failures，0 skipped；包含完整 mock launch
  7 项、JTC topic absence、writer inactive、QoS 和 `/joint_states` 125 Hz 实测。
- repository quality gate：210 passed、1 skipped、83% 门禁覆盖率；skip 只因为通用门不加载
  外部接口 overlay，同一 public DDS test 已在锁定 overlay 下单独 PASS。本机没有 shellcheck，
  该项由 CI 强制，两个本地 shell 文件已通过 `bash -n`。
- 当前无脏改动的实现提交 `185a343...` 真实墙钟 acceptance：14/14 PASS；600 秒内完成
  150,000 个 250 Hz 周期和 18,001 个 30 Hz batch，reject、LateReplace、RT allocation、
  invariant failure、late cycle 均为 0；update p99/p99.9 为 22.231/31.586 us，validation
  p99/p99.9 为 109.146/121.615 us。
- public DDS：1 秒 31 个 sequence 全接受，hardware/controller-manager/enable 均未启动。
- 发布镜像 `rt-control:185a343b08be877cc96941e6187a102c270847ff` 完整构建 PASS，image ID
  `sha256:664f02f97a79910229d6c345cf1803b011541324fbfa265b1859378710c8fc8f`；安装态包／QoS
  smoke PASS。
- 同一镜像的无硬件容器 smoke 使用 `network=none`、`devices=[]`、Domain 223；JTC／rolling
  均 INACTIVE、enable_manager ACTIVE，五个 rolling 端点可见，JTC topic 旁路不存在，停止
  exit 0 且无 ERROR/FATAL。没有执行 production compose `up`、reset、enable 或 motion。

## 6. 明确未完成／不得外推

1. **手眼与真实视觉闭环**：无权威 joint5→camera 外参；左右 TF、左手镜像精度、IBVS/PBVS
   机械闭环均未验收（BQ-139）。
2. **production envelope**：15 deg/s、0.75 rad/s² 等仍为 ESTIMATED_NOT_MEASURED；必须台架
   测方向、停车、跟踪误差与 margin（BQ-138）。
3. **目标机 timing**：DDS、generation、LateReplace、STRICT switch 和 validation PSR 待测；
   16 ms lead 仍 provisional（BQ-140）。
4. **标准全栈 mock enable**：GenericSystem 不模拟 CiA402，公共 enable 会在 0x0040 fail-closed；
   没有为获得绿灯绕过生产状态机（BQ-141）。
5. **明确不在范围**：TF 发布频率、相机／工控机时钟同步、图像时间戳对齐、CSV mode 9、
   双臂同时伺服、左手独立标定、0.8.0 正式发布。

## 7. Motion 下一步

Motion 可以立即基于接口 SHA `9cc9379...` 实现 producer，并以
`electri-102-motion-integration-guide.md` 和两个工具为验收 oracle。建议先在隔离 ROS Domain
完成 hold、单臂小幅整形、丢帧降速和 RejectCode 注入，再把接口反馈提交给 RT-Control。

在 BQ-138～141 和单独现场授权完成前，不得启动真实总线、reset、enable、FJT、rolling 或
任何机械臂运动。本期执行没有进行这些动作。
