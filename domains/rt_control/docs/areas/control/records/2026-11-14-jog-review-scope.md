---
id: control-20261114-01
area: control
title: PR 260 收敛为 Jog 并统一控制器停稳确认
date: 2026-11-14
type: corrective
trigger: 用户接受 PR 260 的 Jog 范围评审；2026-11-14 指示沿用原任务标识 arch-flexible-startup
commits: [feature/arch-flexible-startup]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 背景

针对 de30a2c 的请求修改评审，用户选择将 #260 收敛为 Jog；柔性启动和 PDO 分域保留在独立分支，不改变工控机部署。本次隔离副本合入 main@f74d3ba，保留主线状态页面、Robot Model 与完整实时摘要的实现，以及最新日志轮询生命周期和相机预览 CSP 修复。

## 改动

- 移除模块探测、动态配置选择、网页模块卡片、PDO 补丁 0015–0017及其专属测试；这些路径与当前 main 一致。移除无关的 CI 缓存改动。启动包仅保留本轮完整构建所需的 controller_manager 2.54.3 兼容，证据见下文。
- 最短链路：Jog 客户端 → `/ws/jog` → 共享 ArmAuthority/模式协调 → RollingOutput → 公共 RollingSender → Rolling 控制器。epoch、sequence、确认和终态由发送器拥有，轨迹准入由控制器拥有。
- EnableManager 增加域内只读 `~/confirm_standstill`（std_srvs/Trigger）。模式切换和查询共用内部实时样本、连续窗口和原有 stableActualInterval 判据。查询必须观察请求之后的新窗口；停机、忙、过期或没有新样本均不能确认成功。不切换模式，不写驱动命令。
- JTC-only 配置中已有 motion_joints 也用于该只读确认；网页先核对查询覆盖的关节与受控关节一致。网页保留请求终态和控制器状态证据，删除自己的位置/速度停稳算法、参数策略和 Hermite 加速度预检。
- ArmPage 是唯一权限后台轮询拥有者；删除无生产调用的 confirm_stopped、重复 session 身份和 ConsoleError 转发文件。保留动作发送前需要的权限代次快照，避免取消后迟到发送。
- Python 绑定暴露公共 Phase、SendResult、MAX_POINTS、HOLDING 和状态时效。真实发送器传输测试接入标准 CTest，自动设置扩展路径及 localhost Domain 219；缺失构建产物直接失败。

## 验证

ROS Humble，固定公共接口 78f02ffe5b8f40a9f57bf9405cd00e007914fc84 的源码在隔离目录重新构建；未改 pin。构建和日志位于本机 `/tmp/pr260-jog-20261114`。

- `cmake --build`：EnableManager、Rolling、网页及发送器绑定通过；公共依赖和叶子库在隔离前缀构建。
- `ctest --test-dir <enable-build> --output-on-failure`：109 项通过。
- `ctest --test-dir <rolling-build> --output-on-failure`：70 项通过。
- `ctest --test-dir <web-build> --output-on-failure`：372 项通过，含标准真实 RollingSender 传输测试。
- 上述共 551 项，0 失败、0 跳过。覆盖互斥、模式/保持查询中取消和断线、终态/停稳屏障、未知结果不交接、JTC-only、无新样本、反馈过期、移动中拒绝确认、连续 Jog 的保持值衔接及主线摘要保护。
- `tools/quality_gate.sh`：通过；工具测试 185 通过、11 条依赖未导入的公共接口/IgH 条件跳过；shellcheck 本机未安装。未将这些缺项记为通过。
- `tools/arch_check.py compose --against origin/main`：27 种选择产物不变。
- 初次构建因缺依赖前缀失败；首次测试发现旧夹具和动作权限快照的遗漏。修正后重新运行全部受影响包，以上均为最终源码结果。

### CI 构建兼容修复

head d5d7322 的 CI 43563462262 在 Build 阶段因安装 controller_manager 2.54.3、而启动包仅接受 2.53.1/2.54.2 失败；后续测试未执行。本地使用已安装的 2.54.3 复现同一配置错误后，仅增加这个已核实版本，不放开未知版本。

- 官方 2.53.1、2.54.2（e822ad539633d1d286d1a85ab4e0390c79f218e7）、2.54.3（65d389596953cb4656c4216a3f0bc08a73230739）的 ros2_control_node.cpp 与仓库冻结副本逐字节相同，SHA-256 均为 ca6192e765b4ad433e9cd9053ca8209982f6367e301afd7f99620baa3c4eed92；现有补丁在三者上均通过 `patch --fuzz=0 --dry-run`。
- `integ_bringup` 重新配置、编译和链接通过；链接已安装的 controller_manager 2.54.3，无缺失动态库。没有修改冻结源码或补丁。
- 标准 CTest 4 个目标全部通过。独立包测试初次缺少源码 Python 依赖路径；补齐当前仓库的依赖路径后重跑全部目标，未修改或跳过测试。
- 修复后的 `tools/quality_gate.sh` 通过；证据目录 `/tmp/pr260-jog-20261114/ci-failure-43563462262`。完整 CI 仍以新 head 结果为准。

软件包测试不构成 T1 全图 mock 验收或实机验证；未运行本版本全栈 mock、容器、实机总线、使能和运动。历史记录中的现场证据只适用于当时提交。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: ArmAuthority 只在请求终态明确、且之后的权威实际停稳确认新鲜时交接。停稳判据由 EnableManager 唯一拥有。
- F2: RollingSender 拥有发送协议；Rolling 控制器拥有轨迹准入；操作台不维护第二份轨迹数学限制。
- F3: #260 不包含柔性启动或 PDO 分域；原 de30a2c 保存在 feature/arch-flexible-startup-preserved-20261114。

## 遗留

任务关联按用户指示沿用 #260 原标识 arch-flexible-startup，不编造 Linear/GitHub Issue 编号。PR 标题和描述通过可用的认证通道更新。本版本 T1–T4 与容器/全仓构建尚未验证，CI 结果以推送后新 head 为准。
