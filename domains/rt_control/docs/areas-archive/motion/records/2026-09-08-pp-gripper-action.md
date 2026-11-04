---
id: motion-20260908-01
area: motion
title: 官方夹爪 Action 的可选 PP 握手和最大力下发扩展
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
related: [BQ-147, ecat-axes-20260908-02]
---

## 背景

复用 ros2_controllers 固定版本的 position_controllers/GripperActionController，
补齐其位置适配器未下发 max_effort、不了解 PP 确认与停止状态的缺口。

## 改动

- `0003-opt-in-pp-gripper-commands.patch` 增加默认关闭的 `pp.enabled` 路径，旧位置/effort 控制器不变。
- 保留标准 GripperCommand Action，新增限力、序号、Halt 命令接口和 PP 状态反馈，不创建公共 `/rt` 旁路。
- 非有限数、越界目标、非正或超限力、未就绪和 busy 状态拒绝新目标；不提供覆盖正在执行目标的队列。
- 取消等待同序号停止反馈；到位等待同序号 PP 握手和位置容差。取消/成功并发用原子状态决定先后。
- PP update 只做固定大小运算、接口访问和原子快照；Action 的锁及反馈/结果发布留在非 RT 回调。
- 左右 controller 配置模板保留 TBD。具体 joint、使能组和 scope 运行接入不从旧机参数推断。

## 验证

- 先在未扩展的固定上游上运行 5 项测试并观察到所需行为失败，然后实现扩展。
- Action 测试覆盖限力下发、busy 拒绝、取消/故障、超时、重激活序号、缺配置拒绝、停滞和原始 PDO 闭环。
- 上游 12 个 controller 用例及插件加载测试通过。最初缺少 hardware_interface_testing，
  在隔离目录解包官方测试依赖后复验通过，未修改宿主 ROS 安装或上游测试断言。
- 容器内同一套 Action 测试通过；全量 Docker 构建下载阻塞情况见关联硬件记录。
- 仓库质量门禁：214 个工具用例通过，门禁覆盖率 83%。ShellCheck 本地未安装，保留 CI 检查。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `max_effort` 是每次命令的 N 单位限制；不支持用零值表达无限力，硬件仍执行独立上限。
- F2: 运行中的新目标被拒绝；用户需要取消并等待终态后重发。`allow_stalling` 不代表已经抓稳物体。
- F3: Action 取消成功意味着软件已收到配置要求的停止反馈；超时/故障 ABORTED 不表示物理停机已确认。

## 遗留

真实夹持力、PP 限矩生效顺序和取消停止距离需实机验证。公共 Motion 调用契约、实际机器人模型、
scope-specific controller 生命周期不在本轮发布；alfa_v3 实机 launch 继续阻塞。
