---
id: arm-control-20261019-01
area: arm-control
title: JTC 轨迹点数上限（max_trajectory_points，默认 2000）
date: 2026-10-19
type: fix
trigger: 2026-10-19 12:01:06 台架左臂超周期事故（大轨迹导致首个 update 8.38 ms）
commits: [feature/electri-127-jtc-max-points]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 背景

2026-10-19 12:01:06，外部工具把一条大约 13k–14k 点的轨迹直接发给 `/left_arm_jtc/follow_joint_trajectory`。

Humble JTC 收到新轨迹后，第一个实时 `update()` 对每个点执行 `fill_partial_goal` 和 `sort_to_local_joint_order`，这个周期用了 8.38 ms。

`ros2_control_node` 接着连续补跑落下的周期，导致以下结果：

- WKC 从 1 变为 9；
- 出现 AL 0x001A；
- 7 轴全部报 0x1208 故障；
- enable_manager 进入 FAILED。

上游的 humble、jazzy 和 master 分支都还在实时线程里做这件事。完整分析见 Runtime-LOG 仓库的 `docs/investigation-2026-10-19-jtc-large-trajectory.md`。

## 改动

新增补丁 `patches/ros2_controllers/0004-limit-trajectory-points.patch`，并接入 bootstrap 和 Dockerfile。

- 新参数 `max_trajectory_points`：
  - 类型为整数，默认 2000；
  - 设为 0 时关闭上限。
- `validate_trajectory_msg` 拒绝点数超过上限的轨迹，并输出错误日志，提示重采样或分段发送。
  - 话题和 action 两个入口都会调用这个函数，它运行在非实时回调里。
- gtest `invalid_message` 增加两条用例：
  - 2000 点通过；
  - 2001 点被拒。

`rolling_trajectory_controller` 不受影响。

## 验证

- 工控机构建通过。以下 4 个测试套件全部通过，共 630 项：
  - `test_trajectory_controller` 498 项；
  - `test_trajectory_actions` 113 项；
  - `test_tolerances` 5 项；
  - `test_trajectory` 14 项。
- `test_load_joint_trajectory_controller` 失败，与本补丁无关：
  - 原因是当前工作区的 ros2_control 测试硬件插件没有声明 `test_actuator`；
  - 失败发生在加载硬件阶段，控制器还没有加载。
- mock 栈（domain 92，`use_mock_hardware:=true`）实测：
  - 每个点都等于当前位置，所以仿真中也不会运动；
  - 周期数据由 Runtime-LOG recorder 采集。

| 点数 | 结果 | 最长周期 |
| --- | --- | --- |
| 1000 | 接受 | 1004 µs |
| 2000 | 接受 | 1171–1177 µs（新轨迹到达的那个周期超出大约 170 µs，没有补跑突发） |
| 2001 / 3000 / 30000 | 拒绝 | ≤ 1005 µs |

## 结论与冻结事实

- F1: 超过 `max_trajectory_points` 的轨迹在非实时回调里被拒绝，实时周期不受影响。（T1 mock）
- F2: 默认上限为 2000，由用户指定。在上限附近，新轨迹到达的那个周期大约是 1.17 ms。驱动能不能承受这个延迟，台架还没有验证；需要每个周期都不超时的运动方，应把每条轨迹控制在 1000 点以内。
- F3: 台架只确认了补丁已经编译进工作区。原因：
  - 拒绝超限轨迹需要 JTC 处于激活状态，而 JTC 激活依赖使能；
  - 当时机械臂驱动已断电，没有在台架上发送轨迹。
