---
id: contract-20260902-01
area: contract
title: 新增 base_link 下 lidar_main 固定 TF 与毫米制 STL
date: 2026-09-02
type: feature
trigger: "用户口头需求：提供 lidar_main.STL 及 base_link 下的 6DoF 位姿"
commits: [feat/robot-model-lidar-main-tf]
env: both
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - "工控机源树 xacro 展开与 check_urdf：PASS"
  - "工控机 robot_description 包构建：PASS"
  - "工控机隔离 ROS Domain 230 的 /tf_static 数值 smoke：PASS"
  - "工控机独立 lidar-main 镜像构建：PASS"
  - "本地 tools/tests/test_tf_contract.py：PASS"
  - "STL SHA-256：549449b3acf4da35264538a678bd82a1448c8571180e9c467eee949beac4197a"
supersedes: []
related: [BQ-125, T-020, ELECTRI-106]
---

## 背景

现场需要在共享机器人模型中增加主雷达 frame `lidar_main`。用户提供了相对
`base_link` 的 6DoF 位姿和齐次变换矩阵，并提供了带有坐标系/单位声明的 STL。
该变更属于公共 Robot Model/TF 资产，不改变任何 ROS 接口 schema 或实时控制逻辑。

## 改动

- 在 `robot_description` 中新增 `lidar_main` link 和 `lidar_main_joint` 固定关节。
- 固定边为 `base_link -> lidar_main`，`xyz` 为
  `0.382364228640 0.133500000000 0.121820508080`，`rpy` 为
  `0 0.523598775598 0`。
- 将用户 STL 安装到 visual/collision 资产目录；文件头声明
  `frame=base_link; unit=millimeter`，因此 mesh 使用 `scale=0.001 0.001 0.001`。
- STL 顶点保留在其声明的 `base_link` 坐标中；visual/collision 使用给定位姿的逆
  origin `xyz=-0.270226881461 -0.133500000000 -0.296681769019`、
  `rpy=0 -0.523598775598 0`，避免几何重复变换。
- 按现有传感器模型约定同时提供 visual 和 collision mesh；未猜测质量或惯量。

## 验证

已验证：

- 工控机源树执行 `xacro`、`check_urdf`，树中出现 `base_link -> lidar_main`。
- 工控机隔离 Domain 230 的 `robot_state_publisher` 使用 transient-local
  `/tf_static` 收到精确平移和四元数（绕 Y 轴 +30°）。
- 工控机 `robot_description` 包构建、完整独立测试镜像构建通过；镜像未替换生产容器。
- 本地 xacro、STL 资产校验、TF 契约测试和 `tools/quality_gate.sh` 通过。

未验证：

- perception、motion、navigation 和 RViz 尚未完成同一模型版本的联合消费验证。
- 生产容器切换后的 Domain 7 `/tf_static` 与碰撞规划效果另由 release/deploy 记录验证。
- 本记录不授权使能或运动。

## 结论与冻结事实

- F1: 共享模型新增固定 TF 边 `base_link -> lidar_main`，位姿为
  `xyz=(0.382364228640, 0.133500000000, 0.121820508080)`、
  `rpy=(0, 0.523598775598, 0)`。
- F2: `lidar_main.STL` 的权威资产 SHA-256 为
  `549449b3acf4da35264538a678bd82a1448c8571180e9c467eee949beac4197a`，坐标系为
  `base_link`，单位为毫米，消费时必须使用 0.001 缩放。
- F3: 该变更不新增公共接口、不改变 `odom -> base_footprint` 所有权，也不改变任何
  ros2_control 关节、限制或实时控制参数。

## 遗留

该模型变更必须回填平台/Robot Model 权威仓库，并通知 perception、motion、navigation
和 TF/RViz 消费者同步版本。正式合入 `main` 前需完成 PR、统一模型版本、生产镜像重建
及目标机 `/tf_static` 验证；若碰撞 mesh 对规划产生不兼容影响，应在消费者联合评审中
单独裁决，而不是静默删除碰撞几何。
