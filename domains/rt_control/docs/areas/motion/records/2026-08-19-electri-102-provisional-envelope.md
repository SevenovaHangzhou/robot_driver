---
id: motion-20260819-10
area: motion
title: ELECTRI-102 14 轴 provisional 包络严格加载
date: 2026-08-19
type: feat
trigger: ELECTRI-102 / T-15 / E102-D08 / E102-D37 / BQ-138
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/rt_control_bringup/config/rolling_envelope_provisional.yaml, src/rt_control/rolling_trajectory_controller/src/envelope_loader.cpp, src/rt_control/rolling_trajectory_controller/test/test_envelope_loader.cpp, src/rt_control/rt_control_bringup/test/test_rolling_envelope_contract.py, src/rt_control/rolling_trajectory_controller/test/test_state_publisher.cpp]
supersedes: []
related: [ELECTRI-102, BQ-138]
---

## 改动

新增固定协议轴序的 14 轴 provisional artifact。每轴显式提供位置上下界、正负速度、正负
加速度、正负停车加速度和双侧 position margin；旋转位置界取当前 driver URDF 构建副本与
`joint_limits.yaml` 的更保守交集，其他数值按 ELECTRI-102 已批准规则。metadata 明确
`ESTIMATED_NOT_MEASURED`、估计方法、owner、BQ-138 排期入口和“不替代台架实测”警告。

loader 严格要求 root/metadata/axis 字段全集、14 轴顺序、有限正动态量和可用安全区间；不从
URDF、PLC 或代码补缺项。原始文件字节 SHA-256
`e355f72990a1c73b62d591f733cd4d2e743b78298f541dc0e92ce0ec16ccd0c4` 直接成为
`limits_version`。

controller 只有在 source=provisional、独立 provisional opt-in 和文件成功加载三者同时成立
时配置；启动记录 WARN。Open/public state 把 source=3、test_only=false 和同一 hash 持续
暴露。activation 重建 stop checker 时保持 provisional authority，不退回 test-only。

## 验证

- loader RED 先因缺少 API 失败；实现后覆盖有效 artifact、10 个逐字段缺失、错误 source、
  错轴序、未知字段、零值、负值、NaN 和缺文件，全部 fail-closed。
- controller RED 证明 test-only opt-in 不能放行 provisional；GREEN 覆盖独立 opt-in、缺文件、
  mock activation、WARN、public source/test-only/hash。
- 独立 pytest 从当前 URDF 和 `joint_limits.yaml` 重算 14 轴位置交集，并参数化检查 14 轴
  速度／加速度／停车／margin：16/16 通过。
- rolling CTest 13/13；bringup 的 envelope contract CTest 1/1。

没有访问现场总线、reset、enable 或运动。权威 robot_description 尚不可用，未来同步会改变
交集时必须更新 artifact/hash；这不会被本 PASS 隐藏。

## 冻结事实

- F1: provisional artifact 的精确字节 SHA-256 是 public limits version。
- F2: 缺字段、错来源、错轴序或非法数值均 fail-closed，且无 URDF/PLC fallback。
- F3: provisional source 在 configure、activation、Open 和 public state 中保持一致。
- F4: 本文件不替代 BQ-138 台架实测，也不授权硬件运动。
