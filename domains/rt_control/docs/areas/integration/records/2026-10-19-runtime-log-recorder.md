---
id: integration-20261019-01
area: integration
title: 可选 Runtime-LOG 1 kHz 记录控制器接入组合器与 scope 启动，台架默认开启
date: 2026-10-19
type: feature
trigger: ELECTRI-127；用户 2026-10-18 决定 Runtime-LOG P1（黑匣子）先用现有状态接口打通，插件在工控机单独编译，A/B 通过后台架默认开启
commits: [feature/electri-127-runtime-log-recorder]
env: native
risk: T3
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PASS
evidence: []
supersedes: []
related: [ecat-axes-20261017-01]
---

## 背景

Runtime-LOG（个人私有仓库 `kkozia188/Runtime-LOG`）的 P1 黑匣子需要在 1 kHz update 线程里把状态接口拷进共享内存。
插件 `runtime_log_recorder/RecorderController` 放在 Runtime-LOG 仓库；该仓库私有，组织 CI 拉不到，
所以不进 `deps.repos`：本仓库不依赖它编译，插件在工控机上用 Runtime-LOG 的 `deploy/build_recorder.sh`
按固定提交单独编译进同一 merged install，提交号写入 `install/share/runtime_log_recorder/RUNTIME_LOG_BUILD`。

## 改动

- `unit_deployment.schema.json`：新增可选 `runtime_log_recorder: {enabled, slots}`。
- 组合器：开启且 scope 为分级启动时，加入控制器 `runtime_log_recorder`，参数 `interfaces` 为生成的
  robot_description 中全部 ros2_control 状态接口（按描述顺序），`slots` 默认 2000；写入 `lifecycle.yaml` 的
  `recorder_controllers`。它不进 `controller_order`；关闭或缺省时产物与改动前逐字节相同（测试覆盖）。
- scope 启动：生命周期控制器加载成功后，用独立 spawner（进程名 `runtime_log_recorder_spawner`）加载；
  它的退出不被监视，插件缺失或加载失败不会停止 scope。
- 有序停止：记录控制器在 `controllers_inactive` 阶段与生命周期控制器一起停用，位于驱动失能之后，失能过程仍被记录。
- `Alfa_V3-bench-01` 部署开启记录控制器（A/B 通过后）。

## 实时约束（插件侧，Runtime-LOG 仓库）

- 共享内存段在 `on_configure`（非实时）创建、全段触页、`mlock`；update 内只读预解析的接口指针并拷贝 double，
  无分配、锁、系统调用、日志。Humble 的 `on_activate` 在 update 线程内执行，因此只做指针解析（容量已预留）。
- 共享内存不可用时控制器仍正常配置和激活，只是不记录（日志 `RUNTIME_LOG_RECORDER_DISABLED`）。
- 不认领命令接口，没有控制器依赖它。

## 验证

- composer + bringup 测试 253 passed（新增 4 项：关闭/缺省无变化、开启时全部状态接口且其余产物不变（mock 与实机两种）、
  `slots` 越界被 schema 拒绝；停机顺序含记录控制器）；`check_v3_branch_contract`、`rt_control_layer_gate`、
  `repository_gate` PASS；`tools/tests` 247 passed / 13 skipped。
- 台架（失能、无运动）：重新生成 `bench_left_arm` 产物，只有 `controllers.yaml`（新增控制器）、`lifecycle.yaml`
  （`recorder_controllers`）和 `artifact.yaml` 变化；启动后记录控制器配置并激活（32 个字段，段 644 KB，已 mlock），
  Runtime-LOG 读者无缺口。
- A/B（每组 600 s、60 万周期，CPU14 上 `sched_switch` 测 RT 线程切入间隔）：

| 组 | 标准差 µs | 偏差 p99 / p99.9 / 最大 µs | 偏差 > 50 µs | 丢帧 / 发送错误 |
| --- | --- | --- | --- | --- |
| A：未加载记录控制器 | 0.524 | 3 / 5 / 9 | 0 | 0 / 0 |
| B：记录控制器激活 | 0.866 | 4 / 5 / 9 | 0 | 0 / 0 |
| C：同一次启动、记录控制器停用 | 0.881 | 4 / 5 / 9 | 0 | 0 / 0 |

  B 与 C 一致，标准差差异来自两次栈启动之间，不来自记录控制器；尾部指标三组相同；CPU14 无外来进程。结论：无可测影响。
- 手动触发：保存 12001 周期窗口（前 10 s、后 2 s），无缺失。
- 停后：主站 Idle 后按白名单 SDO 读 7 轴 0x603F/0x6041 成功（ar 用户）。

## 遗留

- 整机部署是否开启另行决定；整机字段数更多，接入时按同样方法 A/B。
- 零差 PDO 扩展（速度、力矩、0x603F）、ICube 指令镜像、控制器埋点属 Runtime-LOG P1 第二批。
- 最近 3 次台架启动中有 2 次在交接时出现数千个 UNMATCHED 与发送错误（已预绑核），需另查。
