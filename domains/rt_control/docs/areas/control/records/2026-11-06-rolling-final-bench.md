---
id: control-20261106-02
area: control
title: Rolling final 正常收尾、STARTING 与断流停止实机验证
date: 2026-11-06
type: commissioning
trigger: 方案 1 接口变更（final、STARTING、STOP_COMPLETED）后的实机验证
authority: 用户 2026-11-06 “你来做实物验证吧”，运动前逐段通知
commits: [075c403]
env: native
risk: T4
writes: {reset: no, enable: yes, motion: yes, plc: no}
verified: PARTIAL
evidence:
- ar@192.168.100.40:~/rt-control-arch-20261106/runs/final-20261106/client.jsonl
- ar@192.168.100.40:~/rt-control-arch-20261106/runs/final-20261106/launch.log
supersedes: []
related: [control-20261106-01, control-20261105-03]
---

## 范围

新部署 `~/rt-control-arch-20261106`：robot_driver `075c403`，robot_interfaces `ea647d6`（vendor 与锁定一致），
其余 vendor 与镜像沿用 `20261105d`。scope `bench_dual_arm_head`，profile `bench_dual_arm`，ROS domain 78。
组合产物与旧部署相比只有 `robot_description.xml` 中的部署目录名不同。旧栈（`20261105d`）有序停止：
RT_SCOPE_SHUTDOWN 四项均为 True，18 从站回到 PREOP。新栈启动后 15 OP / 3 PREOP；`runtime_log_recorder`
仍为已知的加载失败。

发送端为 robot_interfaces `cb99666` 的 `RollingSender`（台架外置测试程序 `client_two_degree`）：等待连接与首批
结果，30 Hz 发送 0.4 s 窗口，最后一个窗口 `final=true`。轨迹以开始时的实测位置为中心
0 → +2° → −2° → 0，6 s，峰值 3°/s；每段运动前均已通知现场。

## 结果

| 测试 | 批次 | 结果 | 停止原因 | 回位误差 | 其他关节漂移 |
| --- | --- | --- | --- | ---: | ---: |
| 右 J7 | 1–172 | completed | STOP_COMPLETED | −0.0048° | 0.0069° |
| 右 J7（同一 epoch 再次开始） | 173–344 | completed | STOP_COMPLETED | −0.0027° | 0.0048° |
| 左 J7 | 345–516 | completed | STOP_COMPLETED | −0.0021° | 0.0089° |
| 右 J2 | 517–688 | completed | STOP_COMPLETED | +0.0007° | 0.0055° |
| 左 J4 | 689–860 | completed，2 批 NOT_ACCEPTING | STOP_COMPLETED | +0.0007° | 0.0048° |
| 右 J7，3 s 断流 | 861–954 | stopped | STOP_UPDATE_TIMEOUT | 停在 −0.588° | 0.0062° |

- final 批发出后 463–473 ms 报 STOP_COMPLETED，即窗口剩余时长（约 0.45 s）加状态发布周期；不再等待更新超时。
- 首批均在第一次发送时被接受（未出现首批等待超时）；epoch 全程为 1，序号连续。
- 断流：最后一批后按更新超时刹停并保持；之后切回 FJT_READY，JTC 将右 J7 从约 −0.11° 送回 0.472°（目标 0.48°）。
- 左 J4 运行中 2 批 NOT_ACCEPTING：控制器继续执行上一次接受的轨迹，后续批次正常接受，运动按计划完成。
  此前 Python 脚本 14 段实机未观察到；隔离环境同插件 2/15 段出现。原因未查明，单独排查，不归入本次改动。
- `/rt/disable` 返回 success；readiness 为 IDLE / not_enabled，errors 为空。

本次实机操作经现场授权，操作人：现场用户与 Agent；确认短语入口：用户“你来做实物验证吧”。

## 结论与边界

- final 正常收尾、同一 epoch 再次开始、断流停止三种结局在实机上均按新契约报告，发送端无需任何时间推断。
- 未在实机验证：FINAL_NOT_AT_REST 拒绝（单元测试覆盖）、final 之后的非 final 替换（单元测试覆盖）、
  STARTING 期间切模式被拒（未测）。只测了 4 个关节。
- 运行中偶发 NOT_ACCEPTING 未解决。
