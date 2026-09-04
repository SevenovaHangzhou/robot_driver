---
id: release-deploy-20260904-01
area: release-deploy
title: ELECTRI-94 Native 依赖闭包与可移植 EtherLab 前缀
date: 2026-09-04
type: corrective
trigger: ELECTRI-94
commits: [a069ab398b1617f2546d32dbce76b70e06ec2f92]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - "隔离 workspace /home/kkozia/.tmp/e94-native-verify-a069ab3 从 deps.repos 干净导入四个 vendor；robot_interfaces=92d6ff2ed0b45684d7da2170d96703ca8be569f，ecat_icube=1390be742986f4e898ca112e49bb24805be9899a，ros2_canopen=fef50e54b1c94c50e908e2c5d0b8888eed907e8d，ros2_controllers=cbcf66218ff43353f9fb5fe7a2c33f458d578d73；ecat_icube 0001..0007、其它冻结补丁按序 apply/verify PASS。"
  - "IgH userspace-only 从 versions.env 的 2f7f884f1c7d377c02a7d627eb06512126a0e50e 构建到 workspace 私有前缀；0001-preserve-verified-pdo-config.patch SHA-256=89fcab7e72eac7fc4f00dd94ad133ac94ed1206b2cc55642a89d6cd869daf4ce；安装 ecrt.h 与 libethercat.so.1.2.0 与构建产物字节一致，未生成 .ko。"
  - "显式 -DETHERLAB_DIR=/home/kkozia/.tmp/e94-native-verify-a069ab3/igh-prefix 的 Native build：29 个生产闭包包全部完成；另有 ros2_control_test_assets 测试资源包通过，ethercat_manager/ethercat_msgs 额外目标通过。"
  - "同一 build/install/log 根执行 colcon test；补入同一 ros2_control_test_assets 提交 e65ddd72804f3f2d9b19e533a15ed436b2f3fc42 的 hardware_interface_testing test-only base path 后，colcon test-result --verbose 汇总 1433 tests, 0 errors, 0 failures, 18 skipped。"
  - "安装后的 robot_interfaces_qos 五个 profile runtime smoke PASS；tools/quality_gate.sh PASS（207 tests，策略覆盖率 83%；本机 ShellCheck 不可用，由 CI 强制）。"
related: [release-deploy-20260903-01, ELECTRI-94]
---

## 变更与验证

0007 是 ecat_icube 的新增顺序补丁，仅把 `ethercat_interface` 与
`ethercat_manager` 的 `ETHERLAB_DIR` 改为默认 `/usr/local/etherlab` 的 CMake
`CACHE PATH`；bootstrap 与 Docker 应用顺序同步，未改变驱动行为。默认生产路径保持兼容，
Native/CI 可通过 `-DETHERLAB_DIR` 选择隔离前缀。

首次全量测试暴露的三个上游 controller 失败均为测试插件缺失：测试资源引用
`test_actuator`，而宿主只提供 namespaced `test_hardware_components`。在验证 workspace
构建并安装同一 ros2_control 提交的 `hardware_interface_testing` 后串行重跑三个包，
失败归零；这不是六个独立缺陷。

## 运维限制

旧的、已经应用过冻结补丁且经过构建/测试工具写入的 vendor 工作树，不支持无条件原地
增量 `prepare`：冻结树校验可能因额外生成或格式化差异拒绝重新应用 0001。运维和 CI
必须使用新的空 workspace，从 `deps.repos` 重新导入并按顺序应用全部补丁；不得通过清理、
reset、改写 vendor 历史或扩大 patch manager 来掩盖差异。旧 workspace 保留作审计证据。

本记录没有构建或启动 Docker，没有启动 Native 运行时，没有连接总线、写设备、reset、enable
或运动；目标工控机和远端分支未操作。
