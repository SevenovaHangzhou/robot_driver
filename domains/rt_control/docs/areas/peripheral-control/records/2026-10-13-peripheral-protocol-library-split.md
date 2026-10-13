---
id: peripheral-control-20261013-01
area: peripheral-control
title: 四个外设包的协议库与 ROS 转换分离
date: 2026-10-13
type: feature
trigger: KKOZIA-3（SPEC v1.1 §5.7 / ADR-11）
commits: [feature/rt-control-peripheral-protocol-kkozia-3]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 背景

按架构师裁定从治理基线 e82e0d5 建分支，与四层迁移并行，Draft PR 投 main，
依赖 KKOZIA-4（M0a）。本任务为第三档不改行为的重构；三个新协议库的独立 gtest
是 issue 明确验收要求。此前因治理文件与编号缺失暂停，已按 2026-10-13 裁定解除。

## 改动

- bms_node 的 protocol/can_transport 与 lpms_nav3_can 的 decoder/socket_can
  头文件移入各自 `include/<pkg>/protocol/`；沿用原库目标及 gtest。
- modbus_tcp_rtu485 新增 `modbus_transport`、`ultrasonic_decoder` 库，原 inline
  函数体移到 cpp；`ultrasonic_ros_conversion` 在协议目录外将中立 severity 映射为
  原 ROS 诊断级别，并保留 Range 字段与配置校验。
- plc_io_modbus 的 ModbusClient 方法移至 cpp，安装导出 `modbus_client` 静态库。
  生产网络 IO、超时、资源关闭、异常文本与节点调用次序保持不变。
- 新增三个独立 gtest，复用既有 Modbus/E08 向量与 PLC 源码既有帧语义。
  链接器 `--wrap` 注入假 IO；分片、异常、错误头、断连、模拟 poll 超时与恢复均不访问设备。
  原自定义 main 测试保留，原断言只随 include/ROS 转换接缝同步，未放宽。
- 消费者：BMS 节点、dual_battery_controller 与其测试；LPMS 节点、ros_conversion 与测试；
  LED/超声波节点及原传输测试；PLC 节点。全仓搜索未发现其它旧头路径消费者。
  下游 bringup 只消费节点运行入口。外部 C++ 消费者须改 include 路径并重新链接；
  ROS 话题、服务、参数、消息定义、配置数值、供电/真空逻辑和 plc_node 未修改。
- 他人路径机械性小改仅为 `domains/rt_control/PROGRESS.md` 追加本记录索引；
  未修改 R8、CI、Docker、bootstrap 或其它工程代码。R8 实现归 KKOZIA-6。

## 验证

环境为本地 ROS 2 Humble / C++17；仅从 deps.repos 导入 robot_interfaces 固定提交
ea582c8a711874813d2ad7f0801b366bd32ab67e，未改依赖锁定。

- T0：`colcon build --symlink-install --packages-up-to bms_node lpms_nav3_can modbus_tcp_rtu485 plc_io_modbus --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=`：8 包通过。
  clang-format 14 改动行格式化后四包再次构建通过，编译日志无 warning/error。
  首次构建发现 ultrasonic_ros_conversion 的 CMake PUBLIC/plain 签名混用，修正后通过。
- T0/T1：source ROS 与 install 环境后，`ROS_LOCALHOST_ONLY=1 colcon test --packages-select bms_node lpms_nav3_can modbus_tcp_rtu485 plc_io_modbus --return-code-on-test-failure` 通过。
  `colcon test-result --test-result-base build/<包> --verbose` 分别报告 58、34、13、4 项，
  合计 109 项结果、0 errors、0 failures、0 skipped（colcon 汇总含 CTest/gTest 结果）。
- T0：`python3 -m pytest src/rt_control/rt_control_bringup/test/test_ultrasonic_launch.py src/rt_control/rt_control_bringup/test/test_interface_runtime_launch.py -q`：8 passed。
- T0：对四包 protocol 目录执行 issue 给定的 R8 等价 include 检查：7 个头，0 违规。
  临时外部 CMake 消费者通过 find_package 链接七个安装后协议目标，编译及无 IO 运行通过。
- T0：`tools/quality_gate.sh` 通过：225 passed、12 skipped，门禁集合覆盖率 84%；
  12 项跳过属于未提供 EtherCAT/IgH 上游源的既有测试；本地无 shellcheck，CI 执行该检查。
  `git diff --check`、固定版本 style gate、暂存区审查通过。
- `tools/run_scoped_tests.sh --base e82e0d5` 未全通过：四包测试通过，但脚本扩展到
  未构建的 rt_control_bringup。尝试 `colcon build --packages-select rt_control_bringup`
  被缺少 robot_hw_ethercat、robot_hw_can 等整机运行依赖安装产物阻止。
  未伪造这些依赖；以上 8 项相关启动契约以源码 pytest 单独验证。
  早期两次工具调用还分别遇到临时 venv 缺 PyYAML、未 source ROS 的环境错误；
  后续恢复系统 Python 并 source ROS/install 后，四包全部通过，剩余失败仅是 bringup 闭包。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 四包共七个协议/传输库以 `include/<pkg>/protocol/` 为头文件边界，头文件无 rclcpp 或 *_msgs include；ROS 转换位于该目录之外。
- F2: 协议库可由安装后的 CMake targets 独立链接；已有协议/节点测试和三个新增 gtest 通过，ROS 运行接口及控制行为未作修改。
- F3: T0/T1 结果不构成实机、整机 bringup 闭包或正式发布验收；未开放任何设备写入权限。

## 遗留

完整 bringup 闭包与 CI 全量结果待具备依赖的环境确认；R8 正式规则由 KKOZIA-6 接入。
M0a 合入后合并 origin/main（不 rebase、不强推）再标 Ready，合并由人执行。
T2 及以上未执行；真实通信、供电切换、PLC/灯带输出和现场停机由人按独立授权验证。
既有 BQ-150/BQ-152 硬件准入限制不因本次拆库解除，本任务无新增硬件事实或 BQ。
