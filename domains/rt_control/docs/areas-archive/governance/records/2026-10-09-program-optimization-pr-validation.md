---
id: governance-20261009-01
area: governance
title: 程序优化 PR 提交前门禁与隔离验证
date: 2026-10-09
type: investigation
trigger: 用户要求执行协作流程并提交 feature/rt-control-program-optimization 的代码与 PR
commits: [feature/rt-control-program-optimization]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - /tmp/rt-program-optimization-quality.log
  - /tmp/rt-program-optimization-build.log
  - /tmp/rt-program-optimization-final-tests.log
  - /tmp/rt-program-optimization-test-result.log
  - /tmp/rt-program-optimization-gripper-test-result.log
  - /tmp/rt-program-optimization-mock.log
related: ["robot_driver#52", BQ-149, BQ-150, BQ-152]
supersedes: []
---

## 背景

按 Issue #52 集成当前 BMS、头部、IMU、LED 与力传感器相关改动，目标分支为
`main`。功能事实及历史现场证据分别保留在各功能区记录；本记录仅描述本轮
提交前验证，不把历史实机记录认定为本轮重新验证。

## 改动

- 校正启动包测试：新传感器台架入口需要安装且声明广播器运行依赖，其余独立模块
  依赖隔离断言保留。台架默认 Mock，完整 V3 draft 门禁不变。
- 头部 README 构建与 source 示例从仓库根目录执行，移除开发者本机绝对路径。
- 本地 `.git/info/exclude` 排除 build_head/install_head/log_head、临时导入的
  src/src 和 symlink_install_manifest.txt；未删除用户文件，未修改公共门禁。
- 按功能拆分提交；公共接口 pin/schema、Robot Model、Docker/Compose、宿主配置不变。

## 验证

环境：ROS 2 Humble，隔离输出 `/tmp/rt-program-optimization`；现有冻结并打补丁的
EtherCAT 台架安装作为 underlay。本轮是受影响范围构建，不是全仓干净构建或发布验收。

- `git diff --check`：PASS。
- `COVERAGE_FILE=/tmp/rt-program-optimization.coverage bash tools/quality_gate.sh`：
  PASS，207 passed / 13 skipped，门禁覆盖率 83%。本机无 ShellCheck，CI 强制执行。
- `bash -n tools/build_force_sensor_bench.sh`：PASS。
- 对全部 7 个变更 Python 文件执行 `python3 -m py_compile`，字节码输出到隔离目录：PASS。
- 隔离 `colcon build`：PASS，初次 15 个包；补充 robot_description、固定版本且应用
  三个既有补丁的 gripper_controllers，以及 CI 相同的 hardware_interface_testing。
  ros2_controllers 固定 SHA `cbcf66218ff43353f9fb5fe7a2c33f458d578d73`，三个补丁逐一
  `git apply --check` 后应用到临时副本。测试夹具固定 SHA
  `e65ddd72804f3f2d9b19e533a15ed436b2f3fc42`；用户 vendor 工作树未修改。
- `bash /tmp/rt-program-tests.sh`：PASS。脚本在本机 ROS_DOMAIN_ID=77、
  ROS_LOCALHOST_ONLY=1 下对 bms_node、damiao_head_controller、lpms_nav3_can、
  modbus_tcp_rtu485、robot_hw_ethercat、rt_force_torque_broadcaster、
  rt_control_bringup 逐包执行 `ctest --test-dir <build/package> -T Test --output-on-failure`，
  共 36 个目标；同样执行 gripper_controllers 的 2 个目标，全部通过。
- `colcon test-result --test-result-base /tmp/rt-program-optimization/build --verbose`：
  PASS，324 条汇总结果、0 errors / 0 failures / 0 skipped（包括 CTest 与包内测试结果）。
- `colcon test-result --test-result-base /tmp/rt-program-optimization/gripper-build --verbose`：
  PASS，15 条汇总结果、0 errors / 0 failures / 0 skipped。
- 本机 ROS_DOMAIN_ID=78、ROS_LOCALHOST_ONLY=1 执行 `python3 /tmp/rt-program-mock.py`：
  PASS，显式 use_mock_hardware=true；头部 20 次失能请求成功且控制器存活，左右力传感器
  均发布 9 路命名原始状态，两套 Mock 启动均正常结束。

初次失败与修复：质量门禁扫描到未忽略的本地构建生成物；首次 ROS 测试被沙箱拒绝
创建 DDS 套接字，已停止。随后测试误用系统未补丁夹爪，且启动包测试缺少模型依赖、
存在与新增台架入口冲突的依赖断言。使用隔离冻结补丁、补齐依赖并修正断言后重跑。
附加夹爪加载测试缺少 test_actuator 插件，按 CI 的固定 SHA 测试夹具补齐后通过。
保留初次日志；最终结果不混入旧 CTest dashboard 失败报告。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 提交前公共质量门禁和受影响包自动测试通过；不代表全仓 CI、实机或发布验收通过。
- F2: 头部响应生命周期修复在本轮 Mock 中连续 20 次失能成功；力传感器台架 Mock
  能发布两侧完整九路状态并退出，不确认工程单位、TF 或采样时效的现场有效性。
- F3: 本地生成物和导入依赖保持原位，仅通过本地 Git 排除避免进入源码检查和提交。

## 遗留

完整 CI 和负责人评审待完成；本机 ShellCheck 未执行。K2 实体接口与安全阈值、
主电池断电行为、IMU 协方差标定、力传感器单位/TF/状态语义/新鲜度及长期运行、
头部真实生命周期仍需现场验证。本轮无 CAN/Modbus/EtherCAT 实机操作，无生产 Docker
封装；历史 LED 与传感器实测记录保持各自条件和限制。
