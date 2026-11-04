---
id: release-deploy-20260913-01
area: release-deploy
title: ELECTRI-118 对齐 PR25 后的 main 并验证三代机集成
date: 2026-09-13
type: corrective
trigger: 用户要求重新处理最新 main 冲突并创建新 PR
commits: []
env: both
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [release-deploy-20260912-02]
related: [ELECTRI-118, BQ-147, ecat-axes-20260912-01, ecat-axes-20260912-11]
---

## 整合范围

用户新合并的 PR #25 将 GitHub main 更新到
`8fc332f1aa9544039243a07d6db1226f2d17242a`。此次从该提交建立
`feature/electri-118-gen3-modular-bringup`，迁入此前 ELECTRI-118 的模块配置、
PP 夹爪、swerve_driver、物理清单和双臂原位使能实现。保留原开发和上一次同步工作区，
未部署或操作工控机。原来的三个根目录工作笔记不进入 PR。

核心兼容问题是：新增原位入口必须遵守 main 的启动和主站生命周期约束。
保留 X503 PREOP 一次快照、非 PREOP CoE 拒绝、Ti5 分配恢复及原有 0001..0012
补丁；PP 周期钩子仍作为 0013 追加。三代机预检只在 master inactive、全部 PREOP
时读取 SDO，运行中不新增邮箱访问。

## 冲突处理

- 三处文本冲突：bringup CMake 同时保留 PREOP、machine profile 和 stationary 测试；
  PROGRESS 与 EtherCAT 区域索引合并双方记录。
- 保留 main 的 `ecat-axes-20260912-01`；三代机当日记录顺延为 02..11，更新引用。
  保留 main 的 `ecat-axes-20260908-01`；PP 记录改为 02，更新 Action 记录引用。
  所有区域记录 ID 已核验唯一。
- 保留此前的 PP 事项 BQ-147 和周期钩子 0013，未覆盖 main BQ-143 或原有补丁。
- 上游加载测试复现已记录的 Humble 测试插件缺失：`minimal_robot_urdf` 引用
  `test_actuator`，安装的 hardware_interface 只提供 namespaced 测试组件。
  CI 增加官方 `ros2_control@e65ddd72804f3f2d9b19e533a15ed436b2f3fc42`
  的 `hardware_interface_testing` 构建和测试 overlay。该依赖只在 CI 临时目录中使用，
  不更改生产依赖 pin、主站、公共接口或运行镜像。

## 验证

- `REQUIRE_SHELLCHECK=1 tools/quality_gate.sh`：277 passed、13 skipped；
  受门禁代码覆盖率 83%，ShellCheck、架构、文件卫生和停机策略通过。
  跳过项需要未配置的外部源码检查环境，不计作通过。
- bringup 的 stationary、machine profile、composition、PREOP 与 X503 包 pytest：
  166 passed。首次因未加入 X503/QoS Python 路径而失败；补齐冻结依赖路径后通过。
- 生产 Dockerfile 完整构建：32 个包通过，全部冻结补丁按序应用成功。
  首次直连 VCS 下载停滞后终止，传入既有环境的预定义代理 build args 后成功；
  没有修改 Dockerfile 网络策略或在仓库记录代理值。
  镜像 `rt-control:electri-118-main-pr-20260913`，ID
  `sha256:7e2baab36cba196977975aa4cb9e3a8419a96e19293aa7bd35cdce253a4dc104`。
  `src/patches/docker/tools/deps.repos/versions.env` 与 PR 工作区逐文件比对一致；
  构建后的补充仅涉及文档引用、验证记录和 CI 测试依赖。
- 镜像内重新构建 enable_manager、robot_hw_ethercat、swerve_driver 并运行
  `colcon test --merge-install --packages-select enable_manager robot_hw_ethercat swerve_driver`：
  215 条测试记录，0 errors、0 failures、0 skipped。首次测试命令漏传
  `--merge-install`，在执行测试前被 colcon 拒绝；修正命令后通过。
- 上游 gripper_controllers、ethercat_interface、joint_trajectory_controller 已重新构建；
  加载用例缺插件和离线 xmllint 缺 schema 的环境问题已复现。
  补齐上述官方测试插件，并通过 XML catalog 提供 ROS 官方 schema 本地副本后，
  三个失败目标定向重跑为 6 条记录通过。完整首轮结果保留，并在同一构建产物上执行
  `colcon test --merge-install --ctest-args --rerun-failed`，最终汇总 709 条记录，
  0 errors、0 failures、12 skipped；跳过项是 ament 对 cppcheck 2.7 性能问题的既有
  文件检查跳过，不是新设测试豁免。JTC 的 496 个控制器用例和 113 个 Action 用例通过。
- 两个独立的 `--network none --cap-drop ALL` 容器运行安装后的入口：
  `rt_control_start --enable-only use_mock_hardware:=true` 加载三个 active 控制器，
  恰好 16 个可用且已占有的 control_word 命令接口，无运动命令接口；
  默认旧机入口使用 Mock 并关闭 PLC/BMS，三个状态/使能控制器及差速控制器 active，
  whole_body_jtc inactive。两者均经 `/rt/disable`、控制器停用、ecat_arms inactive
  后正常退出，exit code 0，无 UNCLEAN_SHUTDOWN。
- Git 差异、冲突标记、记录 ID、workflow YAML、暂存区及 PR 契约在提交前检查。

## 边界与遗留

本轮为源码集成和无硬件验证，不是新基线实机验收。旧原位使能实测仍属于原开发快照，
不能替代此版本的 DC/连续周期交接、上电使能、停机和故障复位复验。
旧记录中的 OP 下 SDO 回读不能继续作为新 main 的验证方法；新主站只允许 PREOP CoE，
运行模式核验需使用合适的 PDO/现场证据。

三代机通用模块 launch 保留 draft/runtime 门禁；真实 Model、scope runtime 组合、
步科和编码器映射/标定、PP 位移/力换算及 Halt 停止语义仍待闭合。
PP 模式原位使能不触发 bit 4，也不代表夹爪运动与限力已验证。
旧机默认启动配置、模型与公共接口保持兼容；EcSlave ABI 扩展要求全部 EtherCAT
消费者同批重构建，禁止混用旧二进制。GitHub CI 结果以新 PR 实际运行结果为准。

## 结论与冻结事实

- F1: 三代机集成已基于 GitHub main@8fc332f，保留 PR25 的启动/CoE 约束并消除冲突。
- F2: 完整镜像和旧机/三代机 Mock 生命周期验证通过，真实运行准入与实机验收仍分开管理。
- F3: 官方测试硬件仅用于 CI/隔离验证，不形成生产硬件依赖或替代实际设备证明。
