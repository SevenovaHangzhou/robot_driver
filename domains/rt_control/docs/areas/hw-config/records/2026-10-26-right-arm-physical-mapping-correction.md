---
id: hw-config-20261026-02
area: hw-config
title: 右臂物理映射从左臂1..7纠正为X3右臂9..15
date: 2026-10-26
type: corrective
trigger: 用户指出18121读到的是左臂姿态，并要求确认从站1..7的物理归属
commits: [feature/right-arm-scope-20261026]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - src/rt_control/rt_control_machine_config/evidence/Alfa_V3_bench_right_arm_scan_2026-10-26.yaml
supersedes: [hw-config-20261026-01]
related: [BQ-146]
---

## 背景

此前按“仅接右臂、X3、无力传感器”的预期接线创建9从站配置，却未先核对实际总线，
错误地将位置1..7命名为右臂。软件关节名称与网页反馈一致，只能证明软件链路，
不能证明物理设备归属。用户指出读数来自左臂后，本次只读读取主站缓存拓扑。

## 改动

- `bench_right_arm.yaml`、单机零点绑定与manifest由9从站、右臂1..7改为实际18从站、
  右臂9..15；左臂1..7与蓝点8/16只声明身份，不分配控制接口。零点计数、方向与编码器
  策略保持既有右臂配置数值。
- EtherCAT owner增加可选 `topology_preflight`，右臂运动scope启动前核对主站未激活、
  从站数量/身份/PREOP状态、X3和内部端口。该分支只调用缓存查询 `ethercat master/slaves`，
  不调用SDO；原位scope原有完整检查保持原行为。
- 远端按旧文件哈希检查后增量安装6个源码文件，保留远端既有 `csp_profile` 支持及
  其他用户改动。旧源码与活动产物备份到工作区 `backups/right-arm-physical-mapping-20261026`。
  安装及安装后校验时活动 `artifacts/bench_right_arm_motion` 与备份哈希一致；本次没有覆盖
  活动产物或启停运行栈。之后检测到目标机被外部切换，见下述最新运行状态。

## 验证

- T2只读：工控机执行 `ethercat master -m 0`、`ethercat slaves -m 0 -v`。
  实际18从站；主耦合器port3→1、port1→9、port2→17。左右归属依据已确认的
  X2左/X3右接线、现场用户观察及扫描分支，不能仅凭电机identity判断（序列号均为0）。
- 缺陷回归先在旧配置失败（1..7不等于9..15），修正后完整组合器与EtherCAT Python测试
  `python3 -m pytest src/rt_control/rt_control_composer/test src/rt_control/robot_hw_ethercat/test -q`：
  282 passed。初次运行有22项因缺少xacro命令失败；将既有提取依赖的bin目录加入PATH后通过，
  未修改测试断言。
- 右臂回归确认9..15、port1→9和仅缓存查询，并拒绝数量9、错误分支、错误身份及已激活主站。
- 工控机隔离overlay的普通/原位scope × real/mock四组合产物往返通过。
  将本次扫描文本显式转换为停机后的inactive/PREOP预期状态用于离线回放，验证分支与identity
  一致且无SDO；这不是实际停机或新栈通信验证。
- `tools/quality_gate.sh`：248 passed、13 skipped；门禁覆盖率86%。
  本地缺少ruff、clang-format、shellcheck，门禁已提示，未声称执行这些检查。
- `bash tools/check_ecat_sync_shutdown_policy.sh`通过。
- 工控机 `colcon build --base-paths robot_driver/src --merge-install --symlink-install
  --packages-select robot_hw_ethercat rt_control_machine_config rt_control_composer rt_control_bringup`：
  4包通过；未重启现有进程。
- 同包选择的 `colcon test --base-paths robot_driver/src --merge-install`：EtherCAT报告
  117 tests / 0 errors / 0 failures，bringup报告51 tests / 0 errors / 0 failures。
  composer报告215 tests / 0 errors / 8 failures，包含6项模型XML快照断言失败及2个外层测试组失败。
  在临时副本恢复修正前的6个源文件后，与修正后分别重跑10个快照用例，输出文件哈希和
  失败集合完全相同：4个arm、2个head快照断言均为既有失败，另4项通过。差异包含现有模型
  限位、dynamics和关节origin与旧快照不符；未改模型或放宽断言，不声称远端全量测试通过。
  首次未指定base-paths时因候选目录内同名包而未运行，指定正式源码目录后才得到上述结果。
- 安装后的普通/原位scope × real/mock四组产物在临时目录生成并通过 `load_artifact` 往返，
  真实配置的XML位置均为9..15，只有右七轴控制接口；没有启动这些产物。
- 工作目录验证日志保存在 `/tmp/right-arm-correction-20261026/`；临时依赖未进入仓库默认配置。

尚未验证修正后的实机启动、18121右臂反馈、使能或运动。先前采得295帧并与API数值吻合的
反馈实际来自左臂，撤回其“右臂读值已验证”的结论。

最新只读检查发现目标机在准备过程中被外部切换：活动右臂产物已变为9..15，哈希不再等于
本次安装时备份，但实际唯一 `ros2_control_node`（检查时PID599175）的启动参数来自
`candidates/la77-right-wrench-bench-sm-20261024/artifact`，硬件system为 `bluepoint_raw_bench`，
无关节、只含主站健康与右腕力传感器。18121公开页面连接失败，未发现操作台进程。
因此磁盘上的右臂产物不能作为“右臂scope已运行”证据，也不再声称旧错误栈仍在运行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本次实扫为18从站：0主耦合器、1..7左臂、8左蓝点、9..15右臂、16右蓝点、17内部子设备。
- F2: X2/X3是耦合器物理接线端口；当前X2对应ESC port3→1，X3对应ESC port1→9。
  EtherCAT位置随实际接线而定，不是固定的左右臂编号。
- F3: 当前右臂scope应将9..15绑定到 `right_joint1..7`；关节命名不能替代物理拓扑证据。
- F4: 修正源码与产物离线验证不改变已运行栈；需要有序停止旧栈并重新生成产物后启动，
  再只读核对反馈才能确认18121已经使用右臂数据。

## 遗留

后续用户已明确授权并完成退出传感器台架、右臂scope启动和实时反馈核对，见
[运行反馈验证](2026-10-26-right-arm-scope-runtime-verification.md)。以下保留本次纠错准备结束时的边界。

远端6项既有模型快照断言失败另行处理，不能将本次验证写成全量通过。当前传感器台架
占用总线，切换至右臂scope需依根契约第9节和域契约3.7取得单独明确授权；先有序退出
当前台架，再启动右臂scope并只读核对拓扑与未使能反馈，不自动复位、使能或下发运动。
本轮未提交或推送Git。
