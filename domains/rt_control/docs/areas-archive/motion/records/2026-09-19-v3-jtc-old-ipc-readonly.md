---
id: motion-20260919-02
area: motion
title: V3 JTC-only 旧工控机旁路与原位使能验证
date: 2026-09-19
type: commissioning
trigger: "用户指定 ar@192.168.100.40 做十四轴 JTC 联调，并补充 0x730F 厂家解释及临时方向值"
commits: ["feature/v3-jtc-ar-ipc"]
env: native
risk: T3
writes: { reset: yes, enable: yes, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: ["motion-20260919-01", "release-deploy-20260918-01", "ecat-axes-20260912-11"]
---

## 背景

V3 已包含 14 轴 FJT 和 rolling 软件，但旧工控机的原旁路 release 仍为 `1256885`，
不含运动 runtime。用户此次目标仅为通过 JTC 控制左右 7+7 个 CSP 轴，并确认
机械零位对应 URDF 零位；每轴真实正方向尚待慢速测试，用户先指定 `+1` 为试验初值。

## 改动

- 在 `v3@df55bdc` 的独立本地分支增加显式 `rt_control_start --arm-jtc`，保持默认
  validation 和原 `--arm-runtime` 不变。JTC-only 不配置/加载 rolling，不复制其临时
  envelope；`enable_manager` 用现有非模式切换路径，在使能完成后激活唯一的 14 轴 JTC。
- JTC-only 在控制器与 enable_manager 加载成功后启动现有公共
  `control_enable_adapter`；未增加绕过启停服务的 Motion 旁路。真实运行可指定独立
  `calibration_file`，默认仓库草稿仍是 `verified: false`，缺少逐轴方向时拒绝启动。
- 针对本轮发现的分支对调增加独立 `branch_mapping_verified` 启动门禁；只有逐轴
  接线/拓扑复核后才能将该字段置 true，单改 `verified` 不会放开真实硬件。
- 对真实配置逐轴核对 joint 对应的 ring position 与已确认机械零位计数。命令换算为
  `target_counts = zero_counts + direction * rad * 524288 / (2*pi)`，反馈为其逆变换。
- 用户指定十四个 `direction` 暂时全部写为 `+1`，供后续逐轴慢速方向试验；
  配置标记 `direction_status: user_provisional_plus_one`，保留
  `verified: false`。调换后 16 轴拓扑、ring 与零位证据已复核，因此
  `branch_mapping_verified: true`；此配置不得解释为已经完成
  机械正方向验证，真实 JTC builder 仍拒绝未验证校准。
- 以不切换 `/home/ar/rt-control-current` 的方式在旧工控机部署旁路候选到
  `/home/ar/rt-control-v3-jtc/releases/v3-df55bdc-jtc-candidate`；从精确 V3 基线 bundle
  导入四个冻结 vendor 仓库并应用冻结补丁。候选源码同步自
  `feature/v3-jtc-ar-ipc`，但未切换 current，不能当作生产 release。

## 验证

- 本机质量门禁：292 passed / 13 skipped；本机受影响包构建通过，bringup 7 组
  CTest 通过；新增测试覆盖 JTC-only 不加载 rolling、公共使能适配器、真实模式拒绝未
  验证方向以及零位计数一致性。
- 最终提交前本机新增专项回归 37 passed；尝试从本机空前缀构建完整硬件闭包时，
  因该工作树未导入 vendor `ethercat_generic_slave` 而在 CMake 配置阶段中止。
  同一源码已在目标机完整冻结 vendor 闭包中成功构建，完整闭包继续由 PR CI 验证。
- 目标机 `ar-Default-string`、`5.15.0-1032-realtime`、隔离 CPU14；候选冻结 vendor
  补丁准备通过，20 packages build PASS；质量门禁 292 passed / 13 skipped。
- 目标机初轮 `colcon test --merge-install --base-paths ...`：bringup + 公共适配器
  130 tests、0 failures/errors。增加独立分支映射门禁与严格整数方向检查后，
  重建硬件包和 bringup，复跑得到 228 tests、0 failures/errors。
  首次测试命令误从 `/home/ar` 扫描无关目录，已停止该命令并在候选目录重跑通过。
- EtherCAT master 0 为 Idle / Active no / Link UP，18 从站均为 PREOP。只读 SDO
  `0x6064` 发现 ring 1..8 的当前位置与历史 ring 9..16 机械零位逐轴相符，
  ring 9..16 与历史 ring 1..8 零位相符，全部 16 个差值绝对值不超过 10 counts：

  | 当前 ring | 当前 6064 | 原记录对侧 ring / 零位 | 差值 |
  | ---: | ---: | --- | ---: |
  | 1 | 252115 | 9 / 252125 | -10 |
  | 4 | 465957 | 12 / 465961 | -4 |
  | 8 | 292161 | 16 / 292159 | +2 |
  | 9 | 21618 | 1 / 21613 | +5 |
  | 12 | 212362 | 4 / 212362 | 0 |
  | 16 | 222143 | 8 / 222142 | +1 |

- 目标机隔离 ROS domain 216 的 Mock JTC-only 启动通过：`whole_body_jtc` inactive，
  broadcaster 与 enable_manager active，公共 `/control/set_enabled` 服务、
  `/whole_body_jtc/follow_joint_trajectory` action 和 14 轴 `/joint_states` 可见。
  Mock 公共使能返回 `enable_batch_timeout`（GenericSystem statusword 保持 `0x0040`），
  因此没有宣称通过“使能后完整 FJT”链路；限时退出后模拟硬件已失能。
  首次 Mock 限时退出时 Python 适配器出现 ROS 上下文已关闭后的退出码 1，第二次
  Mock 退出适配器为 exit 0；需在长期运行测试中关注停机竞争。
- 用户调换现场分支接线后，EtherCAT Link UP、18 从站均 PREOP；16 个 `0x6064`
  逐轴重新对应本侧历史零位，最大偏差 11 counts。只读
  `verify_stationary_bus()` 核查 coupler X2/X3 下游、全部从站身份、PREOP、控制字 0
  与仅 J3/J4 的已知 `0x730F` 均通过；用户已确认现场安全并持有急停。
- 首次尝试真实无运动入口因只传了 EtherCAT 启动 CPU、未传实时优先级而拒绝硬件
  activation；进程退出后总线 Idle。补充目标机已验收的 CPU14/FIFO80 后，
  硬件 OP、16 个电机 OP、工作计数完整，末端分支器保持 PREOP，250 Hz manager
  FIFO80 成功；真实入口始终不导出 position command interface。
- ring 3 和 12 在 OP 后出现归档过的 `0x730F` Fault。第一次 `/rt/reset_fault`
  返回 `operation_in_progress`，未复位；诊断进入 `fault_requires_reset` 后再次调用
  返回 success，全部 16 轴错误码为 0、状态均为 `0x0250`。

- 一次 `/rt/enable` 返回 success；16 轴状态均为 `0x1637`，一次完整读回
  `|0x607A-0x6064|` 最大 6 counts，PP 未下发新目标。紧接着显式 `/rt/disable`
  返回 success；16 轴错误码均为 0，状态字 `0x0250` 或 `0x1250`，均不在
  OperationEnabled。限时退出再次确认 `already_disabled`、hardware inactive；
  最终 master Idle/Inactive、18 从站均 PREOP、无控制进程，原 current link 未变。

本次真实原位使能基于用户本轮的实机验证授权、现场安全确认及手持急停；没有执行
JTC 轨迹、PP 开合或任何真实位置运动。

## 0x730F 原因与使能前处理

用户补充的故障成因：电机自身是多圈编码器，现场没有接电池，计划将其按单圈使用；
用户已与厂商确认，这种状态会报 `0x730F`，并要求每次使能前先重置故障。
本轮实测对应左 J3（ring 3）、右 J4（ring 12）。操作顺序为等 enable_manager
稳定后检查诊断，若为 `fault_requires_reset`，通过该控制器的 `/rt/reset_fault`
按受控流程复位，逐轴确认 Fault 退出且 `0x603F=0`，随后才能申请使能；
复位失败或出现其他错误码时停止联调。已有公共 `control_enable_adapter` 会在
`fault_requires_reset` 时尝试一次受控复位，不把没有故障的电机当作要写入的复位目标。
这只是用户转述的厂商原因，不代表已验证 PP 夹爪所需的多圈电池/参考条件。

## 结论与冻结事实

- F1: 调换前左右 8 轴分支与原零位档案对调；用户恢复接线后，当前 16 个
  ring/零位重新对应（偏差不超过 11 counts），真实无运动使能与有序失能已验证。
- F2: JTC-only 旁路源码与目标机离线构建通过，真实模式仍因校准草稿
  `verified: false` 而拒绝启动；分支映射已验证，14 个
  `direction: 1` 仅为试验假设，布尔 `true` 也不能冒充整数 `+1`。
- F3: 本轮没有修改旧工控机 current link；真实 OP 和 16 轴使能/保持/失能
  已完成，但没有执行任何 JTC/PP 位置轨迹。
- F4: 用户与厂商确认无电池的多圈编码器按单圈使用会引发 `0x730F`；
  本轮 J3/J4 故障经受控复位后才使能，之后每次使能前都需检查/复位并确认清零。
- F5: 十四轴 `+1` 是用户指定的慢速方向试验初值，不是已验证方向；
  草稿和真实 JTC 的 fail-closed 门禁保持区分。

## 遗留

分支拓扑已恢复。对十四轴 `+1` 逐轴慢速测试并记录与 URDF 正方向是否一致，
再结合逐轴限位/低速包络，生成独立校准文件（`verified: true`、
`branch_mapping_verified: true`）并逐关节验收。
公共 `/control/safety_state` 的 arms-only 生产者仍未在 JTC 入口完成；Mock
`enable_batch_timeout` 需要与真实使能行为区分。真实运动必须另行记录目标轴、位移、
速度、负载、急停与现场人员条件；本记录的原位使能验证不能升级为实机运动通过。
