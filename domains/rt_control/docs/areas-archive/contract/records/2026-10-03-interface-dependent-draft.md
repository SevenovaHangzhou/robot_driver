---
id: contract-20261003-01
area: contract
title: ELECTRI-174 依赖接口候选的 driver Draft PR
date: 2026-10-03
type: corrective
trigger: ELECTRI-174，用户要求接口仍在评审时推进 robot_driver PR
commits:
  - feature/electri-174-rt-interface-draft
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-152, BQ-153, ELECTRI-174]
---

## 背景

用户授权在接口评审尚未结束时推进 driver PR。此分支以 GitHub main@2234eef 为基线，保留前一工作树，
将 V3 接口实现增量整理为可构建的 Draft。Draft 用于联合评审，不能视为完整 Spec 实现或发布候选。
本文纠正 2026-10-02 记录和对话中“软件基本完成、主要只缺实物”的过度完成表述。

## 改动

- `deps.repos`／`src/interfaces/source-lock.yaml` 临时固定已推送的接口候选
  `f18caab1d6c94ff17584a470131786fb19f21562`（contract 1.0.0），同步 V3 pin 校验。
  这是 GitHub robot_interfaces PR #11 的 head，尚未批准或发布。
- Gitea robot_interfaces PR #12 的 head `53c382c5a21ad909b66575b46d95667327833517`
  与此候选的 Git tree 一致，但不能因此声称两个 commit SHA 相同。
- 保留泵阀、结构化 IO／传感器与位置动作草稿供审查；位置 adapter 的 `configured=true` 和
  触边运行节点的 `protection_enabled=true` 明确拒绝。底层 stop/hold、writer 和保护接线不是
  填完硬件参数即可自动完成，必须先实现并验证。
- 接口组合 launch 仅支持无硬件预览；真实模式和超声波采集请求直接拒绝。
  未配置的位置资源不创建命令 publisher，避免预览 graph 出现已退役的头部命令端点。
- 新增跨进程预览测试：状态发现、泵未配置拒绝、头部 Goal 未配置拒绝、无旧命令 publisher、
  所有测试进程有序退出。只启动本地适配器，不启动设备节点。

## 验证

- 14 个包（公共接口全包、域内接口、control_api_adapter、plc_io_modbus、plc_node、
  modbus_tcp_rtu485、lpms_nav3_can、bms_node、robot_description）干净构建 PASS。
- 对应构建目录 `colcon test-result` 最终汇总：161 tests，0 errors／failures（包含 CTest／pytest 汇总条目，不作为独立场景数）。
- `rt_control_bringup` 隔离构建 PASS，结果汇总 106 tests，0 errors／failures；不声称完成真实控制器闭包。
- 新增跨进程预览测试 PASS：消息与服务发现、未配置命令拒绝、无旧命令 publisher 和退出；不将发现／拒绝测试当作运动闭环。
- `tools/quality_gate.sh` PASS：207 passed、12 skipped，门禁覆盖率 83%；本机无 ShellCheck，CI仍需执行。
- 新依赖只通过 `deps.repos` 导入，`src/vendor/` 加入忽略规则，提交不包含公共接口副本。
- 整机 EtherCAT／全部控制器闭包、目标机、跨域客户端迁移和实物测试未执行；本机 EtherLab
  与 ros2_control_test_assets 缺失仍为环境限制，不用修改门禁／伪造依赖绕过。
- 所有硬件写操作均未执行。PR 验证只采用无设备程序和合成输入。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 接口 PR 的不可变 head SHA 可以用于依赖型 Draft 的可复现构建；不能用于声明正式发布。
  依赖批准合并后需统一最终 SHA、复查变化并重跑验证，再考虑转为正式 PR。
- F2: 当前 driver 是部分实现，以下软件项尚未闭合；禁止将它们全部归因于缺少实物参数。
- F3: Gitea driver main@946ec11 落后于本分支起点，缺少 GitHub 已合并的 Tier A (#48) 和
  金凤凰 BMS (#50)。Gitea 对应 Draft 明确包含这两个前置提交，不能把它们混作本次接口新增。

## 遗留：合并前软件检查清单

- [ ] 位置 Action：真正的执行器 backend、受管启停和取消/故障停止确认；新样本计数、速度未知时
  不能假报停稳、逐轴限位、原子 Goal 准入、停止中／故障时的资源释放与异常收尾。
- [ ] 头部旧命令入口：真实 controller 的唯一写入与旧入口退役；当前 preview 无 publisher 不代表
  已修复独立头部 runtime。PP／升降配置不是已投入运行的控制器。
- [ ] 触边：DI 到锁存核心、全机 stop 执行、停止反馈、复位权限及重启持久化的运行接线。
  目前只有纯逻辑测试和“未安装”状态，不具备实际全机保护。
- [ ] 按能力 DomainReadiness：目前 module 状态不能替代旧全局 readiness；必需依赖、质量错误、
  多能力晚订阅快照仍需完整实现与测试。
- [ ] 传感器：六维力公共质量状态与运行组合、IMU freshness 配置、红外通道清单；
  超声波故障通知及时性及消息时间／逐路过期在慢链路下的验证。
- [ ] 真空：泵与阀事务快照的原子性、未知结果只由更新的回读解除、逐通道真实时效、
  取消竞争与异常时占用清理、输出观测序号／时间、跨进程分侧实际执行测试。
- [ ] 整机 joint_states／TF：完整运行组合与关节覆盖测试，一套 RSP 开关并不等于整机完成。
- [ ] 安装包、完整控制器依赖闭包与跨域同 SHA 验证；移除或修订与当前代码不符的完成声明。
- [ ] 依赖接口 PR 批准／合并，统一最终 source-lock；Gitea driver 的前置基线同步。

实物继续按 BQ-152 分批补：真空 → PP/升降/头部 → IMU/六维力；触边安装后启用，换电与
RT 闭环力控另立后续工作。以上硬件缺口与软件清单分开追踪。
