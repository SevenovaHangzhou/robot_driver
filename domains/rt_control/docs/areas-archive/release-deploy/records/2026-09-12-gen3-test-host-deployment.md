---
id: release-deploy-20260912-01
area: release-deploy
title: 三代机开发快照部署至用户指定测试主机
date: 2026-09-12
type: feature
trigger: ELECTRI-118，用户明确要求 SSH 部署，16 个电机稍后连接
commits: []
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-147, BQ-145, BQ-146, ecat-axes-20260912-02]
---

## 背景

用户要求先将当前开发版本部署到指定测试机，再连接双臂。网络初始不可达，恢复后完成隔离部署，
未改现有生产入口，也未启动真实硬件。此项是测试主机准备，不是完整生产版本发布。

## 改动

- 部署原有 main 基础 `308dbfca75c2fb5fec53241a6e6ead37dcd4f8fa` 加本轮未提交开发改动，
  没有在部署时合并远端后来新增的 28 个提交。
- 源码和四个 deps.repos 精确版本打包，按序应用全部窄补丁并验证；初始归档哈希为
  `da376bb5f1dd48d496c8bfc67e4ed40fbad78d3d6c63908415acb1ef8779ec5e`。
- 目标使用新建隔离目录 `electri118-gen3-20260912.wvvShj`；源码、install 和日志独立。
  添加部署校验、arms_only 静态校验和环境加载脚本，不添加自启动。
- 随后将用户提供的 GR10-EC-6SW ESI 和配置澄清同步到源码并安装，追加文件校验清单。

## 验证

- 目标 Ubuntu 22.04/ROS 2 Humble，30 个完整 RT-Control 依赖链包源码构建成功（4 min 42 s）。
- gripper_controllers、robot_hw_ethercat、rt_control_semantic_components、enable_manager、
  swerve_driver、robot_hw_canopen 六个相关包测试：282 条记录、0 错误/失败/跳过。
  最初缺少上游假硬件插件，仅在隔离目录解包官方 hardware_interface_testing 后复验通过。
- 44 项 profile/validator/launch 测试通过；10 个组合静态校验通过；安装版 arms_only launch
  输出 14 CSP + 2 PP、master 0、draft，无硬件节点创建。
- 归档/增量文件哈希核验与七个关键包 install prefix 检查通过；旧部署 symlink 和运行服务保持。
- 本地 quality_gate：214 项通过，门禁覆盖率 83%，ShellCheck 本地不可用。
- 主机 doctor 未通过：安装元数据缺少所要求的 IGH_PRESERVE_PDO_PATCH_SHA256。
  基础提交相同，但记录的是另一份 DC 补丁。未修改内核/驱动，未将其当作实机准入通过。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 开发快照已在测试机部署、构建和离线验证；不是只完成本地打包。
- F2: arms_only 仍为静态校验入口，16 个执行器不能等同于总线设备总数或已具备真实控制能力。
- F3: 实机启动前仍需完成 IgH 补丁准入、新分支器/电机拓扑与身份、PDO/标定和 runtime composition。

## 遗留

BQ-147/145/146；完整生产 Docker 交付未重建。主机代码未提交/推送，未访问真实总线、复位、使能或运动。
部署过程未保存 SSH 密码，现有工作区与服务未覆盖。
