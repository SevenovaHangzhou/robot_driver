---
id: motion-20260925-01
area: motion
title: ELECTRI-136 接收新版机械惯性初值并分离正式模型迁移
date: 2026-09-25
type: investigation
trigger: ELECTRI-136；用户接受机械近似参数作为标定初值并要求导入新版参数
commits: [feature/ELECTRI-136-gravity-ff, feature/electri-136-inertial-import]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-152, MECHINE-35, motion-20260924-01, motion-20260924-03]
---

## 背景

用户确认第一版只实现机械臂＋夹爪自重补偿；机械质量、质心可以作为近似初值，后续在台架上
用多姿态静态数据标定。接受该来源为初值，不等于确认左右真实质量分布或模型坐标对应关系。
本轮执行可确定的候选数据接收，并核实正式运行模型导入的前提。

## 改动

公共模型权威仓库为 `https://github.com/SevenovaHangzhou/robot_description.git`。
创建独立工作树 `repos/features/ELECTRI-136-description-inertial`，分支
`feature/electri-136-inertial-import`，基于公共 `robot_v3_suction_chassis` 的
`ec69ca04297896c1296720324d23cdb8f80d9e63`。未编辑已有模型维护工作树，未提交或推送。

- 新增 `model_sources/arm_inertial_20260924/`，逐字节接收 4 个 JSON/URDF 文件和
  24 个 STL，共 28 个源文件、493981 字节；原始 URDF SHA-256 仍为
  `abdabe00eb1f79761fbe5603c4dff5e6b473e2f9429c36d0e436c9ed3f60942e`。
- `manifest.json` 保存全部源文件 SHA-256、任务号、来源、未知修订、两个模型基线及
  `accepted_as_initial_estimate=true`、`hardware_verified=false`。
- `inertial_candidates.json` 提取 14 臂 link 和 2 hand 的原始质量/COM/惯量及源 part 身份。
  CAD COM 用 mm，source link COM 用 m，惯量 kg·m²；不做推测性的左右镜像修正。
- 每侧七连杆合计 43.146kg，包含 hand 为 43.232kg。目标 link/刚体变换尚未填写，
  `runtime_ready=false`；来源目录不属于 CMake 安装目录。
- 新增来源 README，并在公共模型根 README 增加入口。URDF/Xacro、运动学、限位、默认姿态、
  SRDF、两种末端、运行 meshes 和 RT-Control source-lock 均未修改。

**尚未完成正式运行模型惯性字段替换。** RT-Control 锁定
`17f5bdc46b8f2580ee81aed919da7b404da3bdaf`；公共模型已推进到 V3.1.1 的 `ec69ca0`，
包含 link 更名、双臂机械零位、部分运动学、Updown 零位/范围、命名姿态以及相机等其他资产变化。
此次公共分支更新不能作为一次惯性参数导入自动覆盖到驱动。

新机械导出也不是公共模型的同坐标副本：其关节为 `*_linkN_joint`，限位统一 ±1.57rad；
局部原点不同，且对应网格不全等。需要确定最终消费模型及 source→target link 的变换依据，
才能应用 `c_target=R*c_source+t` 和质心惯量 `I_target=R*I_source*R^T`。
这一前提是坐标与版本问题，不是要求机械把参数精确到实测水平。

## 验证

- 逐一对照原导出与候选目录的 SHA-256：28/28 相同，PASS。
- 检查 16 条提取记录与原 URDF 一致、质量和、惯量正定及三角不等式：PASS。
- 对比权威任务工作树 HEAD，确认所有已跟踪 `urdf/srdf/config/meshes/CMakeLists.txt/package.xml`
  原文未变：PASS。
- `check_urdf <候选来源>/robot.urdf`：PASS，18 links、17 joints。
- source ROS Humble 后，以独立 `.runtime/audits/ELECTRI-136-model-import/{build,install,log}`
  目录执行 `colcon build --symlink-install --packages-select robot_description`：PASS。
- source 本次安装 overlay 后执行 `colcon test --packages-select robot_description` 和
  `colcon test-result --verbose`：37 tests、0 errors、0 failures、0 skipped，PASS。
  这些检查验证公共基线和候选来源隔离，不证明已用新惯性值构建运行模型。
- RT-Control `tools/quality_gate.sh`：PASS，286 passed、13 skipped，门禁覆盖率83%；
  本机没有 ShellCheck，保留 CI 强制检查提示。公共模型仓库没有该脚本，以其现有包构建和测试验证。
- 两个工作树均执行 `git status --short --branch`、`git diff --check`、`git diff --stat`、
  `git diff`、`git diff --cached --check`、`git diff --cached`；原始候选逐文件哈希校验，
  新 manifest/候选内容与原 XML 交叉检查，新增文档全文审查。暂存区均为空，保留已有改动。
- 架构：公共模型改动放在其独立任务工作树，域边界/依赖方向 PASS；运行模型/接口/硬件行为未变，
  无生成物或无关文件纳入交付，未提交、推送或创建 PR。
- 无 Pinocchio 新参数运行接线、Mock active、PDO 写入、使能或运动；台架标定仍未进行。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 用户接受本批原始质量/COM/惯量作为未标定 CAD 初值，允许继续离线工作，保留已记录的
  左右质量属性变换疑点；不擅自选取镜像推导值覆盖来源。
- F2: 新参数已进入公共模型独立任务分支的候选来源，尚未替换运行模型；目标 link 映射为空，
  runtime/hardware 验证标记保持 false。
- F3: 当前驱动旧版与公共 V3.1.1 的坐标/语义不同。需先确定目标模型，再完成源到目标的坐标映射；
  模型升级及消费者迁移不能隐含在惯性字段更新中。

## 遗留

待确认本次正式导入的目标：保持 RT-Control 锁定旧模型，或以公共 V3.1.1 为目标并另做相应
消费者迁移。用户随后选择公共 V3.1.1；该选择由后续记录 `motion-20260925-02` 闭合。
两条路径都需有依据的 source→target link 刚体变换；目前没有把猜测值写入运行模型。
台架装配后仍需确认关节实际零位、0x6077/0x60B2 单位与方向、PDO/抱闸时序，再执行自重标定与
带载提起/卸载对比。模型原始数据接收不解除 BQ-152 的实机准入条件。
