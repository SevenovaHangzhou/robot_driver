---
id: motion-20260925-02
area: motion
title: ELECTRI-136 按用户选择迁移公共 V3.1.1 模型基线
date: 2026-09-25
type: feature
trigger: ELECTRI-136；用户选择“迁移公共V3.1.1”
commits: [feature/ELECTRI-136-gravity-ff]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-152, motion-20260925-01, governance-20260925-01]
---

## 背景

用户明确选择把本次惯性初值的目标设为公共 V3.1.1，并同步推进驱动模型基线与相关配置迁移。
本事件完成已发布公共模型的源码同步；机械新初值的 source→target 坐标变换仍需独立取证。

## 改动

- 由独立 `robot_description` 仓库的完整提交
  `ec69ca04297896c1296720324d23cdb8f80d9e63` 导入 195 个文件，tree 为
  `864dfff98fdd4cf54df35095232b38728c073c01`。导入前确认目标包无用户修改；
  用原始 Git archive 同步，移除被 `robot_v3_1_1.xacro` 替代的旧 `robot_v3_0_9.xacro`。
- 同步 `src/description/source-lock.yaml`、`alfa_v3.yaml` 模型绑定、
  `machine_profile.py` 的冻结版本校验、V3 branch gate 和相关测试；不修改公共模型原文。
- 采用已发布模型的 `left/right_link1..7`、机械原始零位和各命名姿态；控制关节名仍为
  `left/right_joint1..7`。Updown 改为最高点零位 `[-1,0]m`，旧坐标等于新坐标加0.5m。
  原来左右某些轴的逻辑零位偏置不能直接沿用；编码器/模型零位匹配仍待台架标定。
- 公共提交同时带来 head/chest 相机、base_footprint、部分机械臂运动学及描述参数更新；
  这些是所选完整公共提交的内容，不是私有拼接版本。默认仍选 gripper 变体用于 V3 硬件绑定。
- 不更改 EtherCAT 位置比例/偏置、PDO、总线和 enable 策略；机器配置仍 draft，默认 validation-only，
  gravity controller 仍未接启动面，模型及力矩换算 verified=false。
- 候选来源 manifest 的目标版本已更新为 ec69ca0，另提供 `mapping_request.json`，列出16个
  source/proposed-target link 及两侧父坐标中的原点，明确刚体变换尚为空。两侧父坐标不同，
  不能直接相减原点数值；也未把43.146kg/臂的机械新值写入 ec69ca0 的运行模型。

## 验证

- 将驱动模型副本逐文件与公共 ec69ca0 的 Git blob 比较：195/195 一致，PASS。
- 在独立 `.runtime/audits/ELECTRI-136-model-import/driver-{build,install,log}` 目录，
  source ROS Humble 后执行 `colcon build --symlink-install --packages-select robot_description rt_control_bringup`：
  两包 PASS。source 本次 overlay 后执行对应 `colcon test` 和 `colcon test-result --verbose`：
  115 tests、0 errors、0 failures、0 skipped，PASS。
- 用本次 overlay 展开 `robot_dual_gripper.urdf.xacro`，`check_urdf`：PASS。
  包测试同时覆盖两种末端、TF/零位、底盘/头部、V3.1.1 来源、命名姿态和 profile validation。
- 定向 `pytest`：`robot_hw_ethercat/test/test_gravity_ff_drafts.py`、
  `test_ethercat_hardware_variant.py`、`gravity_ff_controller/test/test_config.py`：43 passed。
  首次命令把控制器测试名写成不存在的 `test_draft_config.py`，未收集测试；更正后上述43项通过。
- `python3 tools/check_v3_branch_contract.py`：PASS。
- 迁移后最初仓库门禁因上游8份哈希冻结原件无末尾换行失败；通过
  [精确哈希限定的格式兼容](../../governance/records/2026-09-25-description-snapshot-byte-preservation.md)
  解决，未改源文件。最终 `tools/quality_gate.sh`：288 passed、13 skipped，门禁覆盖率84%，PASS；
  本机无 ShellCheck，保留既有 CI 强制检查提示。
- 搜索运行源码的旧 link frame、旧 Xacro 和旧 pin：运行路径已迁移；动力学单测故意保留
  link/joint 同名的合成 fixture，继续验证旧模型兼容性。

- 两工作树完成 `git status --short --branch`、`git diff --check`、`git diff --stat`、
  `git diff`、`git diff --cached --check`、`git diff --cached`；新增来源逐文件哈希及记录模板/链接
  校验通过。修改的 Python 文件 `py_compile` 通过。保留本轮此前所有审查文档与候选数据。
- 架构自审：模型权威仓库及完整版本身份保留，依赖方向 PASS；公共坐标/模型兼容性为已授权迁移，
  外部消费者联合验证未完成；Docker/设备权限和实时控制路径无变更。无无关修改、构建生成物或
  敏感信息纳入交付；没有提交/推送授权，暂存区为空。

所有操作仅针对源码、离线模型和包测试。未重建未改动的 EtherCAT/动力学 C++ 插件：宿主缺少
其冻结上游依赖和 Pinocchio；未运行这些插件加载新模型的 Mock、外部 Motion/TF 消费者联合测试、
实际编码器零位验证或实机。模型接口迁移因此不能称为完整系统/发布验收。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本任务工作树现在固定公共 V3.1.1 ec69ca0；其源码原文、source-lock 和机器模型绑定一致。
- F2: 用户已裁决目标模型版本，取代“尚待选择旧版还是V3.1.1”的状态；实机坐标匹配和外部
  消费者联合验证仍开放，运行准入未改变。
- F3: 机械新质量/COM/惯量仍是独立候选来源，不能把公共模型升级冒充新惯性参数已完成挂接。
  source→target link 变换及部件归属是下一项输入，不再阻塞公共基线迁移本身。

## 遗留

由机械/模型所有者提供本次导出到 V3.1.1 link 坐标的变换，或直接按 V3.1.1 link 坐标重导惯性参数；
用户已选择“坐标转换后合入”，明确不授权同名 link 原样复制、目测配准或猜测变换。
数据精度可保持 CAD 初值，不要求先达到实测精度。完成后先在独立 robot_description 任务分支
更新对应惯性字段，再形成可追溯版本供 RT-Control 和其他消费者同步。
完整消费者验证需补齐 C++ 依赖环境及外部 Motion/TF 配套，台架装配后继续闭合单位、零位、
抱闸时序及自重标定。用户已授权提交和推送本轮可验证的基线/候选来源；推送不代表惯性挂接完成。
