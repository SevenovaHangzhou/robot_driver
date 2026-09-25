---
id: motion-20260925-03
area: motion
title: ELECTRI-136 新机械惯性初值按授权临时FK映射合入V3.1.1
date: 2026-09-25
type: feature
trigger: ELECTRI-136；用户要求合入新机械参数并选择“采用FK临时映射”
commits: [feature/ELECTRI-136-gravity-ff, 11f6d906dcb4cd5abba7cd4693afc9bb34c6a81e]
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-152, motion-20260925-01, motion-20260925-02]
---

## 背景

用户明确要求把新机械惯性数据写入运行模型，并授权一种可复现但未实机验证的临时映射：
左右臂分别对齐对应 J1/link1 零位 frame，再使用 source/target URDF 的零位 FK 求每个 link 的
坐标变换。该授权取代“必须等待机械给出16个变换才能做初值导入”的工作阻塞，不代表机械坐标
或惯性参数已经通过实测。

## 改动

独立 robot_description 分支 `feature/electri-136-inertial-import` 提交
`11f6d906dcb4cd5abba7cd4693afc9bb34c6a81e`（tree `ff4bddfc25e56b88b85b124c7825038664922021`），
其完整内容同步到 RT-Control 构建副本，source-lock、机器模型绑定和校验同步。

映射定义（每侧独立）：

```text
A = T_target(link1, q=0) * inverse(T_source(link1, q=0))
T_target_link_from_source_link = inverse(T_target(link, q=0)) * A * T_source(link, q=0)
c_target = R*c_source + t
I_target_COM = R*I_source_COM*R^T
```

- 输入 source 原始 URDF SHA-256：`abdabe00eb1f79761fbe5603c4dff5e6b473e2f9429c36d0e436c9ed3f60942e`。
- 目标为公共 V3.1.1 ec69ca0 的夹爪展开模型，SHA-256：
  `8ad2533523a88b7b87f6768ec0042f97283db2fa6f6afb3a743c322f3e97b826`。
- 16 个矩阵、变换前后 COM、目标惯量、方法及授权写入
  `model_sources/arm_inertial_20260924/mapping_result.json`；manifest 锁定其 SHA-256。
- 左右 link1..6 更新质量/COM/惯量；夹爪宏按 side 分别使用转换后的 link7 和 moving_jaw 数据。
  单臂七连杆合计43.146kg，含活动夹爪43.232kg。吸盘末端的2.618kg及原COM/惯量不变。
- V3.1.1 关节名称、父子关系、零位、轴向、限位、Tool0、mesh、collision、Updown语义不变。
- 坐标映射和硬件验证标记仍为false；CAD初值不能替代台架标定，gravity active双门禁仍关闭。

## 验证

- robot_description构建通过；最终44 tests、0 errors、0 failures、0 skipped。
- 新测试覆盖16条映射身份、源哈希、正交旋转/det=1、J1单位对齐、质量/COM/惯量输出、
  正定性/三角不等式及吸盘末端未被覆盖。另使用已有独立FK实现重算每侧锚点对齐，
  在共同参考系对照实际展开URDF与原文件的COM/惯量，误差小于1e-10；不只与结果JSON互相比较。
- 首轮旧测试仍断言夹爪4.750/0.342kg而失败，已随本次明确变更更新为4.968/0.086kg，重跑全过。
- 两种末端均展开并通过check_urdf；去掉所有inertial块后，与ec69ca0两版展开模型逐结构一致，
  确认没有运动学/几何变化。
- 驱动副本、版本锁、机器绑定及校验固定到模型feature分支的完整提交11f6d90；模型和bringup
  两包构建通过，122 tests、0 errors、0 failures、0 skipped。相关前馈草案/硬件配置43 passed。
- `tools/quality_gate.sh`：288 passed、13 skipped，覆盖率84%；V3 branch contract通过。
  新导入4份原始CAD文件同样没有末尾换行，按完整路径和SHA-256登记，不改原文；总计12份。
- 已重新从相对Xacro路径生成发布URDF，消除首次展开时带入的本机绝对路径注释。
- 未完成Pinocchio/C++插件加载新模型的Mock或数值回放，宿主缺少冻结依赖和Pinocchio。
  上述结果为模型/配置及离线转换验证，不是动力学插件或实机验收。

- 驱动模型副本与独立模型提交11f6d90逐文件比较，229/229一致；28份原文件哈希未变。
- Git自审：逐项检查工作树/暂存区差异、来源和测试；无无关修改，生成的URDF以相对路径展开，
  无新增本机绝对路径依赖。公共模型所有权和域依赖方向保持，控制接口/运动学/设备权限未改。
  Docker封装N/A，完整外部消费者和实机仍未验证；用户此前提交/推送授权持续有效。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 新机械质量、COM、惯量已按用户授权的临时FK规则写入公共模型分支并同步驱动副本。
- F2: 临时映射是CAD初值而非机械权威变换，保留来源和矩阵以便回退/重新标定。
- F3: 运行URDF现在含新惯性；实机准入仍未开放，第一版范围仍为机械臂＋夹爪自重。

## 遗留

台架需核对实际零位、模组/夹爪质量归属、左右COM疑点、力矩单位和方向、抱闸时序，
再进行多姿态静态标定、影子运行和分级前馈投入。临时映射不保证两个不同机械模型在所有姿态
下动力学等价，尤其不能把零位下的坐标对齐视为实物装配已确认。
