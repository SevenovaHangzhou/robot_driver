# robot_description

> 本分支在 `feat/v3-analytic-proxy`（e189bdb）上加入研华底盘台架已部署的底盘修正（ELECTRI-127）。
> **当前底盘入口：`urdf/robot_analytic_proxy_mobile.urdf`**，不是下文保留的旧默认入口。
> `base_footprint → base_link` 为 `xyz=[0,0,0.32]`、`yaw=+π/2`；新车头 `+X` 对应原 `base_link −Y`。
> 四个 `steer*_joint` 的 origin yaw 为 `−π/2`，局部轴不变；转向 q 限位为 ±135°。
> 四个 `wheel*_joint` 为 continuous，速度上限 **31.4 rad/s**。
> 电机原码方向、标定零偏和总线配置属于控制部署，不包含在本模型快照中。
> 部署入口 SHA-256：`bba180d064ff4fe672b9b3fdfa7e8e8fd1bc9d1cac4f7cbc13c751ebd0fbff1c`。


`robot_v3_suction_chassis` 当前默认提供 V3.2.2 整机几何与惯量、V3.1.1
逻辑零位/方向/限位和地面 `base_footprint` 的生产描述。

## 默认模型

- 入口：`urdf/robot.urdf.xacro`
- 直接加载：`urdf/robot.urdf`
- SRDF：`srdf/robot.srdf`
- 初始位置：`config/initial_positions.yaml`
- 关节限位：`config/joint_limits.yaml`
- 命名姿态：`config/named_poses.yaml`

模型只有 17 个运动关节：Updown、头部 yaw/pitch 和双臂 14 轴。
V3.2.2 原始包没有舵轮、行走轮和主动悬挂关节，因此生产描述也不再声明这些
自由度。V3.2.2 原始模型不额外安装旧 V3.1.1 吸盘网格；左右 `tool0` 是直接
挂在 `link7` 局部 `+Z` 方向 `0.151m` 的无几何语义帧，不增加运动自由度。

## 根坐标

`base_footprint` 是唯一物理根和地面接触参考。`base_link` 位于其正上方：

```text
base_footprint -> base_link: xyz=[0, 0, 0.335]m, rpy=[0, 0, 0]
```

SRDF 使用 `map → base_footprint` planar virtual joint 表达底盘 `x/y/yaw`；
这些量不进入实体关节状态或关节轨迹控制器。

## 零位与限位

外观、碰撞网格和质量属性来自 V3.2.2。控制接口仍使用冻结的 V3.1.1
逻辑关节名和限位。为了让既有姿态数值直接复用，V3.2.2 原始零位被适配为：

- 左 J1：原始角 = 逻辑角 - 90°
- 右 J1：原始角 = 逻辑角 + 90°
- 左 J5：原始角 = 逻辑角 + 90°
- 右 J5：原始角 = 逻辑角 - 90°
- Updown：原始位置 = -逻辑位置

这些变换已经固化到 URDF joint origin/axis，下游继续发送 V3.1.1 逻辑角，不得
再重复施加偏移。

双臂限位保持：J1 ±180°、J2 ±105°、J3 ±180°、J4 ±145°、J5 ±180°、
J6 ±110°、J7 ±180°；Updown 为 `[-1,0]m`，`0m` 是最高点。

## 关键姿态

- 第一初始：Updown `-0.3m`，左臂 `[155,-105,20,90,-90,-40,0]°`，右臂取相反数。
- 第二初始：Updown `-0.6m`，左臂 `[30,80,20,90,90,60,0]°`，右臂取相反数。
- 第一放置：Updown `-0.3m`，左臂 `[140,-100,-180,20,-90,-20,0]°`，右臂取相反数。
- 第二放置：Updown `-0.6m`，左臂 `[0,100,0,-30,100,30,0]°`，右臂取相反数。

完整精度数值见 `config/named_poses.yaml`。

## 临时解析代理模型

需要闭式解析 IK 的仿真与规划任务可以显式加载：

```bash
xacro $(ros2 pkg prefix robot_description)/share/robot_description/urdf/robot_analytic_proxy.urdf.xacro
```

对应姿态文件为 `config/named_poses_analytic_proxy.yaml`，参数清单为
`config/analytic_proxy_manifest.json`。该模型以 V3.2.2 生产描述为基础，仅理想化
左右 J3/J4/J5/J7 的毫米级装配原点，使肩、肘、腕轴组满足解析 IK 的共点假设；
网格、惯量、关节方向、限位、`base_footprint` 和 `tool0=151mm` 均保持不变。

该入口仅供临时算法开发和仿真，不是真机标定结果，也不会替换默认
`urdf/robot.urdf.xacro`。实机与正式交付必须继续使用机械部门确认后的生产模型。

## 四舵轮与 IMU 整机代理入口

`urdf/robot_analytic_proxy_mobile.urdf` 在解析代理上身基础上，接入用户选定的
`local_munsrkg7_6rqaj1_urdf_stl` 新机械底座与四舵轮。它包含 25 个运动关节、
30 个 link，使用 `config/initial_positions_analytic_proxy_mobile.yaml` 与
`config/joint_limits_analytic_proxy_mobile.yaml`。

- 上身所有 link、joint（含机械臂八处代理原点、头部和 Tool0）相对 `base_link`
  保持原代理模型定义。
- `base_link` 的碰撞和惯量采用新机械来源；可视网格 `part_001_solid_001.stl`
  使用之前 V3.2.2 的带罩壳版本，以遮住轮子。该罩壳仅用于外观，不作为新增碰撞或质量。
  四组 `steer1..4` / `wheel1..4` 的几何、惯量、轴和旋转中心按源文件保留；关节类型、转向零位及限位按本页顶部的部署修正。
- `base_footprint → base_link` 为 `[0,0,0.32]m`、yaw `+π/2`，基于源轮心高度 -0.22m 与
  名义轮半径 0.10m；四轮网格最低点距地面约 0.34mm（网格离散误差）。
- `base_link → imu_link` 为 `[0.403,0,0]m`、RPY `[0,0,0]`，依据用户此前明确
  确认，且父坐标正是本次新机械底座的来源。IMU 只增加坐标系，不虚构外形和惯量。
- 本次选定的新机械底盘没有独立主动悬挂关节。

来源、原文件 SHA256 和 IMU 确认记录位于 `model_sources/mechanical_958c49d/`；
组合策略见 `config/analytic_proxy_mobile_manifest.json`。重新生成入口和配置：

```bash
python3 tools/build_analytic_proxy_mobile.py
```

这是供查看与规划使用的组合模型，机械臂仍为实验性解析代理；上身与新底座装配
需机械复核。IMU 设备 ENU/NED 输出约定不由该安装 TF 定义。原生产入口保持独立。

## Alfav3 双臂整机入口

**入口：`urdf/robot_alfav3_mobile.urdf`**，SRDF：`srdf/robot_alfav3_mobile.srdf`，
关节限位：`config/joint_limits_alfav3_mobile.yaml`，清单：`config/alfav3_mobile_manifest.json`。

- 双臂来自 `model_sources/alfav3_arms_1011_4`（SolidWorks 导出，已去掉中文包名并改为本仓库命名）。
  导出的 `base_link` 即 `arm_carriage` 躯干（42.8 kg vs 42.3 kg），只用于确定安装位置：
  Alfav3 `base_link → arm_carriage` 为 `xyz=[0.266, 0, 0.36515]m`、`rpy=[π, 0, 0]`，
  由两侧 J1 轴线拟合（残差 4.7 µm / 1.5 µrad）。
- 零位：Alfav3 导出零位 = 机械零位 = 驱动 2^18 计数零位。它对应旧入口的
  左臂 `[90, -15, 180, 0, 90, 0, 0]°`、右臂 `[-90, 15, -180, 0, -90, 0, 0]°`；
  相对旧入口，左 J6、右 J3/J4 轴向相反。旧入口的姿态数值不能直接用于新入口。
- 关节正方向 = 驱动编码器计数增大方向。左 J3、J6 的导出轴与此相反，入口里取反
  （2026-10-18 左臂台架实机确认）。右臂未经实机验证，保持导出轴。
- 限位以新零位为中心对称；effort/velocity 沿用旧入口，未经实机验证。
- `left|right_tool_link` 是导出的法兰件（link7 +Z 98.7 mm）；`left|right_tool0` 在 link7 +Z
  `98.7 + 151 mm` 处，姿态与 link7 相同，与旧入口 tool0 物理位姿一致。
- 视觉网格保持导出原始字节；碰撞网格为生成的包络（base_link 与 link1–5 用 CoACD 凸分解，
  link6/link7/tool_link 用单凸包，凸块外扩 1 mm），全部视觉顶点都在包络内。
- 质量与惯量来自导出，与旧入口差异较大，未经实机验证。
- 重新生成：`tools/import_alfav3_arms.py <导出目录>`（需 coacd/trimesh，开发工具），
  然后 `python3 tools/build_alfav3_mobile.py`。

## 来源与已知边界

V3.2.2 原始导出保存在 `model_sources/robot_v3_2_2/`，来源哈希见
`config/v3_2_2_source_manifest.json`。原始导出报告仍提示 16 个未分配 CAD part；
这些 part 未被原始 URDF 引用，也未被生产入口强行补入。

`urdf/robot_v3_2_2.urdf` 保留机械原始关节命名和通用限位，仅用于来源审查；
运行时必须使用默认生产入口。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
source install/setup.bash
colcon test --packages-select robot_description
colcon test-result --verbose
```

测试检查来源哈希、单树根坐标、17轴清单、V3.1.1限位、四套姿态到原始
V3.2.2几何的等价映射、Tool0固定链、SRDF引用和安装后mesh解析；代理模型
额外检查只修改约定原点、三组轴共点和第三放置位15轴合同。
