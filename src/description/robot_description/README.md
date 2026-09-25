# robot_description

本分支 `robot_v3_suction_chassis` 默认提供“V3.1.1 双臂 + 吸盘 + 主动悬挂舵轮”版本（26 个可动关节）。
直接加载 `urdf/robot.urdf`；可编辑入口为 `urdf/robot.urdf.xacro`，
配套配置为 `config/initial_positions.yaml` 和 `config/joint_limits.yaml`。

## 1. 功能说明

本仓库提供机器人 URDF/Xacro、SRDF、网格、初始关节位置和关节限位资产。
双臂关节链和升降坐标采用 V3.1.1；模块化机身、主动悬挂底盘、双轴头部、
双夹爪/双吸盘和 Tool0 沿用 Kkozia 分支，两版均集成四组独立转向/行走舵轮。
头部相机、头部 MID-360 和胸部 ZED X 使用 2026-09-21 新 CAD 安装位置。

## 2. 输入输出

- 输入：`urdf/robot.urdf.xacro` 及其包含的 V3 几何与末端文件；
  `end_effector` 参数可取 `suction`（默认）或 `gripper`，同时选择两侧末端。
- 输出：展开后的 URDF、`srdf/robot.srdf`、初始位置和关节限位 YAML。
- `base_footprint` 是四个零位轮轴中心的平均 XY 位置，并沿 Z 下移 `0.1m`
  轮半径得到的地面旋转中心；`base_footprint → base_link` 为
  `[0.1900000028, -0.0000442724, 0.4000025060]m`。
- 按 ROS `base_link` 约定，`+Y` 一侧命名为 `left_*`，`-Y` 一侧命名为
  `right_*`。
- 左右 J1～J7 的关节原点、轴向和 J1～J6 连杆资产来自 V3.1.1；
  J7 机械末端由所选 Kkozia 夹爪/吸盘配置替换。机械原始零位已关于中轴面镜像，
  不再对右 J1/J5 叠加180°逻辑零位偏移；镜像姿态统一满足 `right_jointN=-left_jointN`。
- `updown=0m` 为最高点，逻辑范围为 `[-1m, 0m]`；实体 1m 行程未改变。
- 默认初始（SRDF `home`）姿态为 `updown=-0.3m`，左臂
  `[155, -105, 20, 90, -90, -40, 0]°`、右臂
  `[-155, 105, -20, -90, 90, 40, 0]°`。
- 第二初始（SRDF `second_home`）姿态为 `updown=-0.6m`，左臂
  `[30, 80, 20, 90, 90, 60, 0]°`、右臂
  `[-30, -80, -20, -90, -90, -60, 0]°`。
- 卸货（SRDF `unloading`）姿态为 `updown=-0.3m`，左臂
  `[130, -105, -180, 20, -90, -30, 0]°`、右臂
  `[-130, 105, 180, -20, 90, 30, 0]°`。
- 第二卸货（SRDF `second_unloading`）姿态为 `updown=-0.6m`，左臂
  `[0, 100, 0, -45, 100, 45, 0]°`、右臂
  `[0, -100, 0, 45, -100, -45, 0]°`。
- 四个命名姿态中未列出的头部、悬挂、舵轮和夹爪关节均为0；完整数值合同见
  `config/named_poses*.yaml`。
- 夹爪版的 `left_moving_jaw_joint` 和 `right_moving_jaw_joint` 分别连接同侧
  `*_link7` 与 `*_moving_jaw`，沿父链接局部 `+X` 方向移动，范围为
  `[0, 0.080]m`，默认 `0m` 表示 CAD 闭合状态。
- 现有 `left_tool0`、`right_tool0` 仍固定在同侧 `*_link7` 的
  `[0, 0, 0.13585]m`；它们不随夹爪开合移动，也未重新定义为夹持中心或吸附面 TCP。

SRDF 只定义五个规划组：

- `left_arm`
- `right_arm`
- `dual_arm`
- `dual_arm_with_updown`
- `whole`

两套末端共用这五个规划组。夹爪版共含 28 个可动关节：14 个臂关节、
1 个立柱升降关节、2 个头部关节、2 个夹爪关节、1 个主动悬挂关节和
8 个底盘转向/轮关节。吸盘版不含夹爪 link/joint，共 26 个可动关节；
真空通断不建模为运动关节。
下游状态发布方需要提供所选版本的全部关节位置。夹爪和底盘关节不加入这五个
规划组；硬件接口与控制器配置由对应软件包维护。

两套末端定义集中在 `urdf/robot_v3_end_effectors.xacro`，共用机身、机械臂、
零位和限位。下游加载时需同时选择对应配置：

| 末端 | Xacro 入口 | 初始位置 | 关节限位 |
| --- | --- | --- | --- |
| 双夹爪 | `urdf/robot_dual_gripper.urdf.xacro` | `config/initial_positions_gripper.yaml` | `config/joint_limits_gripper.yaml` |
| 双吸盘 | `urdf/robot_dual_suction.urdf.xacro` | `config/initial_positions_suction.yaml` | `config/joint_limits_suction.yaml` |

### 双夹爪导入约定

首次夹爪包的 `robot.urdf` 与 `robot_dual_gripper.urdf` 内容相同，以后者为来源。
保留 Kkozia 夹爪固定主体和移动夹爪，同时由 V3.1.1 提供双臂运动链；
夹爪资产、行程和碰撞偏移不因机械臂升级而改变。

| 输入链接或关节 | 仓库链接或关节 |
| --- | --- |
| `link_010` | `right_link7` |
| `left_moving_jaw` / `left_moving_jaw_joint` | `right_moving_jaw` / `right_moving_jaw_joint` |
| `link_017` | `left_link7` |
| `right_moving_jaw` / `right_moving_jaw_joint` | `left_moving_jaw` / `left_moving_jaw_joint` |

新增网格位于 `meshes/robot_v3/`，URDF 使用
`package://robot_description/meshes/robot_v3/` 路径和统一的
`scale="0.001 0.001 0.001"`，将毫米转换为米：

- `ee_fixed_body_visual.stl`、`ee_fixed_body_collision.stl`：固定主体视觉与碰撞网格。
- `moving_jaw_visual.stl`、`moving_jaw_collision.stl`：移动夹爪视觉与碰撞网格。

每侧固定主体质量为 `4.968 kg`，移动夹爪质量为 `0.086 kg`；质量、质心与惯量来自
ELECTRI-136 新机械导出并按临时 FK 规则转换到 V3.1.1 link frame，尚待台架标定。
固定主体碰撞网格仍保留输入的 `[-0.040201, -0.071594, 0]m` 偏移，
不能按视觉网格的零偏移处理。夹爪的行程、effort、velocity、damping、friction
均为输入包描述值，尚需实机核对；曲柄与连杆的闭环运动约束未建模。

### 双吸盘导入约定

吸盘包的 `robot.urdf` 与 `robot_dual_suction.urdf` 内容相同，以后者为来源。
它将两侧末端整体替换为吸盘，并移除移动夹爪，使用相同的
`link_010 → right_link7`、`link_017 → left_link7` 映射。

### 机械臂 link / joint 命名

`robot_fixed_names.urdf` 修正了机械臂实体与关节同名的问题。控制接口继续使用
`left_joint1..7` 和 `right_joint1..7`；对应实体改为 `left_link1..7` 和
`right_link1..7`。每个关节连接前一级 `*_linkN` 与后一级 `*_linkN+1`，
末端 `tool0` 和夹爪/吸盘主体连接 `*_link7`。该变更不改变坐标、零位、网格、
质量或惯量。

机械臂关节的 effort/velocity 同步采用该文件中的分级参数：J1/J2 为
`647 / 1.309`，J3 为 `484 / 1.309`，J4 为 `459 / 1.749`，J5 为
`217 / 2.618`，J6 为 `107 / 3.142`，J7 为 `36 / 3.142`。

- 每侧质量：`2.618 kg`；质心：`[-0.000330, -0.000125, 0.070303]m`。
- 网格：`meshes/robot_v3/suction_visual.stl` 与 `suction_collision.stl`。
- 视觉原点：`[0, 0, 0]m`；碰撞原点：`[-0.158443333, -0.088374996, 0]m`。
- 质心惯量沿用输入文件，网格统一使用 ROS 包路径和 `0.001` 毫米缩放。

### 主动悬挂四舵轮底盘导入约定

主动悬挂来自 2026-09-12 的整机 `robot.urdf` 导出。该文件的
`base_link.stl` 同时聚合上车体和固定底盘，不能直接接入现有语义模型；
公共 `urdf/robot_v3_chassis.xacro` 因此保留已校核的 `chassis_base` 底板，
只迁移主动悬挂承载件和四组新舵轮。9 个新网格位于
`meshes/active_suspension/`，从源米制 STL 转为毫米制 STL，URDF 继续统一使用
`scale="0.001 0.001 0.001"`。

源 `Link_29` / `Joint_29` 分别映射为 `active_suspension_carriage` /
`active_suspension_joint`。源关节沿局部 `+Z` 轴移动，逻辑范围为
`[-0.15, 0]m`；`0m` 是 CAD 收回零位，`-0.15m` 是最大伸出位置。
前侧两组舵轮固定连接 `chassis_base`，后侧两组共同随悬挂承载件移动。

| 舵轮编号 | 物理位置（+X 前、+Y 左） | 源 link | 父 link | 转向关节 | 行走轮关节 |
| --- | --- | --- | --- | --- | --- |
| 01 | 右前 | `Link_27/28` | `chassis_base` | `caster01_joint` | `wheel01_joint` |
| 02 | 右后 | `Link_30/31` | `active_suspension_carriage` | `caster02_joint` | `wheel02_joint` |
| 03 | 左后 | `Link_32/33` | `active_suspension_carriage` | `caster03_joint` | `wheel03_joint` |
| 04 | 左前 | `Link_21/22` | `chassis_base` | `caster04_joint` | `wheel04_joint` |

装配按“轮胎最低点接地、立柱底面贴底盘上表面”对齐：

- `base_footprint → base_link` 由四轮轴心和 `0.1m` 轮半径推导为
  `[0.1900000028, -0.0000442724, 0.4000025060]m`。
- `base_link → chassis_base` 为 `[-0.19, 0, -0.034415142]m`，无旋转；
  保留旧底盘的水平中心 `[0.005, 0.015]m`（相对 `base_footprint`）。
- 源整机坐标到 `base_footprint` 的对齐平移为
  `[-0.001, 0.015, 0.25673805]m`。
- 悬挂零位包含 `0.1565mm` 网格高度补偿；四轮最低点为 `z=0m`，底盘安装板
  下表面为 `z=0.320m`、上表面为 `z=0.332m`。
- Kkozia 上身保持原 `0.0142m` 安装抬升；双臂通过
  `xyz=[0,0,-0.0142]m, yaw=90°` 的固定适配帧接入 V3.1.1 关节链。

旧 `model_base` 中的 `part_038` 至 `part_046` 共九个固定底盘零件已从视觉和
碰撞中移除。旧模型质量/惯量与网格按 `1200 kg/m³` 的积分结果一致；据此用
平行轴定理从原总成扣除旧底盘 `48.9719861146 kg`，将剩余上身质量由
`79.116973 kg` 调整为 `30.1449868854 kg`，同步更新质心和惯量。
固定底板、主动悬挂承载件及四组舵轮质量合计 `55.40394687 kg`，沿用各自
输入 URDF 的质量属性。
这些属性来自 CAD/网格估算，不是新增的实测质量。

本次保留新输入的八个 `revolute` 关节、`[-3.14159, 3.14159]rad` 限位和零初始位置，
包括四个行走轮；因此当前描述不提供行走轮无限连续旋转。
导出报告列出的四个未分配零件 `part_002`、`part_005`、`part_009`、`part_011`
未进入源 URDF，本次也未额外补入。

### 头部和胸部相机导入约定

头部与胸部相机来自 `local_mub984qs_3132rb_urdf_stl` 的 2026-09-21 导出，
定义集中在 `urdf/robot_v3_head.xacro`，运行时网格位于
`meshes/head_chest_camera/`。源 `robot.urdf`、`parts.json`、`user_model.json`
和 `export_report.json` 保存在 `model_sources/head_chest_camera_20260921/`。

- `head_mount_fixed` 将新 CAD 的偏航轴固定到 `arm_carriage`，零位为
  `xyz=[0.267, 0, 1.4228]m, yaw=90deg`；接口名保持不变。
- `head_joint` 连接 `head_mount → head_yaw`，`head_pitch_joint` 连接
  `head_yaw → head`。两轴沿用源 `[-1.57, 1.57]rad` 行程、effort=10、velocity=5。
- `head` 承载新头部相机和 MID-360；镜头表面零件从源 `up_and_down`
  重新归入 `head`，使它们正确跟随偏航和俯仰。
- `chest_camera_fixed` 将胸部 ZED X 固定到 `arm_carriage`，相机本体几何中心为
  `[0.366, -3.92e-9, 1.11530000583]m`，局部 `+X` 朝机器人前方，并随
  `updown` 升降。
- CAD 中 `part_047/057` 头部双目视锥、`part_067` 雷达视场体和
  `part_071/081` 胸部双目视锥是设计辅助几何，不进入 URDF 的 visual、
  collision 或质量属性。
- `head` 的 `1.00116361294 kg` 和 `chest_camera` 的 `0.137417570647 kg`
  仅由保留的实体网格按 `1200 kg/m^3` 积分得到，仍需用实物数据替换。
- 当前只提供机械安装 link，未定义厂商光学坐标系；外参标定完成后再增加
  `*_optical_frame`。
- `whole` 规划组同时包含回转与俯仰关节，仍保持原来的五个规划组。

### 待集成的机械臂惯性参数初值

ELECTRI-136 接收的 2026-09-24 机械导出及逐连杆参数保存在
[`model_sources/arm_inertial_20260924`](model_sources/arm_inertial_20260924/README.md)。
该来源作为台架标定前的 CAD 初值；用户选择公共 V3.1.1 为目标并授权临时 FK 坐标映射，
当前分支已据此更新机械臂及夹爪版惯性块。结果仍未实机验证，吸盘末端参数未被覆盖；
原始文件哈希、映射方法和后续标定边界见该目录说明。

## 3. 依赖模块

- ROS 2 Humble
- `ament_cmake`
- `xacro`
- 测试时需要 `ament_cmake_pytest` 和 `check_urdf`

## 4. 编译方式

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description --symlink-install
```

## 5. 启动方式

本仓库只提供描述资产，不拥有整机启动入口。由 bringup 或 MoveIt 配置包加载：

```text
share/robot_description/urdf/robot.urdf.xacro
share/robot_description/srdf/robot.srdf
```

展开吸盘版：

```bash
source install/setup.bash
xacro "$(ros2 pkg prefix --share robot_description)/urdf/robot_dual_suction.urdf.xacro" -o robot_dual_suction.urdf
```

也可在公共入口传入 `end_effector:=gripper`。未指定参数时加载吸盘版。

修改 Xacro 后需同步本分支的直接加载文件；测试会检查两者一致：

```bash
xacro urdf/robot.urdf.xacro -o urdf/robot.urdf
```

## 6. 测试方式

```bash
colcon test --packages-select robot_description
colcon test-result --verbose
```

测试覆盖 URDF 单树合法性、左右物理侧、V3.1.1 双臂来源、升降逻辑坐标、
默认姿态、关节限位、五组 SRDF、双夹爪所属侧与行程、末端坐标系稳定性、
吸盘质量与碰撞偏移、两套模型的网格配置及可动关节 YAML 覆盖，
并检查末端切换不改变共同的机器人结构。底盘测试还直接用 STL 顶点检查
四轮零位接地、四轮轴心平均值与 `base_footprint` 重合、后轴伸出 150mm、
转向后接地高度、安装面贴合、旧底盘移除和质量去重。
实机使用前仍需在 RViz 和实物上
核对两侧末端安装、吸附面 TCP，以及夹爪运动方向和开合干涉。

## 7. 负责人

运控与机器人描述负责人共同维护；机械安装、编码器方向和实机零位仍需对应负责人验收。
