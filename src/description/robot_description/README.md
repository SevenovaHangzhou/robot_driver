# robot_description

本仓库只维护一套模型：V3.2.2 移动底盘/升降 + Alfav3 双臂 + 总装1011++6 上身基座及头部。
当前模型基于 Rerun 中展示的 `main 3126c2f` 模型，双臂沿用当前 URDF 的零位和对齐 V3 的执行器参数。

**唯一 URDF 入口：[urdf/robot_alfav3_mobile.urdf](urdf/robot_alfav3_mobile.urdf)。**
保留这个文件名，以便已有下游入口继续使用。它不是历史文件 `robot_v3_2_2.urdf`。

## 运控手臂碰撞体（2026-10-27）

左右 `link1`～`link7` 的碰撞 STL 来自用户提供的 `robot_urdf_stl_20261027`，以共同 `arm_carriage` 和各模型零位求固定刚体转换后导入当前 link 坐标。外观、关节轴/原点/零位、惯性、工具碰撞和 TCP 均未随之替换；J2 当前限位为左臂 −92°～+122°、右臂 −122°～+92°。

转换来源、14组矩阵和顶点核对见 `config/arm_collision_import.json`，复现工具为 `tools/import_motion_collision.py <源目录>`。零位最大 STL 浮点误差约 1.5e-8 m；同关节角下两套运动链并非完全等价，历史对比姿态 J2=90°/122°时最大碰撞顶点差约 1.105/1.367 mm；其中右 J2=122°超出当前限位，仅为运动学对比数据，不可作为执行目标。本变更未验证全姿态碰撞安全裕量。

URDF/Rerun保留导入的原始三角面外形。MuJoCo派生模型同步引用这些网格，但其网格接触采用凸包近似，会填充凹陷，不能当成与FCL/原始STL完全一致的碰撞结果。

## 文件

| 路径 | 用途 |
| --- | --- |
| `urdf/robot_alfav3_mobile.urdf` | 唯一模型定义，可直接编辑 |
| `srdf/robot_alfav3_mobile.srdf` | 规划组和碰撞排除，不含旧命名姿态 |
| `config/joint_limits_alfav3_mobile.yaml` | 当前模型关节限位，与 URDF 保持一致 |
| `config/alfav3_arm_module_specs.json` | eRob 执行器参数、额定/峰值口径和来源 |
| `config/alfav3_mobile_manifest.json` | 模型约定、零位偏移和历史来源 |
| `config/mesh_assets.json` | 58 个必需网格及 SHA-256 校验值 |
| `mesh/` | URDF 引用的 58 个原始网格，按部件和 visual/collision 分类 |
| `test/test_current_model.py` | 单树、网格、限位、惯性、零位与 SRDF 校验 |

MuJoCo 模型、转换网格、转换/碰撞演示工具及专属测试已迁至同级 [robot_simulation](../robot_simulation/README.md)：模型入口为 `../robot_simulation/mujoco/robot.xml`，完整场景位于 `../robot_simulation/world_description/`。仿真项目读取本仓库的 URDF 和原始 `mesh/`，本仓库独立维护机器人描述。

2026-10-23 按仓库全部 URDF/XML 的实际引用清理：删除旧 `meshes/` 下 81 个未引用 STL（86.6 MiB）及其空目录，移除 `model_sources/` 下与正式入口完全相同的 URDF 副本。`config/mesh_assets.json` 记录原始 URDF 网格；MuJoCo 转换网格由 XML 引用。

网格统一放在 `mesh/`，URDF 的 `package://robot_description/mesh/…` URI、资产清单和安装配置同步使用以下结构；仿真项目的 MuJoCo XML 通过相对路径引用这些原始网格：

```text
mesh/
├── chassis/          # 底盘、转向与车轮，20 个 STL
├── torso/            # 总装1011++6 上身基座，1 个 STL（米）
├── head/             # H1/H2、左右眼视距和 mid360，5 个 STL（米）
└── arms/
    ├── visual/       # 双臂视觉，16 个 STL
    └── collision/    # 双臂碰撞，16 个 STL
```

原 V3 底座的 `part_001_solid_001.stl` 在 `chassis/` 中命名为 `base_link.stl`，避免与另一来源的同名文件冲突。资产校验值保留不变。
这次清理没有改变 URDF、SRDF、网格、关节限位数值或动力学参数。

## 1011++6 上身和头部替换（2026-10-24）

源文件 `../总装1011++6/urdf/总装1011++6.urdf` 中的 `base_link` 是上身肩部基座，对应本模型的 `arm_carriage`。其网格、质量和惯量替换原升降架上的上身组件，沿用装配变换 `xyz=[0.266,0,0.36515] m`、`rpy=[π,0,0]`；移动底盘 `base_link`、升降和双臂关节变换保持原值。源 STL 已以米为单位，不加旧网格的 0.001 缩放。

旧 `head_yaw/head` 和 `head_joint/head_pitch_joint` 已替换为 `H1_Link/H2_Link` 和 `H1_joint/H2_joint`，接入 `Leye_link`、`Reye_Link`、`mid360_Link` 及其固定关节。H1/H2 位置限位按 2026-11-05 台架实测：偏航 H1 ±45°，俯仰 H2 −40°～+35°（模型坐标）；effort=10 N·m、velocity=5 rad/s 沿用旧头部配置。当前共 35 个 link、25 个活动关节。

头部俯仰轴（2026-11-03 用户复核）：以左右 J1 轴线交点 arm-base-point 为原点、沿 base_link，零位下 H2 原点为 [−19, 0, 246.250093] mm，H1 原点为 [+1, 0, 0] mm；俯仰轴位于偏航轴后方 20 mm。H2 及其固定子连杆的外形、碰撞、惯性已按原世界位置补偿。左右 J1 原点本身有微米级左右差，交点 Y 约 +0.005 mm。

左右眼按无质量视距范围处理，仅作半透明视觉显示，无惯性或碰撞几何。mid360 质量按用户指定为 0.375 kg；源传感器质心与惯量异常，当前采用网格包围盒的均匀实体近似，待实测替换。来源摘要、原始 mid360 惯性数据及近似方法见 `config/alfav3_mobile_manifest.json`。

## 零位和参数

- 模型零位为全部关节角/位移为 0；`updown=0 m` 是升降上端点。
- 左 J5 的模型零位绕自身 +Z 轴偏转 180°：`q5_export = q5_model + π`；右 J5 保持导出零位。正方向不变。
- 左 J3/J6 轴向已按台架确认结果取反；右臂编码器零位和方向仍需实机校验。
- 双臂角度限位：J1/J3/J5/J7 ±160°、左 J2 −92°～+122°、右 J2 −122°～+92°、J4 ±150°、J6 ±135°。
- 双臂 effort：`[647,647,484,459,204,107,36] N·m`，表示启停峰值扭矩，不是连续额定值。
- 双臂速度：URDF 与限位 YAML 统一为 30°/s，加速度 60°/s²（只在限位 YAML 的 `max_acceleration_rad_s2`，URDF 无此字段；用户 2026-11-04）。执行器峰值速度按 `[12.5,12.5,12.5,16.7,30,30,30] rpm` 换算，见 `alfav3_arm_module_specs.json` 的 `peak_velocity_rad_s`；J5 连续额定扭矩为 87 N·m。
- 14 个臂关节的 `friction=0`、`damping=0` 是未辨识占位值。
- `*_tool_link` 承载末端质量，`*_tool0` 为无质量 TCP，位于 link7 的 +Z 方向 0.2497 m。
- 包含 `imu_link` 和头部雷达 `mid360_Link`。模型中的末端、质量和惯量不能替代实机装配确认或动力学标定。

## 修改与验证

直接编辑当前 URDF，并同步相关限位 YAML/清单。历史生成器依赖已归档的多个模型，已移出当前分支，避免重新生成时覆盖当前模型。
若修改网格，应更新 `config/mesh_assets.json` 对应校验值。需要同步 MuJoCo 派生模型时，从仿真项目执行 `.venv/bin/python tools/build_mujoco.py`，并运行该项目的 `test/test_mujoco_model.py`。

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q test
colcon build --packages-select robot_description
colcon test --packages-select robot_description
colcon test-result --verbose
```

## 完整备份

清理前完整版本：`3126c2f0716eaefc13d6bfbbf955027125156950`。
GitHub 和 Gitea 均保留标签 **`backup/before-single-model-20261020`**。
旧 V3/V3.1.1、解析代理模型、历史 Xacro、姿态、生成脚本与源模型全部可在该标签中找到。

独立取回备份，不影响当前工作区：

```bash
git worktree add --detach ../robot_description-archive backup/before-single-model-20261020
```

本次仓库清理不修改工控机部署或硬件编码器标定。
