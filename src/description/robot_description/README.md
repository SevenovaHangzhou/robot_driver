# robot_description

当前候选 **unified-20261112-r2**：一套运动链，三种几何表示。保留GitHub全部Link和关节，额外加入前后雷达与八个超声波测量坐标系。用于新架构接入前的模型审查，尚未完成新模型规划/实机验收。

| 部分 | 本版采用 |
|---|---|
| base_footprint → base_link | cuRobo PR82：`xyz=(0,0,0.335)m`，零旋转 |
| 底盘、Updown、arm_carriage | cuRobo PR82的本体几何与升降定义 |
| 头部、眼部、头部雷达 | GitHub完整链：`arm_carriage → H1_Link → H2_Link → mid360_Link`，保留原父坐标与局部变换 |
| 四组转向/车轮关节 | GitHub全部关节、visual和惯性保留；规划配置锁定，URDF中不删除 |
| 左右arm_carriage → J1及J1～J7 | GitHub description `4f9069a`，原点、轴向、零位、限位和Link几何原样采用 |
| 工具TCP | 自有J6轴线到TCP **234.2mm**；新link7下`xyz=(0,0,0.2342)`、`rpy=(0,0,-π/2)`保持旧TCP轴约定 |
| 工具实体/惯性、腕部FT传感器、IMU | GitHub版本；TCP不是工具实体Link，也不移动FT安装位 |
| 底盘碰撞 | 已确认的封闭外观外壳；轮组没有独立碰撞几何 |
| 前后雷达 | 下表用户确认的footprint位姿，固定连接base_link；仅测量坐标，不猜型号/外形/惯量 |

当前47个Link、25个运动关节；GitHub原有37个Link、36个关节均完整保留，再新增两个雷达Link及固定关节。规划组仍为升降＋双臂15轴，cuRobo配置将H1/H2和四组转向/车轮共10轴锁在零位。锁定不代表删去运动链。工具实体保持GitHub几何，本版“自有工具”固定的是TCP位置/轴约定，并非重新设计工具外形。

头部各局部安装变换与GitHub `100d251` 相同，没有展平成相对base_link的静态安装。H1/H2转动或Updown升降时，头部雷达随之运动；两个车体雷达保持固定。主要关系如下：

| 父 → 子（关节） | XYZ（m） | RPY（rad） |
|---|---|---|
| arm_carriage → H1_Link（H1_joint） | `(0.267,0,0.36515)` | `(π,0,0)` |
| H1_Link → H2_Link（H2_joint） | `(-0.02,0,0.24625009252407)` | `(π/2,0,0)` |
| H2_Link → mid360_Link（mid360_joint） | `(0.05857,0.066019,0)` | `(-1.5708,0,-0.37885)` |

左右眼也保留各自原始H2父关节，无质量且只作visual。`config/alfav3_mobile_manifest.json`保存GitHub完整关节合同与例外：仅root、Updown和两个TCP采用已确认的覆盖值；其他关节的父子关系、origin、axis、limit必须逐项一致。

| 相对base_footprint | XYZ（m） | RPY（度） |
|---|---|---|
| lidar_main | `(0.453313447977869,0,0.398099404)` | `(0,15,0)` |
| lidar_rear | `(-0.047,0,1.959)` | `(0,-90,0)` |

高度采用看图确认值，未增加15mm。写入base_link下的Z分别为0.063099404m、1.624m。调整Updown不改变这两个雷达的位姿。

## 资产入口

### 运控范围与Home

`config/motion_scope.yaml`声明运控当前只负责`updown`和左右J1～J7共15轴；完整URDF仍保留25轴。
`config/poses/home.yaml`是用户选定的15轴关节预设，来自`alfa-v3-curobo-legacy-26.11.10-buildable`发布中的`rezero_home.json`，以具名关节、rad/m保存并保留来源哈希。
它不是完整25轴状态，也不包含头部或车轮指令。其余10轴应由实时状态输入，或由仿真启动配置明确提供；缺失时不能自动填零。
执行方应拒绝超出15轴清单的指令；本文件是声明合同，不代表仓库本身提供了执行器。未来增加控制轴须显式修改清单并验证消费者。
Home可供仿真或任务选择，不属于机械出厂参数，不自动触发归位。已检查全部15轴数值符合当前限位；尚未完成新模型下Home自碰撞、到达路径或Task验证。模型生成器不会覆盖这份业务预设，更新模型后需重新检查适用性。

| 文件 | 用途 |
|---|---|
| `urdf/robot_alfav3_mobile.urdf` | 唯一权威运动链，含visual与collision |
| `urdf/variants/robot_visual.urdf` | 仅visual几何；关节、坐标系、惯性与权威模型一致 |
| `urdf/variants/robot_collision.urdf` | 仅collision几何，适合FCL/碰撞预览 |
| `config/curobo/robot.yaml`、`collision_spheres.yaml` | cuRobo配置与**258个有效球**：原臂/升降架222球＋新头部36球 |
| `config/curobo/fixed_body_boxes.yaml` | **必须一并加载的4个固定盒体**，frame=base_link |
| `config/model_variants.yaml` | 三种表示的统一入口索引 |
| `ultrasonic_channel_1..8_link` | 八个 A22 探头的测量坐标系（表面中心，+X 为声束方向）；无几何、无惯性 |
| `config/alfav3_mobile_manifest.json` | 固定来源、部件选择、Link转换、关节源合同、产物哈希 |
| `config/mesh_assets.json` | 当前引用的mesh与哈希；不依赖外部工作区绝对路径 |
| `srdf/robot_alfav3_mobile.srdf` | 具名组及相邻Link排除，不含业务Home |

原臂球心按共同arm_carriage下的零位刚体变换迁入新Link局部坐标；半径不变。底盘原71球不再放入资产，升降架20球和机械臂球保留。旧head/head_yaw的20球删除，避免套到不同几何和坐标链上。新H1/H2/mid360按各自碰撞mesh以60mm网格分组三角面、生成包住分组全部顶点的球，另加0.5mm明确表面余量，分别8/18/10球；每个三角形都包含于其分配球内。该局部表面近似不是实体体积覆盖或整机安全证明。

上游运动链存在毫米级差异，所以旧模型的世界轨迹/碰撞结果不可直接复用。新球位姿始终由本版运动链计算。轮组保留visual/惯性但没有独立collision，规划碰撞仍使用外壳/四盒；本资源不提供车轮接地动力学配置。

**四盒不是标准cuRobo YAML自动支持的机器人OBB。** 使用者须显式接入sphere-box（机器人）和box-box SAT（环境）后端；只加载`robot.yaml`会缺少底盘/立柱碰撞。`validate_curobo_assets.load_config(root)`只提供可搬移配置解析，不声称完成规划服务接入。

原臂/本体球盒仍是研究近似：按网格顶点与三角形中心采样，最大缺口约55.4mm（升降架）；手臂/工具约5～26mm。新头部采样全部被覆盖，但整体覆盖报告不等于体积证明，三种显示一致也不等于碰撞安全。4盒/258球未加入RRT专用10%膨胀、负载球或外部场景。前后雷达只有TF；头部mid360保留GitHub实际visual/collision/惯性及新球近似。

双臂 J6 限位为 **±110°**（2026-11-12 更新），权威 URDF、visual/collision 变体及限位配置保持一致。

双臂速率采用GitHub30°/s、60°/s²；头部和轮组限位来自GitHub，Updown沿用旧模型。头部mid360质量0.375kg，惯性为上游几何近似，未重新硬件确认。质量/惯量保持对应来源，不把XML正定校验当动力学标定。旧算法Home与轨迹均未登记为本版有效结果。

## 生成、检查、预览

生成器固定读取Git对象，不读取研究工作区未提交修改。PR82确认以下资产与PR80 `92bd8cdb`逐字节一致，来源URDF/YAML/四盒哈希写入清单。

```bash
cd /mnt/mydisk/ALFA/SevenovaHangzhou/robot_description
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/compose_robot_model.py \
  --description-repo /mnt/mydisk/ALFA/SevenovaHangzhou/robot_description \
  --description-ref 4f9069a86765385d3a2773f1bded3b5cf310218a \
  --algorithm-repo /mnt/mydisk/ALFA/alfa_robot_v3_curobo \
  --algorithm-ref 92bd8cdb4d8b79793c428727db429288c17dd4f2 \
  --output-root /mnt/mydisk/ALFA/SevenovaHangzhou/robot_description
```

生成需要Python3、NumPy、PyYAML、trimesh和两个来源Git对象；会生成配套资产并删除上一份mesh清单中已无引用的文件。更新源版本后，必须重新审核差异及运行检查，不能直接沿用旧验证报告。

```bash
cd /mnt/mydisk/ALFA/SevenovaHangzhou/robot_description
source /opt/ros/humble/setup.bash
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pytest -q test
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/validate_curobo_assets.py --samples 64
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/validate_curobo_assets.py --samples 64 --all-joints
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/audit_collision_coverage.py
```

GPU检查使用现有Fast API（PR82依赖的cuRobo `76c2f4b6`），读取本版完整具名状态，比较FK、TCP和全部球坐标；不执行规划或硬件命令。普通测试无GPU依赖；独立解析器对拍需要yourdfpy。覆盖审计需要trimesh。

合并前模型的验证记录：`validation/unified_model_20261112.json`，该记录不代表本次 J6 限位更新后重新进行了 GPU 验证。模型测试覆盖GitHub完整关节清单、头部原父链和动态雷达、保留轮组关节及三种表示一致性。GPU分别验证15轴锁定配置和解锁全25轴的诊断模式；解锁只用于坐标对拍，不修改发布规划配置。轮子在URDF中保持continuous，当前cuRobo解析器内部将其约为±6.28rad；本次诊断采样在±π内，未宣称无限圈导航规划。比较的是坐标/资产一致性，尚未运行完整IK、轨迹规划、Gazebo或真实碰撞场景验收。

```bash
cd /mnt/mydisk/ALFA/SevenovaHangzhou/robot_description
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/preview_model_variants.py --port 8291
```

预览依赖已安装的Viser、yourdfpy、trimesh、SciPy。默认“TF单机”：半透明外观、完整39项TF树、搜索/显隐/聚焦和父关节局部位姿；名称按相机视角分列到屏幕两侧，引线指向真实原点，文字排布不改变FK。可切换“三栏对照”，同一状态驱动三种表示，只作左右平移，无隐藏旋转或高度补偿。支持透明度、关节、坐标轴长度调节。三个雷达有原点标记，头部还显示真实网格和球近似；“聚焦所选TF”可近距离检查H1/H2及其传感器。当前状态来自页面关节值，不是ROS实时TF。

来源单模型仍保存在GitHub `100d251` / Gitea `8570a78`；本PR不改动这些历史提交。
