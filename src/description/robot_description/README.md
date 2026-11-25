# ALFA Robot Description

当前模型：`carriage-assembly-approved-20261124`。本版把用户在 8353 页面确认的机械装配与四个碰撞盒发布到默认入口；升降零位按用户确认的当前装配高度定义。

## 当前入口

| 文件 | 用途 |
| --- | --- |
| `urdf/robot_alfav3_mobile.urdf` | 默认整机模型，完整运动链、外观、惯性与碰撞网格 |
| `urdf/variants/robot_visual.urdf` | 同一运动链，只含外观 |
| `urdf/variants/robot_collision.urdf` | 同一运动链，只含碰撞网格 |
| `config/curobo/robot.yaml` | cuRobo 15轴规划配置，其他11轴锁定 |
| `config/curobo/collision_spheres.yaml` | 239个有效球：双臂202、升降架30、头部7 |
| `config/curobo/fixed_body_boxes.yaml` | 用户确认的4个固定碰撞盒，坐标系为base_link |
| `config/carriage_assembly/approval.json` | 安装、升降零位、坐标与来源合同 |
| `config/carriage_assembly/approved_fixed_body_boxes.yaml` | 页面最终确认的四盒快照 |
| `config/motion_scope.yaml`、`config/poses/{home,stow}.yaml` | 15轴运控范围及具名姿态 |

标准URDF collision版本仍保留。当前算法的Fast链路使用球与四盒，不从URDF collision网格自动建模；其他仿真/FCL消费者可以继续使用网格。**四盒需要消费方显式接入sphere-box和box-box SAT，不能只加载robot.yaml就声称碰撞模型完整。**

## 坐标与安装

- `base_footprint → base_link`：平移 **[0.07478297093, 0, 0.370] m**，旋转为0。
- 四个轮组安装点相对各自父节点沿X减去74.78297093mm，抵消车身的新X偏移；轮组相对footprint的原X/Y及朝向保留。Z随根高度从335mm变成370mm。
- 左右J1零位的link1负Z轴延长线相交，交点对齐`point_arm_carriage`。旧上身坐标系与这个交点不重合，所以挂接时必须含平移补偿。
- 安装参考点只用于离线计算，不发布额外TF。已将两段固定变换折叠进`updown`，并将升降架visual/collision/inertial转换到`arm_carriage`。默认树仍是`base_link → updown → arm_carriage → 双臂/头部`，所有link和joint名称集合与原main一致。
- 零位下概念安装点（不发布TF）相对footprint为 **[0.443,0,1.66774990748] m**。
- 升降不再固定：`updown`连接base_link与arm_carriage，正方向为base_link +Z；0就是页面确认高度。升降时新架体、安装点、双臂和头部共同移动；底盘、四盒、轮组不随升降移动。
- description保留电控硬件行程 **[-1.033,+0.017] m**，速度0.1666m/s，加速度0.4m/s²。**运控工作范围仍是[-1.00,0]m，消费方必须施加这个较窄范围。** 扩展硬件行程不是运控权限。
- Home/stow的15轴关节值未改变；完整模型48个link、26个可动关节。模型包不发布实机命令；缺失实时关节状态仍按motion_scope处理，离线预览的零值不是真机反馈。

原固定雷达、IMU、超声波的局部base_link安装保留；它们相对footprint随根坐标改变，尚未独立重新标定。

## 碰撞资产迁移

- 所有球的半径与所属link局部球心保持逐字节一致，包括升降架的30球；球通过新TF整体迁移，不重复添加根偏移。
- 升降架新网格与原球统一放在`arm_carriage`下。网格局部origin完成转换；球心、半径不变。惯性参考坐标同步转换，质量和惯性张量数值保留，不重复旋转张量。
- 四盒为以下最终值（单位m，RPY均为0），历史草稿不参与发布：

| 名称 | 中心XYZ，相对base_link | 尺寸XYZ |
| --- | --- | --- |
| base_box_001 | [-0.074,0,-0.102] | [1,1,0.37] |
| base_box_002 | [-0.2,0,0.2] | [0.7,1,0.4] |
| base_box_003 | [-0.33,0,0.39] | [0.35,1,0.21] |
| base_box_004 | [0.023,0,0.796] | [0.294,0.4,1.59] |

这是一套用户确认的近似碰撞资产，不声称完整包络覆盖。表面采样测量、GPU坐标对拍及测试证据在`validation/carriage_assembly_20261124/`。没有以碰撞网格覆盖率修改用户确认的球或盒，没有做整条任务规划、实机运动或控制器验收。

几何与惯性来自装配预览源`7bdd71d`：底盘和升降架采用CAD网格及其惯性，arm_carriage采用新架体的质量和惯性，未重复累计旧架体质量。它不是新的实测动力学标定。

## 重建与验证

所有重建输入都在包内，不依赖临时目录或联网。新生成器使用已锁定原始装配URDF、审批参数与现有mesh；不会覆盖业务Home角度。旧compose_robot_model是历史生成器，已阻止它覆盖当前装配输出。

在当前开发机直接执行：

```bash
cd /mnt/mydisk/ALFA/worktrees/description-carriage-approved && \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/compose_approved_carriage.py && \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pytest -q test
```

GPU验证（只算坐标，不运行机器人）：

```bash
cd /mnt/mydisk/ALFA/worktrees/description-carriage-approved && \
export PYTHONPATH=/mnt/mydisk/ALFA/curobo_71ce1726 && \
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
nice -n 19 taskset -c 0-3 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/validate_curobo_assets.py --samples 32
```

预览（默认入口已是批准的装配模型）：

```bash
cd /mnt/mydisk/ALFA/worktrees/description-carriage-approved && \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/preview_model_variants.py --port 8354
```

历史设计、旧坐标与旧验收记录在`docs/history/README_before_carriage_20261124.md`；它们不能作为本版几何或碰撞验收。
