# ALFA Robot Description

当前默认模型：**chassis-head-default-20261125**。

用户已确认的底盘／头部模型现已替换默认入口 `urdf/robot_alfav3_mobile.urdf`。无需再选择preview路径；visual与collision变体、cuRobo球和固定盒配置同步更新。

## 入口

| 路径 | 用途 |
| --- | --- |
| `urdf/robot_alfav3_mobile.urdf` | 默认整机URDF，含visual/collision/inertial |
| `urdf/variants/robot_visual.urdf` | 同一运动链，仅外观 |
| `urdf/variants/robot_collision.urdf` | 同一运动链，仅碰撞网格 |
| `config/curobo/robot.yaml` | 15轴规划配置、其他11轴锁定 |
| `config/curobo/collision_spheres.yaml` | 239球；双臂/升降架原球，头部候选球 |
| `config/curobo/fixed_body_boxes.yaml` | 3盒：001、002、004，frame=base_link；003已删除 |

网格包含 `mesh/` 和 `preview/chassis_head_20261124/meshes/` 下的资产，均使用本包URI，ROS安装包含这些路径。preview目录保留用户确认的来源，不代表默认入口仍是旧模型。

## 当前合同

- footprint→base_link为`[0.07478297093,0,0.370] m`，旋转0。
- 行走关节的局部 Z 轴穿过轮毂。零舵角时四个车轮的正转轴均沿 `base_footprint` 的 +Y，正轮速对应 +X 行走。
- 底盘轮组、雷达、IMU及超声波安装来自确认的v3.4.2映射版本；保留原轮号/通道物理对应。
- 四轮安装朝向与用户确认的preview完全一致，按用户要求去除a41105c的额外+90°修正。默认入口与preview在相同关节值下显示一致。
- 头部TF和网格取确认版。双臂/升降架/TCP保持此前约定，不导入v3.4.2的双臂MDH符号变化。
- 原48个link和关节保留，新增chest_camera_left及其固定关节；无up_and_down/point_arm_carriage辅助TF。
- updown=0仍为确认的高位，Home/stow关节值不变；description硬件行程[-1.033,+0.017]，Motion必须另外限制工作范围[-1,0]。
- 原右眼帧保留，但新源没有单独右眼模型/外参更新，需后续标定确认。

## 碰撞与动力学边界

当前Fast链路使用球+盒，不能只读URDF就得到完整规划碰撞模型。三个固定盒需要消费方显式接入sphere-box与box-box SAT。URDF碰撞网格仍可供其他消费者使用。

头部7球是新网格的候选拟合，胸口相机尚无球表达；部分新源缺失惯性暂沿用旧值。用户外观确认与本包坐标测试不代表完整碰撞、动力学、全流程规划或实机验收。算法仓库冻结资产和运行中工控机不会随description main自动更新。

## 重建与验证

新默认生成器是`tools/promote_chassis_head.py`；`compose_approved_carriage.py`现转入新默认生成器，防止恢复旧头部/四盒。旧历史生成器仍禁止覆盖当前版本。

```bash
cd /mnt/mydisk/ALFA/worktrees/description-promote-chassis-head && \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/promote_chassis_head.py && \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pytest test -q
```

验证记录见`validation/chassis_head_default_20261125/`；历史文档、旧验收不是本版的验收证明。
