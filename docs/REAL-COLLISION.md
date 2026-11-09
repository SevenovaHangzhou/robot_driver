# 真机模型的双臂互碰与单臂自碰撞

2026-11-07：`prepare_real_experiment.sh` 默认在运行 artifact 的 URDF 上建立碰撞模型，将双臂互碰与各臂自碰撞加入 Mink QP，并在生成 Rolling 窗口前复核三次插值轨迹。该入口只进行离线或只读采集后的影子求解；没有运动发布端，不使能、不切模式，也不启动或重启控制栈。

## 模型和范围

模型唯一来源为当前 artifact 的 `robot_description.xml`。关节顺序、安装变换、轴向、TCP 和限位继续与该文件核对；碰撞几何只读取其 `<collision>`，不从 `<visual>` 猜测，也不引用旧仿真模型。网格根目录由 `--model-share` 指定，默认 `../../sim/robot_description`。

本轮模型有 16 个臂部碰撞 STL，包括两臂各 7 段连杆和末端工具。每个文件的原始 URI、SHA-256 随准备报告保存。导入保留 URDF 的 origin 和 scale；受控连杆或有惯性的末端工具缺少碰撞体、STL 缺失、路径越界或编译失败时直接报错。

| 检查范围 | 本轮实际模型的几何对数 |
| --- | ---: |
| 左右臂之间，包含工具 | 64 |
| 左臂非相邻连杆之间 | 20 |
| 右臂非相邻连杆之间 | 20 |
| 合计 | 104 |

仅排除同一固定刚体及直接相邻刚体的装配接口，与固定版本 Mink 的规则一致；本轮排除 16 对。不根据零位是否碰撞自动删对，不套用其他模型的 SRDF 禁碰表。另一臂及辅助关节仍由等式约束冻结，不会为了避碰自动让另一臂移动。

本次范围为臂间和同臂检查，**未覆盖手臂与躯干、头部、底盘、现场障碍，以及未建模的工具和线缆**。报告明确列出 `scope` 和 `uncovered`，`collision_checked=true` 仅指列出的检查范围。

MuJoCo 对每个 STL 使用凸包距离，会填充凹陷；结果可能比原始三角面碰撞保守，不等同 FCL 或现场几何验收。

## 约束和插值复核

`DisplacementCollisionLimit` 将固定版本 Mink 1.3.0 的法向速度不等式转换为 `delta_q` 的位移不等式，再加入现有配置限位、速度限位和非目标关节冻结约束。末端位置和朝向仍是软任务：受阻时不能保证到达请求目标。影子轨迹未满足路径误差要求时不保存为 Rolling 窗口。

初始姿态低于碰撞余量时拒绝求解，不自动尝试脱离。求解完成后按控制器实际的三次 Hermite 段检查：在区间中点计算形状距离，根据每个关节在整段内的极值和连杆半径上界，给出形状的最大运动量。距离减去两形状的运动上界仍足够时才放行；否则继续细分。有实际越界、数值异常、细分深度或总检查预算耗尽时拒绝轨迹。这能检查端点和中点采样漏掉的段内碰撞；结论只适用于所导入的凸几何和给定余量，不包括实机跟随偏差、制动过程或模型外障碍。

候选参数集中在 `config/real_collision.json`：最小距离 30 mm、检测距离 120 mm、增益 0.05，来源为参考 `real2sim_v3/config/mink_demo.json`。它们明确标为 `candidate_not_hardware_validated`，没有将仿真参数改称实机认可参数；`hardware_motion_ready` 继续为 false。关节加速度仍在轨迹生成后检查，尚未新增 jerk 或力矩约束。

## 离线运行

对已有 artifact 和同模型的关节快照运行，不连接工控机：

```bash
script/prepare_real_experiment.sh \
  --artifact-dir .runtime/experiment-20261107/prepared-current \
  --snapshot .runtime/experiment-20261107/prepared-current/snapshot.json \
  --model-share /home/zlz/Desktop/ing_obj/sim/robot_description \
  --domain-id 78 --arm left \
  --output .runtime/collision-replay-left
```

输出目录须不存在。右臂改为 `--arm right` 并使用另一个输出目录。可用 `--collision-config /absolute/policy.json` 显式指定候选参数；准备入口始终加载碰撞配置，没有无碰撞降级开关。

`report.json` 包含覆盖范围、几何对数、网格指纹和每方向的碰撞检查结果。`shadow-*.json` 只是离线 Window 输入，带有 `replay_only=true` 和占位时间原点，不可直接作为现场命令。当前五字段 Rolling 接口的 epoch/sequence 应由公共 `RollingSender` 管理，不能复用旧 open/close 会话传输。

## 验证边界

专项及关联回归：81 项通过、1 项因缺少外部网关 fixture 跳过。覆盖双臂互碰、同臂碰撞对保留、固定/相邻排除、障碍前停止、非目标关节冻结、初始碰撞拒绝、STL origin/scale、缺资源拒绝，以及稀疏采样漏检和区间预算耗尽。

使用 2026-11-07 13:18:57（北京时间）已采集的真机快照离线回放：左右臂各六方向、0.5 mm/s、1 mm 全部通过。该姿态的模型最小采样间距约 54.07 mm。证据为 `.runtime/real-collision-20261107-verified-left/report.json` 与 `.runtime/real-collision-20261107-verified-right/report.json`，回归日志为 `.runtime/real-collision-20261107-tests.log`。这不是当前实时姿态或实际运动结果，命令发布数为 0。

实现入口：`gripper_jog/collision_model.py` 导入几何，`real_collision.py` 负责约束和区间复核，`real_model.py` 接入求解，`experiment.py` 默认启用并记录结果。
