# 双臂限位 MuJoCo 碰撞验证（2026-10-22）

结论：未通过无碰撞验收，未提交或推送。

## 方法与通过项

- MuJoCo 3.15.0，加载 `mujoco/scene.xml`，保留现有碰撞几何、接触组和父子 link 排除。
- 编译后的左右臂 14 个关节限位与 `[160, 90, 160, 150, 160, 135, 160]` 度一致。
- 全关节零位：0 接触；100 步无控制动态仿真：无 MuJoCo 警告。此动态检查不表示受控轨迹无碰撞。
- 单关节扫描：每个臂关节在上下限之间均匀取 65 点（含端点），其他关节保持零位，升降位于 `updown=0`。共 910 个姿态，`mj_forward` 后检查 `data.contact`；穿透超过 0.01 mm 计为碰撞。
- 此为离散单关节检查，不能证明所有双臂组合姿态或连续轨迹无碰撞。

## 单关节扫描结果

| 关节 | 碰撞姿态 / 65 | 最大穿透 (mm) | 首个碰撞示例 (deg) | 示例碰撞体 |
| --- | ---: | ---: | ---: | --- |
| left_joint1 | 0 | 0.000 | — | — |
| left_joint2 | 3 | 160.913 | -90.0 | base_link / left_tool_link |
| left_joint3 | 0 | 0.000 | — | — |
| left_joint4 | 3 | 44.224 | 140.625 | base_link / left_tool_link |
| left_joint5 | 0 | 0.000 | — | — |
| left_joint6 | 8 | 49.955 | -135.0 | left_link4 / left_tool_link |
| left_joint7 | 0 | 0.000 | — | — |
| right_joint1 | 0 | 0.000 | — | — |
| right_joint2 | 3 | 158.706 | 84.375 | base_link / right_tool_link |
| right_joint3 | 0 | 0.000 | — | — |
| right_joint4 | 3 | 44.668 | -150.0 | base_link / right_tool_link |
| right_joint5 | 0 | 0.000 | — | — |
| right_joint6 | 8 | 49.942 | -135.0 | right_link4 / right_tool_link |
| right_joint7 | 0 | 0.000 | — | — |

## 原有限位与本次扩展的区别

- 左 J2 = -90°、右 J2 = +90° 均在原有限位内，末端与底座碰撞，最大穿透分别约 160.913 / 158.706 mm。
- 左 J4 = +145°、右 J4 = -145° 原端点已有碰撞，最大穿透分别约 25.700 / 26.258 mm；扩展至 ±150° 后相应端点约 44.224 / 44.668 mm。
- 左右 J6 在原 ±110° 端点无接触，在新 ±135° 端点末端与 link4 碰撞，最大穿透约 49.75–49.95 mm。
- 未修改用户指定限位、网格或碰撞排除以消除这些接触。

## 其他检查

- `check_urdf urdf/robot_alfav3_mobile.urdf` 与 `git diff --check` 通过。
- `python3 -m pytest -q test`：3 通过、3 失败。限位与执行器参数一致性测试通过。
- 两个失败来自工作区已有未跟踪文件 `model_sources/robot_alfav3_mobile.urdf`（破坏唯一入口及资产清单约束）；该文件保留原样，不纳入提交。
- 另一个失败为右 J5 零位约定断言；已用 HEAD 中未修改的 URDF 复现，属于原有测试与模型不一致。本次未调整零位。
- 已同步限位 YAML、README 及转换报告源文件 SHA-256。
