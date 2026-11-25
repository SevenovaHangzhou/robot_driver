# 底盘／头部TF与三盒模型候选

模型入口：`urdf/preview_corrected.urdf`。网格URI相对于robot_description仓库根目录解析。当前分支提供URDF、网格及碰撞配置，尚未替换main默认模型。

- 删除base_box_003，保留001、002、004。
- 底盘轮组、雷达、IMU、超声波采用v3.4.2核对后的安装关系，保留原轮号和通道的物理对应。
- 头部采用新TF与网格；双臂、升降架和TCP保持已确认版本。
- URDF保留visual和collision；cuRobo球配置及三盒位于`config/curobo/`。
- 头部球仍为待审核拟合；新增胸口相机尚无球碰撞表达。未完成完整动力学或规划验收。

变更详情见PREVIEW_NOTES.md；changes.json与dynamic_check.json记录坐标核对结果，manifest.json记录资产哈希。source目录仅用于来源追溯。
