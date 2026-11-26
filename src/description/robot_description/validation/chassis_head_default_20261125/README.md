# 默认入口切换验收

默认URDF与visual/collision变体由确认的preview_corrected.urdf生成，并保留GitHub a41105c的四轮+90度零转角修正。三个默认盒为001/002/004，头部球同步preview候选；没有加入临时高度标注工具。

- 22项模型测试通过：三份运动链一致、安装/升降、双臂不变、新源头部/底盘传感器TF、轮组正轮速朝+X和悬挂、模型资源哈希、碰撞配置/关节合同、独立CPU FK。
- cuRobo 15轴16状态：GPU球与CPU FK最大分量差2.8e-7m，末端最大平移分量差2.4e-7m。见gpu_fk.json。
- 调用旧公开生成入口也可重建新默认模型，urdf/config哈希无变化。见rebuild.json。
- 标准ROS安装已包含preview目录，package URI所引用新mesh在安装后仍可解析。

这是模型和配置一致性检查，不是完整碰撞包络、任务规划或动力学验证。头部候选球、新相机无球和惯性缺失回退的已知边界见README。未更新算法冻结资产、未部署实机。
