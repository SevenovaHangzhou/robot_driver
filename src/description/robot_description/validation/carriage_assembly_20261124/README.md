# 2026-11-24 装配与碰撞资产发布验收

用户最终确认的根TF为X=74.78297093mm/Z=370mm，轮组单独抵消X偏移；双臂link1负Z射线交点对齐CAD安装点；四盒取config/carriage_assembly/approved_fixed_body_boxes.yaml。当前装配高度由用户确认对应updown=0。

按用户要求，正式树不含up_and_down、point_arm_carriage两个辅助link：固定变换合并到base_link→updown→arm_carriage，visual/collision/inertial都转换到arm_carriage下。所有48个link及47个joint的名称、类型和父子关系与此前main相同；保留15轴规划组和11轴锁定。

## 结果

- 模型CPU测试22项通过，包括三份URDF树一致、资源哈希、独立正解、限位、惯性、安装交点、轮组补偿、升降共同运动及原球/最终盒合同。见tests.txt。
- cuRobo发布15轴32个状态，另解锁26轴16个状态：GPU球和末端与独立CPU URDF正解对拍通过；见gpu_fk.json、gpu_all_joints.json。非完整规划/实机验收。
- 删除辅助TF前后，8个升降/臂/头姿态，保留坐标系与升降架几何、惯性位姿最大差4.65e-8m。源Rx使用3.1415927近似pi，折叠后保留原main的精确局部-Z升降轴以兼容cuRobo主轴要求，产生不足50nm/m的数值差；零位完全一致。质量与惯性张量数值不变，惯性参考姿态作刚体变换。见tf_collapse.json。
- Home、stow空载静态的手臂/头部球对四盒无超过1mm穿透；按原算法排除升降架与底盘的内部碰撞。不是全自碰撞或路径验收，见static_body_checks.json。
- 重建后urdf/config/mesh全部逐文件哈希相同；见reproducibility.json。旧生成器被阻止覆盖本版装配。

## 表面覆盖测量边界

没有重新拟合、膨胀或改动用户确认的球/盒。网格顶点和面心采样：底盘四盒覆盖约98.66%，最大漏出4.0mm；升降架30球覆盖约59.59%，最大采样缺口48.45mm。这是现有批准近似模型的测量，不能声称是完整保守包络或无碰撞证明。头部与双臂同样保留原球。完整测量见coverage.json。

## 发布边界

本次仅更新两站robot_description；算法仓库有冻结模型及独立四盒快照，需要另行迁移后才能使用此版。不能只替换URDF而继续使用旧四盒、旧根变换或旧mesh清单。description保留电控[-1.033,+0.017]m行程，Motion工作范围仍必须单独限制[-1,0]m。未部署工控机，未发送实机运动指令。
