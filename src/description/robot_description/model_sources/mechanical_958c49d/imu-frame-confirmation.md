# 最终机械模型 IMU 安装坐标确认

用户提供：IMU相对本次机械导出根`base_link`位置为(403,0,0)mm。
用户经确认选择：IMU三轴与`base_link`同向；图中+X向右、+Y向上，按右手系+Z朝观察者。

固定变换：

```xml
<link name="imu_link"/>
<joint name="base_link_to_imu" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.403 0 0" rpy="0 0 0"/>
</joint>
```

依据截图：`/tmp/pi-clipboard-a7b580a9-4ff6-45e5-84ca-915ccfe746a5.png`及本轮用户对三轴同向的确认。
父坐标为`local_munsrkg7_6rqaj1_urdf_stl/robot.urdf`中的base_link，原URDF SHA256：
`958c49d7889827c9f520ac858783da62ca5ad249eaac765d7d761bd51e543b09`。
不自动等同旧V3部署模型中同名base_link；正式集成时须保留该来源。

本轮仅生成`final-with-imu-viewer.urdf`及`mechanical-final-958c49d-imu.rrd`查看副本。
没有修改机械原文件、公共模型仓库、发布TF或IMU驱动配置。
未虚构IMU质量/惯量/外形，实体几何维持原机械包，新增的是坐标frame。
该安装旋转不定义IMU设备输出的ENU/NED模式或是否由驱动进行了额外轴变换；运行集成时单独核对。
