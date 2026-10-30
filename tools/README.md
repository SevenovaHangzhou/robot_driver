# 验证命令

按改动类型查。所有命令只读源码或构建目录，不访问硬件。规则见根 [`AGENTS.md`](../AGENTS.md) 第 6–7 节；
改动类型与唯一位置见 [`ARCHITECTURE.md`](../ARCHITECTURE.md) §2（`tools/arch_check.py` 实现后以它为准）。

## 每次提交

```bash
tools/run_scoped_tests.sh   # 日常内环：按改动范围裁剪 repository/quality gate 与 colcon test --packages-above
tools/quality_gate.sh       # 提交/PR 前必跑；pre-commit 与 CI governance 用同一份
```

增量内环不能替代 `quality_gate.sh`；命中 `patches/`、`deps.repos`、`versions.env`、`docker/`、CI 等全局路径时改跑全量。
发布门禁用 `python3 tools/release_test_runner.py`，见 `domains/rt_control/testing/README.md`。

## ROS 包（C++ / Python）

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <affected>
colcon test --packages-select <affected> && colcon test-result --verbose
```

公共头文件、接口或共享配置变化时扩大到所有下游（`--packages-above`）。"无测试"不等于验证通过。
C++ 改动还要人工检查实时路径的分配、锁、阻塞、单位/符号、边界与错误传播。

## robot_description 构建副本

```bash
xacro src/description/robot_description/urdf/robot_alfav3_mobile.urdf > /tmp/robot-description-review.urdf
check_urdf /tmp/robot-description-review.urdf
colcon build --symlink-install --packages-select robot_description
```

结构性变更还要检查 TF 树、关节限制、碰撞模型、mesh 路径以及各消费者使用的模型版本。

## 接口

先导入 `deps.repos`，再构建公共 vendor 包、`rt_control_interfaces` 与全部本域消费者，并全仓搜索旧名称：

```bash
colcon build --symlink-install --packages-up-to robot_rt_control_interfaces robot_system_interfaces \
  robot_interfaces_qos rt_control_interfaces
colcon build --symlink-install --packages-above rt_control_interfaces
rg -n "<changed-interface-or-field>" src docker tools domains
```

## 机型配置与组合

```bash
python3 src/rt_control/rt_control_bringup/scripts/validate_machine_profile.py \
  --manifest <manifest> --all
ros2 run rt_control_composer compose_artifact --output /tmp/<dir> --robot-variant <v> \
  --machine-unit <u> --physical-profile <p> --control-scope <s> --mock
```

结构变更前后对每个 scope 生成 artifact 并 `diff -r`，证明输出不变。

## EtherCAT / CANopen / 补丁

```bash
bash tools/check_ecat_sync_shutdown_policy.sh
```

补丁要在 `deps.repos` 指定的上游提交上 `git apply --check`，并构建/测试受影响的上游包；用隔离临时工作区，不污染源码。
CANopen 生成物从权威 EDS 重建，不手改 `.bin/.dcf`。

## Launch、脚本

- Python/launch：`python3 -m py_compile <files>`，并做适用的 `use_mock_hardware:=true` 启动检查；
- Shell：`bash -n <files>`，可用时 `shellcheck`。

容器、Compose 与宿主脚本见 [`docker/README.md`](../docker/README.md)、[`hostsetup/README.md`](../hostsetup/README.md)。
