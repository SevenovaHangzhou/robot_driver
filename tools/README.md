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
python3 src/rt_control/integ_bringup/scripts/validate_machine_profile.py \
  --manifest <manifest> --all
ros2 run integ_composer compose_artifact --output /tmp/<dir> --robot-variant <v> \
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

## arch_check 设计（待实现）

单一入口 `tools/arch_check.py`，吸收现有 `rt_control_layer_gate.py`，不另起第二套。约定：

- 输出格式固定为 `path:line: RULE-ID 一句话`，RULE-ID 对应本规范章节（如 `OWN-1.1`、`SIZE-3`、`STATUS-1.4`）。
- 退出码 0 通过、1 有违反、2 工具自身错误。
- 只读源码树与安装的 share 目录，不访问硬件、网络、构建目录。
- 子命令按"改动类型"而非"规则类型"组织，让执行者改完只跑一条。
- 秒级返回；超过 10 s 的检查（如 colcon build）不属于本入口，由 `run_scoped_tests.sh` 承担。

| 子命令 | 检查什么 | ARCHITECTURE 条款 |
| --- | --- | --- |
| `arch_check all` | 下列全部 | — |
| `arch_check layers` | 包前缀 ↔ 层一致；依赖方向（package.xml、C++ include、Python import、CMake find_package、测试同样计入）；跨包只用 ARCHITECTURE §1.2 四种方式 | 1.2、1.3 |
| `arch_check status` | 每个 `package.xml`、`config/**.yaml`、`launch/*.py` 有 `status: production\|draft\|retired`；`production` 入口不引用 `draft`/`retired`；`retired` 不在构建闭包 | 1.4 |
| `arch_check size` | C++ ≤ 1000、Python ≤ 600、YAML ≤ 400 | 3 |
| `arch_check refs` | `machine_config`、`controllers.yaml`、manifest、policy 文件中所有 `package/file/path` 字符串引用可解析；`authorities.source` 指向存在的文件 | 1.2 |
| `arch_check facts <kind> <name>` | 列出一个事实（`joint`/`module`/`device`/`controller`）在源码树的全部出现位置，并按 ARCHITECTURE §1.1 判定每处是"拥有者"还是"违反"；输出形如 `enable_policy.yaml:12: OWN-1.1 left_joint7 在策略文件中按名引用，应通过 managed_joints 绑定` | 1.1 |
| `arch_check names` | 机型名（`Alfa_V3`、`alfa_v3`）、协议名（`damiao`、`zeroerr`、`kinco`）只出现在允许的层 | 1.1、§5a 原则 |
| `arch_check duplicates` | 已知重复实现签名清单（SocketCAN open、Modbus 读保持寄存器、CiA402 状态字解码、`encode_double`、新鲜度判定）在非 infra 包出现即报 | 3 |
| `arch_check compose <variant> <unit> <profile> <scope>` | 内存 compose 一次并跑全部 schema/交叉校验，不写文件；是 `validate_machine_profile.py` 的替代入口 | 1.6 |

`facts` 的事实来源是 `hwcfg_machines` 的 manifest；工具不维护第二份清单。
