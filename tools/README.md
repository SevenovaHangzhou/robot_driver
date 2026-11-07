# 验证命令

按改动类型查。所有命令只读源码或构建目录，不访问硬件。规则见根 [`AGENTS.md`](../AGENTS.md) 第 6–7 节；
改动类型与唯一位置见 [`ARCHITECTURE.md`](../ARCHITECTURE.md) §2，架构规则由 `tools/arch_check.py` 执行。

## 每次提交

```bash
tools/run_scoped_tests.sh   # 日常内环：按改动范围裁剪 repository/quality gate 与 colcon test --packages-above
tools/quality_gate.sh       # 提交/PR 前必跑；pre-commit 与 CI governance 用同一份
tools/arch_check.py         # 架构规则（quality_gate 也会跑）；秒级，任何改动后都可单独跑
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

## arch_check

`tools/arch_check.py` 执行 `ARCHITECTURE.md` 的架构规则，每次扫描全仓（含未提交、未忽略的文件），
只读源码树，不访问硬件、网络或构建目录。仓库卫生与策略（固定版本、Docker、CI、CODEOWNERS 等）归
`repository_gate.py`，格式归 `style_gate.py`；每条规则只在一个工具里。

```bash
tools/arch_check.py                 # 全部规则组
tools/arch_check.py layers names    # 只跑指定规则组
tools/arch_check.py --json          # 每条发现一行 JSON
```

输出 `路径:行: 规则ID 说明`，退出码 0 通过、1 有发现、2 工具自身错误。

| 规则 | 检查什么 | ARCHITECTURE |
| --- | --- | --- |
| LAYER-1 | `src/rt_control/<目录>/package.xml` 的包名与目录一致；前缀属于六层之一（层由前缀决定，不另列清单） | 1.3 |
| LAYER-2 | 依赖方向：下层不依赖上层；infra 不依赖任何域包；control 只经 `hw_semantic_components` 使用 protocol；periph 只依赖 infra，只有 integration 依赖 periph。依赖取自 package.xml、C/C++ include、Python import、CMake `find_package`，测试同样计入 | 1.3 |
| LAYER-3 | `hwcfg_machines` 只放数据：其他包只能 `exec_depend` 它，它自身不含代码文件 | 1.1 |
| LAYER-4 | 外设包 `include/<包>/protocol/` 不依赖 ROS | 1.3 |
| LAYER-5 | `listen_only_peripherals` 中外设的生产 C/C++ 不写总线 | 6 |
| OWN-1 | 控制频率数值只写在 manifest 与 `integ_composer/config/controller_defaults.yaml` | 1.1 |
| OWN-2 | 组合器与渲染器不读取 `evidence/` | 1.6 |
| NAME-1 | 机型名只写在 `hwcfg_machines`；机型名取自 `hwcfg_machines/machines/<机型>/` 目录名，不区分大小写与分隔符 | 1.1 |
| NAME-2 | 厂家与产品名（`arch_rules.yaml` 的 `vendor_names`）只写在 `hw_*`、`hwcfg_machines`、`periph_*`；包名（如依赖 `hw_can_damiao`）不算 | 1.1 |
| NAME-3 | 关节名只写在 `hwcfg_machines`；关节名取自其模块文件的 `robot_model_joint`、`joint_name` | 1.1 |
| SIZE-1 | 生产源码与配置 C/C++ ≤ 1000、Python ≤ 600、YAML ≤ 400 行；测试、金样、fixtures、schema 与 `integ_bringup/upstream/` 原样上游代码不计 | 3 |
| DUP-1 | 已抽取的实现不得再抄：`arch_rules.yaml` 的 `duplicates` 为每类实现给出拥有者与特征（原始 CAN 套接字、Modbus TCP 帧头、CiA402 状态字掩码、渲染器 XML 参数、已删除的 double 编码），特征出现在其他包的生产源码中即报告；注释不算 | 3 |

内容规则只看生产文件（不含 `test/`、`tests/`、`fixtures/`、`schema/`）；NAME 还不看 `golden/` 与 Markdown，代码注释照查。

`tools/arch_rules.yaml` 只放规则数据：`exceptions`（每条写明规则、文件、原因、批准人与日期；
不再命中的例外本身会被报告）、`listen_only_peripherals`、`vendor_names` 与 `duplicates`。`ARCHITECTURE.md` §3 的
`ctrl_enable_manager` 主文件例外登记于此。

### 后续步骤

每条规则在现状清零后接入门禁（用户 2026-11-06：清零即接入，不逐条确认；需要新增例外或改变行为时先问）。不做豁免清单，§3 写明的单一例外除外：

1. `arch_check compose --against <ref>`：对基线可组合的每个机型/台套/配置/作用域组合，在基线与当前树各组合
   一次并逐文件比较，基线成功而当前失败也报告；用作结构变更不改行为的证据（ARCHITECTURE §7）。按需运行，
   不在默认规则组里。
