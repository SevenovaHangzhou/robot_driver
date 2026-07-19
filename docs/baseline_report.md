# T-001 legacy baseline report

Date: 2026-07-20

This report is read-only evidence for the rt_control migration. No file under any legacy checkout was modified. Requirement and mapping precedence remains: frozen REQ table, hardware mapping export, implementation specification, then existing code as evidence of current state.

## 1. Baseline decision and repository census

The user decision in AMB-011 is applied: the only migration baseline is `/home/kkozia/robot_driver` at `6bc94cd564361b9e4aa785af74683ecf13e258ce`. It is clean. The other four checkouts are reference-only and must not be accepted by `diff_legacy.py`.

| Checkout | HEAD / timestamp | Branch | Worktree | Relevant content |
| --- | --- | --- | --- | --- |
| `/home/kkozia/robot_driver` | `6bc94cd564361b9e4aa785af74683ecf13e258ce`, 2026-06-29 16:36:18 +0800 | `feature/ethercat-motor-driver` | clean | Authoritative EtherCAT config and `alfa_chassis_driver`; no `canopen_master` package |
| `/home/kkozia/alfa_robot` | `948689c59fb741b5451903491be1683ea1a36f08`, 2026-06-05 11:03:37 +0800 | `fixed-platform-dual-arm-acceptance-20260605` | dirty: 3 modified paths and 2 untracked trees | No `dual_arm_ethercat_control/config`; no `canopen_master` |
| `/home/kkozia/code/alfa_robot` | `b345cd2912f7e135bf24a75589ffc22082e008e1`, 2026-05-12 16:38:20 +0800 | `v5_dev` | dirty: `CLAUDE.md` | No `dual_arm_ethercat_control/config`; no `canopen_master` |
| `/home/kkozia/alfa_robot_v5_dev_review` | `be41e9d1f0b549afa2877eb6fe47f165060c294e`, 2026-07-06 19:11:59 +0800 | `v5_dev` | clean | No `dual_arm_ethercat_control/config`; no `canopen_master` |
| `/home/kkozia/robot_driver_review` | `07c87e572ebf1e42e25f3e36a5ca96dcba000c27`, 2026-07-16 02:26:11 +0800 | `feature/unsafe-direct-motion-entry` | clean | Non-authoritative later EtherCAT config and a `canopen_master` implementation |

The authoritative repository has 160 non-build files: 22 C++, 26 HPP, 24 Python, 29 Markdown, 15 YAML, 12 XML, and supporting build/documentation assets. Its relevant history is an ICube source import at `f27098a`, chassis import at `360f4fb`, and documentation at `6bc94cd`.

## 2. Cross-checkout configuration comparison

Only `robot_driver` and `robot_driver_review` contain `dual_arm_ethercat_control/config`. The other three checkouts cannot supply a competing config. File-level comparison found six differing files:

| File | Difference in non-authoritative `robot_driver_review` |
| --- | --- |
| `controllers.yaml` | Adds a 13-axis `dual_arm_command_permit_controller`; the JTC joint list remains the same. |
| `drive_profiles.yaml` | Changes Ti5 candidate PDOs from `0x1600/0x1A00` to position-only `0x1601/0x1A01`; changes validation metadata. |
| `ethercat_topology.yaml` | Adds five activation batches and the 15-position scan expectation; the 13 joint-to-position mappings remain unchanged. |
| `icube_profiles/joint2.yaml` | Changes Ti5 to `0x1601/0x1A01`, removes velocity/effort/mode/gap channels, and adds `0x60C2:01=4`, `0x60C2:02=-3`. |
| `icube_profiles/joint3.yaml` | Same material changes as `joint2.yaml`. |
| `joint_limits.yaml` | Changes validation/status text; physical limit and scaling values are unchanged. |

This is a real semantic conflict, not formatting drift: authoritative `6bc94cd` has Ti5 `0x1600/0x1A00` and additional velocity/effort/mode interfaces, whereas frozen REQ-ECAT-002/004 and the hardware mapping export require position-only `0x1601/0x1A01` plus Ti5 `0x60C2`. The frozen requirements and mapping therefore govern the target, while the legacy checkout remains the named comparison baseline. The resulting `diff_legacy.py` policy question is recorded as BQ-001.

Authoritative config SHA-256 fingerprints:

```text
13c9ed8c7b3bb9ffe926c426cb55467263564b8e68bda1ea10ad8b5a65c679b0  controllers.yaml
c796823de5872867dcf15d852f30c64b7f370870f86b43b79eab3def9b8d7a62  drive_profiles.yaml
0ec2ee6aa48dd86938d81b32b42cf8eaf9701faf618c1b036b0948d92b06cea9  ethercat_topology.yaml
59e54845916aa347d75ac1a567535bc66ffbf490d09f200a4bcb4ccf2a0fe183  icube_profiles/joint1.yaml
fef30dd547127b654b3076a3c4a5537456e663909b7d7eb88745f035ba257f3e  icube_profiles/joint2.yaml
fef30dd547127b654b3076a3c4a5537456e663909b7d7eb88745f035ba257f3e  icube_profiles/joint3.yaml
59e54845916aa347d75ac1a567535bc66ffbf490d09f200a4bcb4ccf2a0fe183  icube_profiles/joint4.yaml
59e54845916aa347d75ac1a567535bc66ffbf490d09f200a4bcb4ccf2a0fe183  icube_profiles/joint5.yaml
59e54845916aa347d75ac1a567535bc66ffbf490d09f200a4bcb4ccf2a0fe183  icube_profiles/joint6.yaml
f9de61a006ccf188ddd89a9c63f3041e6b8fcd8039b65b5f30300e52653f7773  icube_profiles/left_joint6.yaml
14fa80a7c943524df47cad8f788ce0b2433b1c30c6372a808514fa39ecff29c8  icube_profiles/right_joint6.yaml
c04f90306b5ec40ffbeb04b1028198e34118df941957aff36b138112ee2d8cd9  icube_profiles/turn.yaml
7b8bc5868167921ecc461c501f1fcdfc340870abc2f33c9724e2b3d6402b399e  joint_limits.yaml
```

## 3. Authoritative topology and controller configuration

The complete authoritative `ethercat_topology.yaml` is:

```yaml
metadata:
  schema_version: 1
  status: verified_by_igh_scan
  warning: >
    slave position 已按工控机 `ethercat slaves` 实机扫描结果和左右臂人工验证回填。
    单位换算、方向、零位和关节限位仍未确认，不能用于 strict 硬件启动。
master:
  id: 0
  interface: enp3s0
  mac: 8c:59:3c:14:ff:d3
  cycle_hz: 250
  dc_reference: first_device
topology:
  branch_mode: single_master_dual_branch
  junction: EtherCAT 专用 junction/coupler，支持 DC，非普通以太网交换机
  junction_supports_dc: true
slaves:
  - {joint: right_joint1, alias: 0, position: 1, profile: joint1, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: right_joint2, alias: 0, position: 2, profile: joint2, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: right_joint3, alias: 0, position: 3, profile: joint3, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: right_joint4, alias: 0, position: 4, profile: joint4, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: right_joint5, alias: 0, position: 5, profile: joint5, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: right_joint6, alias: 0, position: 6, profile: right_joint6, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: left_joint1, alias: 0, position: 7, profile: joint1, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: left_joint2, alias: 0, position: 8, profile: joint2, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: left_joint3, alias: 0, position: 9, profile: joint3, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: left_joint4, alias: 0, position: 10, profile: joint4, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: left_joint5, alias: 0, position: 11, profile: joint5, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: left_joint6, alias: 0, position: 12, profile: left_joint6, verification: VERIFIED_BY_IGH_SCAN_AND_MANUAL_ARM_CHECK}
  - {joint: turn, alias: 0, position: 14, profile: turn, verification: VERIFIED_BY_IGH_SCAN_AND_USER_ZERO_LIMITS}
```

The complete authoritative `controllers.yaml` is:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 250

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    dual_arm_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_state_broadcaster:
  ros__parameters: {}

dual_arm_trajectory_controller:
  ros__parameters:
    joints:
      - right_joint1
      - right_joint2
      - right_joint3
      - right_joint4
      - right_joint5
      - right_joint6
      - left_joint1
      - left_joint2
      - left_joint3
      - left_joint4
      - left_joint5
      - left_joint6
      - turn
    command_interfaces:
      - position
    state_interfaces:
      - position
    allow_partial_joints_goal: false
    open_loop_control: false
    constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.01
```

AMB-014 is confirmed: the stored JTC already contains exactly 13 axes (`right_joint1..6`, `left_joint1..6`, `turn`), and `turn` is the final joint in the single FollowJointTrajectory controller.

## 4. Launch chain and existing enable behavior

The authoritative runtime chain is:

```text
dual_arm_ethercat_control.launch.py
  -> xacro dual_arm_ethercat.urdf.xacro
  -> robot_state_publisher (normal runtime only)
  -> controller_manager/ros2_control_node
  -> joint_state_broadcaster spawner
  -> dual_arm_trajectory_controller spawner (only selected modes)
```

`ros2_control_node` loads `ethercat_driver/EthercatDriver`; each of the 13 `<ec_module>` entries loads `ethercat_generic_plugins/EcCiA402Drive` and a slave YAML. `EthercatDriver::on_activate()` configures SDOs, activates the IgH master, then runs a blocking update loop waiting for every module's `activation_ready()` or a default 15 s timeout. The normal 250 Hz `read()`/`write()` path calls `master_.readData()`/`master_.writeData()` behind a `std::try_to_lock` mutex.

No fixed five-batch enable implementation exists in the authoritative checkout. The CiA402 plugin contains an optional process-global `coordinated_enable_group` barrier, but the launch explicitly passes an empty group and expected count zero. It does not encode the required right J1-3 -> left J1-3 -> right J4-6 -> left J4-6 -> turn sequence or 0.2 s delays. The later non-authoritative review checkout adds batching in `ethercat_driver/src/ethercat_driver.cpp`, but AMB-007 supersedes that fork patch with `auto_state_transitions: false` plus the new controller-based `enable_manager`.

The existing plugin's global coordinated-enable mutex and the driver's read/write mutex are also incompatible with REQ-RT-004 for the target RT path; they are evidence only and must not be copied into the new controller design.

## 5. CANopen source location

The required full-depth search found:

```text
/home/kkozia/robot_driver_review/canopen_master/include/canopen_master/canopen_codec.hpp
/home/kkozia/robot_driver_review/canopen_master/src/canopen_codec.cpp
```

No `canopen_codec.hpp`, `canopen_codec.cpp`, or `canopen_master` package exists in authoritative `robot_driver@6bc94cd`. Its only chassis launch is `alfa_chassis_driver/launch/chassis.launch.py`. That package implements a separate two-track stack and its stored values conflict with the frozen mapping in material ways (for example node IDs 17/2 rather than 2/3). Therefore it may be used only as behavior cross-check evidence; CANopen target numbers come exclusively from `ecat_canopen_hardware_mapping_export.md`.

The reference-only `robot_driver_review/canopen_master` is the source named by the mapping document for codec/runtime behavior. T-006/T-014 may read it, but `diff_legacy.py` must not treat it as the frozen legacy baseline.

## 6. CSP target-position preload comparison

The authoritative fork is not equivalent to the current upstream reference by simple configuration. It carries a large custom `EcCiA402Drive` patch that:

- reorders the plugin domain map to process TPDO before RPDO;
- records finite/raw `0x6064` and forces one raw `0x607A=0x6064` write;
- blocks switch-on/operation-enable until the preload is complete;
- prevents an initial finite controller command from bypassing the forced preload;
- adds dry-run diagnostics, bounded fault reset, optional startup SDOs, and coordinated-enable state.

`TARGET_POSITION_PRELOAD_REVIEW.md` records a non-motion hardware dry run with all 12 then-configured axes within 5 counts and a maximum delta of 3 counts. That result is useful evidence but does not prove the zero-patch upstream target.

The current reference `/home/kkozia/rt_control_refs/ecat_icube@7a32bd7b8fc066c6668b1df7446c89aff570bb7d` has only the upstream default: for target position it sets `factor * last_position + offset` only after `mode_of_operation_display_ != MODE_NO_MODE`; it does not gate `0x000F` on a completed preload. ZeroErr's frozen position-only TPDO has no `0x6061`, so the source does not establish the prerequisite claimed by REQ-ECAT-005. This discrepancy is recorded as BQ-002 and blocks declaring T-010/T-013 safe without a user-approved resolution.

## 7. Fork-to-upstream file inventory

Comparison reference: local ICube checkout `7a32bd7b8fc066c6668b1df7446c89aff570bb7d` (`jazzy`) versus the authoritative imported source. The authoritative repository has no recorded upstream ancestry beyond its single import commit, and the local reference is not a Humble branch, so this is an exhaustive file-level snapshot diff, not a historical patch series.

### `ethercat_driver`

- Differing: `CMakeLists.txt`, `ethercat_driver_plugin.xml`, `include/ethercat_driver/ethercat_driver.hpp`, `package.xml`, `src/ethercat_driver.cpp`.
- Upstream-only: `examples/`, `include/ethercat_driver/ethercat_bus_manager.hpp`, `include/ethercat_driver/ethercat_ros2_control_xml_parser.hpp`, `src/ethercat_bus_manager.cpp`, `src/ethercat_ros2_control_xml_parser.cpp`, `test/test_ethercat_safety_driver.cpp`.
- Fork-only: `test/test_ec_module_param_parser.cpp`.

### `ethercat_interface`

- Differing: `CMakeLists.txt`, `include/ethercat_interface/ec_master.hpp`, `include/ethercat_interface/ec_pdo_channel_manager.hpp`, `include/ethercat_interface/ec_sdo_manager.hpp`, `include/ethercat_interface/ec_slave.hpp`, `src/ec_master.cpp`, `test/test_ec_pdo_channel_manager.cpp`.
- Upstream-only: `include/ethercat_interface/ec_pdo_group_interface_channel_manager.hpp`, `include/ethercat_interface/ec_pdo_single_interface_channel_manager.hpp`, `include/ethercat_interface/ec_transfer.hpp`, `src/ec_pdo_channel_manager.cpp`, `src/ec_pdo_group_interface_channel_manager.cpp`, `src/ec_pdo_single_interface_channel_manager.cpp`.

### `ethercat_generic_plugins`

- CiA402 differing: `ethercat_generic_cia402_drive/CMakeLists.txt`, `include/ethercat_generic_plugins/cia402_common_defs.hpp`, `include/ethercat_generic_plugins/generic_ec_cia402_drive.hpp`, `package.xml`, `src/generic_ec_cia402_drive.cpp`, `test/test_generic_ec_cia402_drive.cpp`, `test/test_generic_ec_cia402_drive.hpp`.
- Generic slave differing: `ethercat_generic_slave/CMakeLists.txt`, `include/ethercat_generic_plugins/generic_ec_slave.hpp`, `src/generic_ec_slave.cpp`, `test/test_generic_ec_slave.cpp`, `test/test_load_ec_modules.cpp`.
- The CiA402 subtree delta is 2,083 insertions and 159 deletions across seven files; it includes the preload/gate changes summarized above.

### `ethercat_manager`

- Differing: `CMakeLists.txt`, `include/ethercat_manager/ec_master_async.hpp`, `include/ethercat_manager/ec_master_async_io.hpp`, `src/ethercat_sdo_srv_server.cpp`.

### `ethercat_msgs`

- Differing: `CMakeLists.txt`.

## 8. Downstream conclusions

- T-003 may use the user-specified `alfa_robot_description` source; no authoritative pure-kinematics URDF exists in `robot_driver` beyond the hardware-only dual-arm xacro.
- T-004 has enough frozen mapping data to produce the target position-only configuration, but it must not copy the authoritative Ti5 interface shape blindly.
- T-005 cannot honestly report legacy 100% equivalence until BQ-001 is resolved.
- T-006 must use the mapping export for all numeric values and use the non-authoritative codec only for behavior cross-checks.
- T-010 must not claim REQ-ECAT-005 closure until BQ-002 is resolved; T-013 remains hardware-gated regardless.
