import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/rt_control/control_api_adapter"))
sys.path.insert(0, str(ROOT / "src/rt_control/plc_node"))


class VacuumAndStatusContractTest(unittest.TestCase):
    def test_vacuum_idl_uses_fixed_channels_and_no_pressure_field(self):
        interface_root = ROOT / "src/interfaces/robot_control_interfaces"
        texts = "\n".join(
            (
                interface_root / "msg/VacuumState.msg"
            ).read_text().splitlines()
            + (interface_root / "msg/VacuumChannelState.msg").read_text().splitlines()
            + (interface_root / "msg/VacuumChannelFeedback.msg").read_text().splitlines()
            + (interface_root / "msg/VacuumChannelResult.msg").read_text().splitlines()
            + (interface_root / "action/VacuumGrip.action").read_text().splitlines()
        )

        self.assertIn("string LEFT=left", texts)
        self.assertIn("string RIGHT=right", texts)
        self.assertIn("bool attached", texts)
        self.assertIn("bool data_fresh", texts)
        self.assertNotIn("pressure_pa", texts)

    def test_plc_vacuum_inputs_remain_confirmed_di_bit0_bit1(self):
        from plc_node.plc_logic import decode_io_snapshot

        left = decode_io_snapshot(di_status=0b01, do_status=0, io_alarm=0)
        right = decode_io_snapshot(di_status=0b10, do_status=0, io_alarm=0)

        self.assertTrue(left.left_vacuum_established)
        self.assertFalse(left.right_vacuum_established)
        self.assertFalse(right.left_vacuum_established)
        self.assertTrue(right.right_vacuum_established)

    def test_vacuum_state_projection_uses_left_right_order(self):
        from control_api_adapter.vacuum_adapter import (
            PlcVacuumSnapshot,
            build_vacuum_channel_states,
        )

        channels = build_vacuum_channel_states(
            PlcVacuumSnapshot(
                connected=True,
                data_fresh=True,
                left_attached=True,
                right_attached=False,
                left_valve_open=True,
                right_valve_open=False,
                pump_enabled=True,
                error="",
            )
        )

        self.assertEqual([channel.channel for channel in channels], ["left", "right"])
        self.assertTrue(channels[0].attached)
        self.assertFalse(channels[1].attached)
        self.assertTrue(all(channel.data_fresh for channel in channels))

    def test_grip_requires_every_selected_channel_attached_before_success(self):
        from control_api_adapter.vacuum_adapter import (
            ATTACHED_VERIFIED,
            GRIP,
            PlcCommandResult,
            PlcVacuumSnapshot,
            VacuumAdapterCore,
        )

        class FakeVacuumIo:
            def __init__(self):
                self.calls = []
                self.snapshot = PlcVacuumSnapshot(
                    connected=True,
                    data_fresh=True,
                    left_attached=True,
                    right_attached=True,
                    left_valve_open=True,
                    right_valve_open=True,
                    pump_enabled=True,
                    error="",
                )

            def set_output(self, output_name, enabled):
                self.calls.append((output_name, enabled))
                return PlcCommandResult(True, "")

            def read_snapshot(self):
                return self.snapshot

            def wait_for_attachment(self, channels, timeout_s):
                self.calls.append(("wait_for_attachment", tuple(channels), timeout_s))
                return self.snapshot

        io = FakeVacuumIo()
        core = VacuumAdapterCore(io, grip_verify_timeout_s=2.0)

        result = core.execute_goal(GRIP, ["left", "right"], "default", "pick")

        self.assertTrue(result.succeeded)
        self.assertEqual(result.overall_verification_level, ATTACHED_VERIFIED)
        self.assertEqual(
            io.calls,
            [
                ("pump", True),
                ("left", True),
                ("right", True),
                ("wait_for_attachment", ("left", "right"), 2.0),
            ],
        )
        self.assertEqual(
            [channel.verification_level for channel in result.channel_results],
            [ATTACHED_VERIFIED, ATTACHED_VERIFIED],
        )

    def test_release_does_not_turn_off_common_pump(self):
        from control_api_adapter.vacuum_adapter import (
            RELEASE,
            UNVERIFIED,
            PlcCommandResult,
            PlcVacuumSnapshot,
            VacuumAdapterCore,
        )

        class FakeVacuumIo:
            def __init__(self):
                self.calls = []

            def set_output(self, output_name, enabled):
                self.calls.append((output_name, enabled))
                return PlcCommandResult(True, "")

            def read_snapshot(self):
                return PlcVacuumSnapshot(
                    connected=True,
                    data_fresh=True,
                    left_attached=False,
                    right_attached=True,
                    left_valve_open=False,
                    right_valve_open=True,
                    pump_enabled=True,
                    error="",
                )

            def wait_for_attachment(self, channels, timeout_s):
                raise AssertionError("RELEASE must not wait for attached=true")

        io = FakeVacuumIo()
        result = VacuumAdapterCore(io, grip_verify_timeout_s=2.0).execute_goal(
            RELEASE, ["left"], "default", "release"
        )

        self.assertTrue(result.succeeded)
        self.assertEqual(result.overall_verification_level, UNVERIFIED)
        self.assertEqual(io.calls, [("left", False)])

    def test_unknown_grip_profile_is_rejected_without_plc_writes(self):
        from control_api_adapter.vacuum_adapter import (
            GRIP,
            PlcCommandResult,
            PlcVacuumSnapshot,
            VacuumAdapterCore,
        )

        class FakeVacuumIo:
            def __init__(self):
                self.calls = []

            def set_output(self, output_name, enabled):
                self.calls.append((output_name, enabled))
                return PlcCommandResult(True, "")

            def read_snapshot(self):
                return PlcVacuumSnapshot(
                    connected=True,
                    data_fresh=True,
                    left_attached=False,
                    right_attached=False,
                    left_valve_open=False,
                    right_valve_open=False,
                    pump_enabled=False,
                    error="",
                )

            def wait_for_attachment(self, channels, timeout_s):
                raise AssertionError("unsupported profile must not wait for attachment")

        io = FakeVacuumIo()
        result = VacuumAdapterCore(io).execute_goal(GRIP, ["left"], "unknown", "pick")

        self.assertFalse(result.succeeded)
        self.assertFalse(result.accepted)
        self.assertIn("unsupported grip_profile_id", result.error)
        self.assertEqual(io.calls, [])

    def test_pump_disable_rejects_possible_load_held(self):
        from control_api_adapter.vacuum_adapter import (
            VacuumAdapterCore,
            PlcVacuumSnapshot,
            PumpErrorCode,
        )

        class FakeVacuumIo:
            def __init__(self):
                self.calls = []

            def set_output(self, output_name, enabled):
                self.calls.append((output_name, enabled))
                raise AssertionError("unsafe pump disable must not write PLC output")

            def read_snapshot(self):
                return PlcVacuumSnapshot(
                    connected=True,
                    data_fresh=True,
                    left_attached=True,
                    right_attached=False,
                    left_valve_open=True,
                    right_valve_open=False,
                    pump_enabled=True,
                    error="",
                )

            def wait_for_attachment(self, channels, timeout_s):
                raise AssertionError("not used by pump service")

        result = VacuumAdapterCore(FakeVacuumIo()).set_pump_enabled(False, "maintenance")

        self.assertFalse(result.accepted)
        self.assertEqual(result.error_code, PumpErrorCode.POSSIBLE_LOAD_HELD)
        self.assertIn("possible load", result.message)

    def test_safety_summary_uses_approved_sources(self):
        from control_api_adapter.status_adapter import (
            BatteryHealthSnapshot,
            ComponentSnapshot,
            PlcHealthSnapshot,
            build_safety_summary,
        )

        summary = build_safety_summary(
            enable_manager=ComponentSnapshot(
                name="/robot/rt_control/enable_manager",
                level=0,
                fresh=True,
                message="enabled",
                values={"state": "ENABLED", "stage": "success"},
            ),
            ethercat_master=ComponentSnapshot(
                name="/robot/rt_control/ethercat/master",
                level=0,
                fresh=True,
                message="healthy",
                values={},
            ),
            ethercat_slaves=[
                ComponentSnapshot(
                    name=f"/robot/rt_control/ethercat/slave_{index}",
                    level=0,
                    fresh=True,
                    message="healthy",
                    values={},
                )
                for index in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15)
            ],
            canopen_nodes=[
                ComponentSnapshot(
                    name="/robot/rt_control/canopen/node_2",
                    level=0,
                    fresh=True,
                    message="healthy",
                    values={},
                ),
                ComponentSnapshot(
                    name="/robot/rt_control/canopen/node_3",
                    level=0,
                    fresh=True,
                    message="healthy",
                    values={},
                ),
            ],
            plc=PlcHealthSnapshot(connected=True, data_fresh=True, error=""),
            bms=BatteryHealthSnapshot(present=True, fresh=True),
        )

        self.assertTrue(summary.safe_to_start_motion)
        self.assertTrue(summary.control_enabled)
        self.assertEqual(summary.state, "READY")

        blocked = build_safety_summary(
            enable_manager=summary.sources["enable_manager"],
            ethercat_master=summary.sources["ethercat_master"],
            ethercat_slaves=summary.sources["ethercat_slaves"],
            canopen_nodes=[
                summary.sources["canopen_nodes"][0],
                ComponentSnapshot(
                    name="/robot/rt_control/canopen/node_3",
                    level=2,
                    fresh=True,
                    message="Fault",
                    values={},
                ),
            ],
            plc=summary.sources["plc"],
            bms=summary.sources["bms"],
        )

        self.assertFalse(blocked.safe_to_start_motion)
        self.assertFalse(blocked.canopen_ok)
        self.assertIn("/robot/rt_control/canopen/node_3: Fault", blocked.active_faults)


if __name__ == "__main__":
    unittest.main()
