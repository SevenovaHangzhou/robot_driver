"""The real plc_io_modbus node against an in-process Modbus TCP server (no hardware)."""

import os
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
import robot_interfaces_qos
from robot_rt_control_interfaces.msg import SensorStatus, SensorStatusArray
from robot_rt_control_interfaces.srv import SetEnabled
from robot_system_interfaces.msg import ErrorCode
from sensor_msgs.msg import FluidPressure

PUMP, LEFT, RIGHT = 0, 1, 2
LEFT_AI, RIGHT_AI = 0, 1


class FakePlc:
    """FC01 read coils, FC05 write coil, FC04 read input registers; one shared memory map."""

    def __init__(self):
        self.coils = {PUMP: False, LEFT: False, RIGHT: False}
        self.registers = {LEFT_AI: 1000, RIGHT_AI: 1000}  # 0 kPa
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(4)
        self.port = self.listener.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                connection, _ = self.listener.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(connection,), daemon=True).start()

    def _serve(self, connection):
        with connection:
            while True:
                header = connection.recv(7, socket.MSG_WAITALL)
                if len(header) < 7:
                    return
                tid, _, length, unit = struct.unpack(">HHHB", header)
                pdu = connection.recv(length - 1, socket.MSG_WAITALL)
                function, address, value = pdu[0], *struct.unpack(">HH", pdu[1:5])
                if function == 1:
                    body = bytes([1, 1, int(self.coils[address])])
                elif function == 5:
                    self.coils[address] = value == 0xFF00
                    body = pdu[:5]
                elif function == 4:
                    body = bytes([4, 2]) + struct.pack(">H", self.registers[address])
                else:
                    body = bytes([function | 0x80, 1])
                connection.sendall(struct.pack(">HHHB", tid, 0, len(body) + 1, unit) + body)


# One node process for the module: a second DDS participant started right after the first exits
# is not reliably discovered in CI sandboxes.
@pytest.fixture(scope="module")
def plc():
    fake = FakePlc()
    parameters = {
        "digital.module.host": "127.0.0.1",
        "digital.module.port": fake.port,
        "analog.module.host": "127.0.0.1",
        "analog.module.port": fake.port,
        "vacuum_system.configured": True,
        "digital.outputs.vacuum_pump_relay.do_address": PUMP,
        "digital.outputs.left_vacuum_valve.do_address": LEFT,
        "digital.outputs.right_vacuum_valve.do_address": RIGHT,
        "analog.inputs.left_vacuum_sensor.register": LEFT_AI,
        "analog.inputs.left_vacuum_sensor.sensor_id": "ZSE30A-left",
        "analog.inputs.right_vacuum_sensor.register": RIGHT_AI,
        "analog.inputs.right_vacuum_sensor.sensor_id": "ZSE30A-right",
        "poll_period": 0.05,
    }
    arguments = [item for key, value in parameters.items() for item in ("-p", f"{key}:={value}")]
    executable = Path(get_package_prefix("periph_plc_io")) / "lib/periph_plc_io/plc_io_modbus"
    node = subprocess.Popen([str(executable), "--ros-args", *arguments], env=os.environ.copy())
    rclpy.init()
    try:
        peer = rclpy.create_node("plc_test_peer")
        pressures, statuses = {"left": [], "right": []}, []
        for side in pressures:
            peer.create_subscription(
                FluidPressure,
                f"/rt/vacuum/{side}/pressure",
                pressures[side].append,
                robot_interfaces_qos.state(),
            )
        peer.create_subscription(
            SensorStatusArray, "/rt/sensors/state", statuses.append, robot_interfaces_qos.state()
        )
        clients = {
            name: peer.create_client(SetEnabled, f"/rt/vacuum/{name}/set_enabled")
            for name in ("pump", "left_valve", "right_valve")
        }

        def call(name, enabled):
            future = clients[name].call_async(SetEnabled.Request(enabled=enabled, reason="test"))
            rclpy.spin_until_future_complete(peer, future, timeout_sec=5.0)
            return future.result()

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(peer, timeout_sec=0.02)

        assert all(client.wait_for_service(timeout_sec=10.0) for client in clients.values())
        spin(0.5)
        yield fake, call, spin, pressures, statuses
    finally:
        rclpy.shutdown()
        node.terminate()
        node.wait(timeout=10)


def test_pump_stop_waits_for_released_valves_and_pressures(plc):
    fake, call, spin, _, _ = plc
    reply = call("pump", True)
    assert reply.accepted and reply.enabled and fake.coils[PUMP]
    assert reply.error.code == ErrorCode.SUCCESS
    assert call("left_valve", True).accepted and fake.coils[LEFT]
    refused = call("pump", False)
    assert not refused.accepted and refused.enabled and fake.coils[PUMP]
    assert refused.error.code == ErrorCode.RT_POSSIBLE_LOAD_HELD
    assert call("left_valve", False).accepted
    fake.registers[RIGHT_AI] = 1100  # -2.5 kPa: right side still holds vacuum
    spin(0.3)
    assert call("pump", False).error.code == ErrorCode.RT_POSSIBLE_LOAD_HELD
    fake.registers[RIGHT_AI] = 1000
    spin(0.3)
    stopped = call("pump", False)
    assert stopped.accepted and not stopped.enabled and not fake.coils[PUMP]


def test_pressure_is_gauge_pascal_and_invalid_samples_only_reach_the_sensor_state(plc):
    fake, _, spin, pressures, statuses = plc
    pressures["left"].clear()
    fake.registers[LEFT_AI] = 3000  # (3000 - 1000) * -101 / 4000 = -50.5 kPa
    fake.registers[RIGHT_AI] = 100  # below valid_raw_min 800
    spin(0.5)
    pressures["right"].clear()
    statuses.clear()
    spin(0.5)
    assert pressures["left"][-1].fluid_pressure == pytest.approx(-50500.0)
    assert pressures["left"][-1].header.frame_id == "left_vacuum_sensor"
    assert pressures["right"] == []
    by_topic = {sensor.data_topic: sensor for sensor in statuses[-1].sensors}
    right = by_topic["/rt/vacuum/right/pressure"]
    assert right.measurement_state == SensorStatus.MEASUREMENT_INVALID
    assert right.error.code == ErrorCode.RT_SENSOR_INVALID and right.sensor_id == "ZSE30A-right"
    assert by_topic["/rt/vacuum/left/pressure"].measurement_state == SensorStatus.MEASUREMENT_VALID
    outputs = {sensor.sensor_id: sensor for sensor in statuses[-1].sensors}["vacuum_outputs"]
    assert outputs.measurement_state == SensorStatus.MEASUREMENT_VALID
