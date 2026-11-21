"""Real ROS node + TCP peer: input edges, failure, staleness, reconnect and QoS."""

import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rt_control_interfaces.msg import EmergencyStopState as State


class Module:
    def __init__(self):
        self.mode = "hold"
        self.level = 1
        self.requests = []
        self.connections = 0
        self.closed = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.clients = []
        self.thread = threading.Thread(target=self.accept, daemon=True)
        self.thread.start()

    def accept(self):
        while not self.closed.is_set():
            try:
                conn, _ = self.listener.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            self.connections += 1
            self.clients.append(conn)
            threading.Thread(target=self.serve, args=(conn,), daemon=True).start()

    def serve(self, conn):
        with conn:
            conn.settimeout(2)
            try:
                while not self.closed.is_set():
                    header = conn.recv(7, socket.MSG_WAITALL)
                    if len(header) != 7:
                        return
                    tid, proto, length, unit = struct.unpack(">HHHB", header)
                    pdu = conn.recv(length - 1, socket.MSG_WAITALL)
                    self.requests.append((proto, unit, pdu))
                    mode = self.mode
                    if mode == "hold":
                        self.closed.wait(0.8)
                    if mode == "disconnect":
                        return
                    if mode == "malformed":
                        reply = bytes([2, 2, self.level, 0])
                    elif mode == "exception":
                        reply = bytes([0x82, 2])
                    else:
                        reply = bytes([2, 1, self.level])
                    conn.sendall(struct.pack(">HHHB", tid, 0, len(reply) + 1, unit) + reply)
            except OSError:
                return

    def close(self):
        self.closed.set()
        self.listener.close()
        for conn in self.clients:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(timeout=2)


def executable():
    return str(
        Path(get_package_prefix("periph_emergency_stop"))
        / "lib/periph_emergency_stop/emergency_stop_node"
    )


def test_missing_configuration_refuses_start():
    result = subprocess.run([executable()], capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert "port must be explicitly configured" in result.stderr


def test_real_node_states_and_read_only_protocol():
    module = Module()
    rclpy.init()
    peer = rclpy.create_node("emergency_stop_test_peer")
    topic = f"/test/emergency_stop_{os.getpid()}"
    qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    received = []
    peer.create_subscription(State, topic, received.append, qos)
    params = dict(
        host="127.0.0.1",
        port=module.port,
        unit_id=1,
        input_address=0,
        pressed_level=0,
        poll_period_ms=20,
        request_timeout_ms=500,
        stale_timeout_ms=120,
    )
    args = [arg for key, value in params.items() for arg in ("-p", f"{key}:={value}")]
    process = subprocess.Popen(
        [executable(), "--ros-args", *args, "-r", f"/plc/emergency_stop/state:={topic}"]
    )

    def wait_for(predicate, seconds=8):
        start = len(received)
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(peer, timeout_sec=0.02)
            for message in received[start:]:
                if predicate(message):
                    return message
            assert process.poll() is None
        pytest.fail(f"no matching state; received {[(m.state, m.detail) for m in received[-5:]]}")

    try:
        initial = wait_for(lambda m: not m.valid and m.sample_sequence == 0)
        assert initial.state == State.UNKNOWN and initial.stop_requested
        assert initial.sample_age_seconds == -1
        module.mode = "normal"
        released = wait_for(lambda m: m.valid and m.state == State.RELEASED)
        assert released.raw_level and not released.stop_requested
        assert released.sample_sequence > 0
        module.level = 0
        pressed = wait_for(lambda m: m.valid and m.state == State.PRESSED)
        assert not pressed.raw_level and pressed.stop_requested
        assert pressed.producer_instance_id == released.producer_instance_id
        module.level = 1
        wait_for(lambda m: m.valid and m.state == State.RELEASED)
        module.mode = "hold"
        stale = wait_for(lambda m: m.detail == "sample_stale")
        assert stale.state == State.UNKNOWN and stale.stop_requested and not stale.valid
        assert stale.sample_age_seconds >= 0.12
        timed_out = wait_for(lambda m: not m.valid and m.detail != "sample_stale")
        assert timed_out.state == State.UNKNOWN and timed_out.stop_requested
        assert timed_out.sample_sequence == stale.sample_sequence
        module.mode = "normal"
        recovered = wait_for(lambda m: m.valid and m.state == State.RELEASED)
        assert recovered.sample_sequence > stale.sample_sequence
        for failure in ("exception", "malformed", "disconnect"):
            connections = module.connections
            module.mode = failure
            error = wait_for(lambda m: not m.valid and m.detail != "sample_stale")
            assert error.state == State.UNKNOWN and error.stop_requested
            module.mode = "normal"
            wait_for(lambda m: m.valid and m.state == State.RELEASED)
            assert module.connections > connections
        # Every request must be FC02, address zero, count one. No writes or device commands.
        assert module.requests
        assert all(item == (0, 1, b"\x02\x00\x00\x00\x01") for item in module.requests)
        module.mode = "hold"
        wait_for(lambda m: m.detail == "sample_stale")
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=3) == 0
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        module.close()
        peer.destroy_node()
        rclpy.shutdown()


def test_late_join_receives_startup_cache_without_another_publication():
    module = Module()
    rclpy.init()
    peer = rclpy.create_node("emergency_stop_late_peer")
    topic = f"/test/emergency_stop_late_{os.getpid()}"
    # Only the constructor publishes within this test. The 60 s test timer is deliberately
    # much longer than discovery and reception; it is not a production polling setting.
    params = dict(
        host="127.0.0.1",
        port=module.port,
        unit_id=1,
        input_address=0,
        pressed_level=0,
        poll_period_ms=60000,
        request_timeout_ms=500,
        stale_timeout_ms=120,
    )
    args = [arg for key, value in params.items() for arg in ("-p", f"{key}:={value}")]
    process = subprocess.Popen(
        [executable(), "--ros-args", *args, "-r", f"/plc/emergency_stop/state:={topic}"]
    )
    started = time.monotonic()
    try:
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        first = []
        observer = peer.create_subscription(State, topic, first.append, qos)
        while not first:
            assert process.poll() is None
            assert time.monotonic() - started < 5, "startup publication not observed"
            rclpy.spin_once(peer, timeout_sec=0.02)
        # Discovery alone can precede publish(). Observe the first write, then replace the
        # reader, so the new subscription is definitely created after the cached write.
        peer.destroy_subscription(observer)
        created_ns = peer.get_clock().now().nanoseconds
        received = []
        peer.create_subscription(State, topic, received.append, qos)
        while not received:
            assert process.poll() is None
            assert time.monotonic() - started < 10, "cached startup sample not delivered"
            rclpy.spin_once(peer, timeout_sec=0.02)
        message = received[0]
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        assert message == first[0]
        assert stamp_ns < created_ns
        assert message.sample_sequence == 0 and message.state == State.UNKNOWN
        assert message.stop_requested and not message.valid
        assert not module.requests
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        module.close()
        peer.destroy_node()
        rclpy.shutdown()
