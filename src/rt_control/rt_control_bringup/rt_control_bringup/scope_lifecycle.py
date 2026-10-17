"""Generic scope lifecycle: bus locks, staged hardware activation and orderly teardown.

Everything is driven by the composer's lifecycle.yaml; nothing here knows a machine.
No bus command, enable or reset is issued except the public-equivalent /rt/disable.
"""

import fcntl
import os
from pathlib import Path
import tempfile
import time

import yaml

UNCONFIGURED, INACTIVE, ACTIVE = 1, 2, 3


def load_lifecycle(runtime_dir):
    return yaml.safe_load((Path(runtime_dir) / "lifecycle.yaml").read_text())


def lock_directory():
    return Path(os.environ.get("RT_CONTROL_LOCK_DIR", Path(tempfile.gettempdir()) / "rt-control"))


def acquire_bus_locks(names, directory=None):
    """Exclusive, non-blocking; the returned files must stay open for the process lifetime."""
    root = Path(directory) if directory is not None else lock_directory()
    root.mkdir(parents=True, exist_ok=True)
    held = []
    try:
        for name in sorted(names):
            stream = (root / f"{name}.lock").open("a")
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                stream.close()
                raise RuntimeError(f"bus already owned by another rt-control process: {name}")
            stream.truncate(0)
            stream.write(f"{os.getpid()}\n")
            stream.flush()
            held.append(stream)
    except Exception:
        for stream in held:
            stream.close()
        raise
    return held


def activate_hardware(backend, lifecycle):
    """Activate in composer order; a failure stops before any later (actuator) system."""
    for name in lifecycle["hardware_activation_order"]:
        if not backend.hardware_state(name, ACTIVE):
            raise RuntimeError(f"hardware activation failed: {name}")
    return True


def shutdown_sequence(backend, lifecycle, log=print):
    """Attempt every safe phase; a failed phase never becomes success."""
    results = []
    for label, operation in (
        ("motion_inactive", lambda: backend.quiesce(lifecycle["motion_controllers"])),
        ("drives_disabled", backend.disable),
        ("controllers_inactive", lambda: backend.quiesce(lifecycle["lifecycle_controllers"])),
    ):
        try:
            ok = bool(operation())
        except Exception as error:  # noqa: BLE001 - every phase is attempted
            log(f"RT_SCOPE_SHUTDOWN {label}: {error}")
            ok = False
        results.append(ok)
        log(f"RT_SCOPE_SHUTDOWN {label}={ok}")
    # Hardware is never deactivated while controllers may still claim its interfaces.
    if results[-1]:
        for name in reversed(lifecycle["hardware_activation_order"]):
            try:
                cleaned = backend.hardware_state(name, INACTIVE) and backend.hardware_state(
                    name, UNCONFIGURED
                )
            except Exception as error:  # noqa: BLE001
                log(f"RT_SCOPE_SHUTDOWN hardware {name}: {error}")
                cleaned = False
            results.append(bool(cleaned))
            log(f"RT_SCOPE_SHUTDOWN hardware={name} cleaned={bool(cleaned)}")
    else:
        results.append(False)
    return all(results)


class RosLifecycleBackend:
    """controller_manager and enable_manager service client with one global deadline."""

    def __init__(self, timeout=30.0, node_name="rt_scope_lifecycle_client"):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from controller_manager_msgs.srv import (
            ListControllers,
            ListHardwareComponents,
            SetHardwareComponentState,
            SwitchController,
        )
        from rt_control_interfaces.srv import RtEnable

        self.rclpy = rclpy
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(node_name)
        self.deadline = time.monotonic() + timeout
        self.services = {
            "list": (ListControllers, "/controller_manager/list_controllers"),
            "switch": (SwitchController, "/controller_manager/switch_controller"),
            "hardware": (
                SetHardwareComponentState,
                "/controller_manager/set_hardware_component_state",
            ),
            "list_hardware": (
                ListHardwareComponents,
                "/controller_manager/list_hardware_components",
            ),
            "disable": (RtEnable, "/rt/disable"),
        }
        self.clients = {
            key: self.node.create_client(kind, name) for key, (kind, name) in self.services.items()
        }

    def request(self, kind, request):
        client = self.clients[kind]
        while not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= self.deadline:
                raise TimeoutError(f"service unavailable: {kind}")
        future = client.call_async(request)
        while not future.done() and time.monotonic() < self.deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
        if not future.done():
            client.remove_pending_request(future)
            raise TimeoutError(f"service timeout: {kind}")
        if future.result() is None:
            raise RuntimeError(f"empty response: {kind}")
        return future.result()

    def list_controllers(self):
        return self.request("list", self.services["list"][0].Request()).controller

    def quiesce(self, names):
        active = [
            c.name for c in self.list_controllers() if c.name in names and c.state == "active"
        ]
        if not active:
            return True
        request = self.services["switch"][0].Request()
        request.deactivate_controllers = active
        request.strictness = 2
        request.activate_asap = False
        request.timeout.sec = 4
        return self.request("switch", request).ok

    def disable(self):
        if not any(
            item.name == "enable_manager" and item.state != "unconfigured"
            for item in self.list_controllers()
        ):
            return True  # Never configured: no drive can have been enabled by this scope.
        reply = self.request("disable", self.services["disable"][0].Request())
        print(f"RT_SCOPE_DISABLE ok={reply.ok} stage={reply.stage}", flush=True)
        return reply.ok

    def hardware_state(self, name, state):
        components = self.request(
            "list_hardware", self.services["list_hardware"][0].Request()
        ).component
        matches = [item for item in components if item.name == name]
        if len(matches) != 1:
            return False
        current = matches[0].state.id
        # Cleanup is also valid for components which never reached active.
        if current == state or (state == INACTIVE and current == UNCONFIGURED):
            return True
        request = self.services["hardware"][0].Request()
        request.name = name
        request.target_state.id = state
        request.target_state.label = {1: "unconfigured", 2: "inactive", 3: "active"}[state]
        result = self.request("hardware", request)
        return result.ok and result.state.id == state

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def hardware_start_main(argv=None):
    import sys

    runtime_dir = (argv if argv is not None else sys.argv[1:])[0]
    lifecycle = load_lifecycle(runtime_dir)
    backend = RosLifecycleBackend(timeout=120.0, node_name="rt_scope_hardware_start")
    try:
        activate_hardware(backend, lifecycle)
        print("RT_SCOPE_HARDWARE_ACTIVE_DISABLED", flush=True)
        return 0
    finally:
        backend.close()


def shutdown_main(argv=None):
    import sys

    runtime_dir = (argv if argv is not None else sys.argv[1:])[0]
    if not (Path(runtime_dir) / "lifecycle.yaml").is_file():
        print("RT_SCOPE_SHUTDOWN no lifecycle plan; nothing was started", flush=True)
        return 0
    lifecycle = load_lifecycle(runtime_dir)
    backend = RosLifecycleBackend(timeout=30.0, node_name="rt_scope_shutdown")
    try:
        return 0 if shutdown_sequence(backend, lifecycle) else 1
    finally:
        backend.close()
