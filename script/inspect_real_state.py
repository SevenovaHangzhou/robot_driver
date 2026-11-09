#!/usr/bin/env python3
"""Read ROS feedback and controller status; never publish or change lifecycle.

Run with the target workspace's ROS environment. Domain must be explicit.
The only service request is controller_manager/ListControllers.
"""
import argparse
from datetime import datetime, timezone
from hashlib import sha256
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--stream', action='store_true', help='Continuously emit read-only JSON feedback')
    parser.add_argument('--period', type=float, default=.1)
    parser.add_argument('--urdf', type=Path, help='Artifact whose identity accompanies each sample')
    parser.add_argument('--controller-pid', type=int, help='Invalidate feedback if this runtime exits')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232 or not math.isfinite(args.seconds) or not 1 <= args.seconds <= 10:
        parser.error('Domain must be 0..232 and sampling seconds 1..10')
    if not math.isfinite(args.period) or not .1 <= args.period <= 1:
        parser.error('Period must be 0.1..1 seconds')

    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import JointState
    from control_msgs.msg import DynamicJointState
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(domain_id=args.domain_id)
    node = rclpy.create_node('cartesian_jog_readonly_preflight')
    observed, received = {}, {}
    controllers, services, model_hash, source_valid = [], [], None, True
    controllers_at, inspected_at, future = None, -math.inf, None

    def save(key, value):
        observed[key], received[key] = value, time.monotonic()

    try:
        def joints(message):
            save('positions', dict(zip(message.name, message.position)))
            save('velocities', dict(zip(message.name, message.velocity)))

        node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
        try:
            from robot_rt_control_interfaces.msg import RollingJointControlState
            from robot_interfaces_qos import rolling_state
            if not all(hasattr(RollingJointControlState(), key) for key in
                       ('epoch', 'state', 'last_sequence', 'last_result', 'stop_reason')):
                raise AttributeError('Current Rolling state fields are unavailable')
            node.create_subscription(
                RollingJointControlState, '/rt/arms/rolling/state',
                lambda m: save('rolling', {key: getattr(m, key) for key in
                              ('epoch', 'state', 'last_sequence', 'last_result', 'stop_reason')}),
                rolling_state())
        except (ImportError, AttributeError):
            # Older installed contracts remain observable without pretending
            # they support the current protocol.
            observed['rolling_contract'] = 'current_contract_unavailable'
        node.create_subscription(DynamicJointState, '/rt_internal_state_broadcaster/dynamic_joint_states',
                                 lambda m: save('hardware', {name: dict(zip(v.interface_names, v.values))
                                                            for name, v in zip(m.joint_names, m.interface_values)}), qos_profile_sensor_data)
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        deadline = time.monotonic() + args.seconds
        emitted_at = -math.inf
        while True:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if future is not None and future.done():
                response = future.result()
                if response:
                    controllers = [{'name': c.name, 'type': c.type, 'state': c.state,
                                    'claimed_interfaces': list(c.claimed_interfaces)} for c in response.controller]
                    controllers_at = now
                future = None
            if now - inspected_at >= 2:
                inspected_at = now
                services = [(name, kinds) for name, kinds in node.get_service_names_and_types()
                            if any(part in name for part in ('rolling', 'joint_control', 'set_control_mode', 'list_controllers'))]
                source_valid = True
                if args.urdf:
                    try:
                        model_hash = sha256(args.urdf.read_bytes()).hexdigest()
                        if args.controller_pid:
                            argv = Path(f'/proc/{args.controller_pid}/cmdline').read_bytes().split(b'\0')
                            source_valid = str(args.urdf.with_name('controllers.yaml')).encode() in argv
                    except OSError:
                        source_valid = False
                if future is not None and now - requested_at >= 2:
                    client.remove_pending_request(future)
                    future.cancel()
                    future = None
                if future is None and client.service_is_ready():
                    future, requested_at = client.call_async(ListControllers.Request()), now
            if (args.stream and now - emitted_at >= args.period) or (not args.stream and now >= deadline):
                result = {'domain_id': args.domain_id, 'readonly': True, 'controllers': controllers,
                          'controllers_age_s': None if controllers_at is None else now - controllers_at,
                          'services': services, 'model_sha256': model_hash, 'source_valid': source_valid,
                          'captured_at': datetime.now(timezone.utc).isoformat(), 'observed': observed,
                          'receive_age_s': {key: now - at for key, at in received.items()}}
                # JointState may carry NaN velocity when a driver has no velocity
                # channel. Preserve that as unavailable; never invent zero speed.
                def finite(value):
                    if isinstance(value, dict):
                        return {key: finite(v) for key, v in value.items()}
                    if isinstance(value, (list, tuple)):
                        return [finite(v) for v in value]
                    return None if isinstance(value, float) and not math.isfinite(value) else value

                print(json.dumps(finite(result), allow_nan=False), flush=True)
                emitted_at = now
                if not args.stream:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
