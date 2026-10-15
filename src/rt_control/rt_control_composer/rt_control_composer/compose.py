"""Compose offline. Protocol owners own private facts; policies own all safety values."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import uuid

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

from .bindings import bind_parameters
from .model import Composed
from .renderer_registry import render_checked, select_renderer
from .schema.cross_checks import (
    enable_partition,
    manifest_links,
    merge_systems,
    motion_binding,
    runtime_admission,
)
from .schema.manifest_load import load_machine
from .schema.primitives import load_and_validate, load_yaml, resolve_share_path


def compose(
    *,
    robot_variant,
    machine_unit,
    physical_profile,
    control_scope,
    runtime_dir,
    use_mock_hardware=True,
    jtc_only=False,
    require_manifest_ready=False,
    package_share=get_package_share_directory,
    sync_mode="",
    startup_id=None,
):
    """All filesystem writes belong here; no bus command or preflight runs here."""
    runtime = Path(runtime_dir)
    integration_share = Path(package_share("rt_control_composer"))
    schemas = integration_share / "schema"
    config_root = Path(package_share("rt_control_machine_config"))
    data = load_machine(
        config_root, schemas, robot_variant, machine_unit, physical_profile, control_scope
    )
    manifest_links(data, config_root)
    runtime_admission(data, use_mock_hardware, require_manifest_ready)
    manifest, scope, deployment = data["manifest"], data["scope"], data["deployment"]
    mode = scope["render_mode"]
    model = manifest["robot_model"]
    model_share = Path(package_share(model["package"]))
    variant = model["variants"][deployment["end_effector"]]
    if use_mock_hardware and "mock_robot_model" in scope:
        variant = scope["mock_robot_model"]
        model_share = Path(package_share(variant["package"]))
    # Model files are loaded by their owner convention: anchors and merge keys are valid here.
    model_robot = ET.fromstring(
        xacro.process_file(str(model_share / variant["xacro_file"])).toxml()
    )
    robot_model = {
        "joints": [joint.attrib["name"] for joint in model_robot.findall("joint")],
        "joint_limits": yaml.safe_load((model_share / variant["joint_limits_file"]).read_text()),
    }
    if "robot_name" in scope:
        model_robot.attrib["name"] = scope["robot_name"]
    naming = load_yaml(integration_share / "config/interface_naming.yaml")
    if "minimal_robot" in scope:
        robot = ET.Element("robot", name=scope["minimal_robot"]["name"])
        ET.SubElement(robot, "link", name=scope["minimal_robot"]["base_link"])
    elif mode == "stationary_enable":
        robot = ET.Element(
            "robot", name=naming["stationary_robot_name"].format(robot_variant=robot_variant)
        )
        ET.SubElement(robot, "link", name=naming["stationary_base_link"])
    else:
        robot = model_robot
    rendered = []
    bus_rates = []
    rendered_modules = {}
    for name, machine in data["modules"].items():
        module = manifest["modules"][name]
        transport = module["transport"]
        renderer = select_renderer(manifest["owner_refs"], transport)
        layout = manifest["layout"][transport]
        bus_cycle = (
            layout["cycle_hz"] if "cycle_hz" in layout else 1_000_000 / layout["sync_period_us"]
        )
        bus_cycle = scope.get("ethercat_cycle_hz_override", bus_cycle)
        bus_rates.append(bus_cycle)
        request = {
            "module": module,
            "machine": machine,
            "devices": data["devices"],
            "unit": data["calibrations"][name],
            "deployment": deployment,
            "robot_model": robot_model,
            "use_mock_hardware": use_mock_hardware,
            "bus_cycle_hz": bus_cycle,
            "runtime_dir": str(runtime),
            "render_mode": mode,
            "hardware_system_id": scope.get("hardware_system_id", machine["master_system"]),
            "transport_layout": layout,
            "peer_machines": [
                peer
                for peer_name, peer in data["modules"].items()
                if manifest["modules"][peer_name]["transport"] == transport
                and peer.get("master_system") == machine.get("master_system")
            ],
        }
        if sync_mode:
            request["sync_mode"] = sync_mode
        result = render_checked(renderer, request, schemas, config_root)
        if "hardware_system_id" in scope:
            result = {**result, "hardware_system_id": scope["hardware_system_id"]}
        rendered.append((manifest["owner_refs"][transport], result))
        rendered_modules[name] = result
    systems = merge_systems(rendered)
    managed = [name for system in systems for name in system["result"]["managed_joints"]]
    runtime_files = {}
    for system in systems:
        result = system["result"]
        element = ET.SubElement(
            robot, "ros2_control", name=result["hardware_system_id"], type="system"
        )
        hardware = ET.SubElement(element, "hardware")
        ET.SubElement(hardware, "plugin").text = result["hardware_plugin"]
        for key, value in result["hardware_params"].items():
            ET.SubElement(hardware, "param", name=key).text = str(value)
        for key in ("joints", "sensors", "gpios"):
            for fragment in result.get(key, []):
                if (
                    key == "joints"
                    and mode != "stationary_enable"
                    and fragment.attrib["name"] not in robot_model["joints"]
                ):
                    raise ValueError(f"Unknown model joint: {fragment.attrib['name']}")
                element.append(deepcopy(fragment))
        for path, text in result["runtime_files"].items():
            if path in runtime_files:
                raise ValueError(f"Duplicate runtime path: {path}")
            runtime_files[path] = text

    def policy(name):
        reference = manifest["policy_refs"][name]
        share = Path(package_share(reference["package"]))
        path = resolve_share_path(share, reference["file"])
        return load_and_validate(path, reference["schema"], share / "schema", share)

    policies = scope.get(
        "policies", ["enable", "motion"] if mode != "stationary_enable" else ["enable"]
    )
    selected_enable = view = None
    if "enable" in policies:
        enable = policy("enable")
        view = enable["runtime_views"][mode]
        selected_enable = {**enable, **view}
        enable_partition(selected_enable, managed)
    motion = policy("motion") if "motion" in policies else None
    order = deepcopy(scope["controller_order"])
    bindings = {entry["name"]: entry for entry in data["controllers"].values()}
    if jtc_only:
        order["inactive"] = [
            name for name in order["inactive"] if bindings[name]["role"] != "rolling"
        ]
    selected_names = order["inactive"] + order["active"]
    if startup_id is None and any(
        bindings[name]["role"] == "sensor_broadcaster" for name in selected_names
    ):
        startup_id = "force-bench-" + str(uuid.uuid4())
    defaults = load_yaml(integration_share / "config/controller_defaults.yaml")
    manager = {
        "update_rate": bus_rates[0],
        **defaults["controller_manager"],
    }
    controllers = {"controller_manager": {"ros__parameters": manager}}
    trajectory_names = [name for name in selected_names if bindings[name]["role"] == "trajectory"]
    rolling_names = [name for name in selected_names if bindings[name]["role"] == "rolling"]
    if motion is not None:
        trajectory_binding = bindings[trajectory_names[0]]
        group_result = rendered_modules[trajectory_binding["module"]]
        bound = [
            joint
            for group in trajectory_binding["actuator_groups"]
            for joint in group_result["actuator_groups"][group]
        ]
        motion_binding(motion, bound)
    for name in selected_names:
        binding = bindings[name]
        manager[name] = {"type": binding["plugin"]}
        role = binding["role"]
        if role == "enable":
            parameters = {key: selected_enable[key] for key in view["parameter_names"]}
            parameters.update(
                managed_joints=selected_enable["managed_joints"],
                enable_batch_joint_names=[
                    joint for batch in selected_enable["enable_batches"] for joint in batch
                ],
                enable_batch_sizes=[len(batch) for batch in selected_enable["enable_batches"]],
                enable_only=mode == "stationary_enable",
                jtc_name=trajectory_names[0] if trajectory_names else "",
            )
            if motion is not None:
                parameters.update(
                    motion_mode_switching=bool(rolling_names),
                    motion_controller_names=trajectory_names + rolling_names,
                    default_motion_controller=trajectory_names[0],
                    rolling_motion_controller=rolling_names[0] if rolling_names else "",
                    motion_joints=list(motion["joints"]),
                    **deepcopy(motion["enable_manager"]),
                )
        elif role == "trajectory":
            parameters = {"joints": list(motion["joints"]), **deepcopy(motion["jtc"])}
        elif role == "rolling":
            reference = manifest["policy_refs"]["envelope"]
            share = Path(package_share(reference["package"]))
            envelope = resolve_share_path(share, reference["file"])
            runtime_files[envelope.name] = envelope.read_text()
            parameters = {
                **deepcopy(motion["rolling"]),
                "envelope_file": str(runtime / envelope.name),
            }
        elif role in ("joint_state", "internal_state"):
            parameters = deepcopy(defaults[role][mode])
            if role == "joint_state" and motion is not None:
                parameters["joints"] = list(motion["joints"])
        elif role == "sensor_broadcaster":
            owner_result = rendered_modules[binding["module"]]
            parameters = deepcopy(owner_result["controller_parameters"][name])
            context = owner_result["controller_context"][name]
            parameters.update(
                wrench_topic=naming["wrench"].format(**context),
                raw_topic=naming["raw"].format(**context),
                sensor_state_topic=naming["state"].format(**context),
                diagnostic_name=naming["sensor_calibration"].format(**context),
                calibration_topic=naming["family_calibration"].format(**context),
                startup_id=startup_id,
            )
        elif role == "external":
            share = Path(package_share(binding["package"]))
            source = yaml.safe_load(resolve_share_path(share, binding["config_file"]).read_text())
            parameters = deepcopy(source[name]["ros__parameters"])
        else:
            raise ValueError(f"Unsupported controller binding role: {role}")
        controllers[name] = {"ros__parameters": parameters}
    if scope.get("preserve_controller_file", False):
        binding = next(
            bindings[name] for name in selected_names if bindings[name]["role"] == "external"
        )
        share = Path(package_share(binding["package"]))
        controllers = yaml.safe_load(resolve_share_path(share, binding["config_file"]).read_text())
        controllers["controller_manager"]["ros__parameters"]["update_rate"] = bus_rates[0]
        if "parameter_view" in scope:
            for name, values in defaults["parameter_views"][scope["parameter_view"]].items():
                controllers[name]["ros__parameters"].update(deepcopy(values))
    elif "minimal_robot" in scope:
        manager.pop("thread_priority", None)
    parameter_bindings = scope.get("controller_parameter_bindings", {})
    if not set(parameter_bindings).issubset(selected_names):
        raise ValueError("Parameter binding references an unselected controller")
    for name in selected_names:
        controllers[name]["ros__parameters"] = bind_parameters(
            parameter_bindings.get(name, {}), controllers[name]["ros__parameters"],
            rendered_modules, mock=use_mock_hardware,
        )
    # fault_dependencies remain owned by the separate machine safety policy. These
    # migrated entry points did not emit them; validation does not add runtime fields.
    robot_description = ET.tostring(robot, encoding="unicode")
    # Validate all paths before writing any output, including collisions with launch outputs.
    reserved = {"controllers.yaml", "controller_order.yaml", "robot_description.xml"}
    if reserved.intersection(runtime_files):
        raise ValueError("Owner runtime path collides with an integration output")
    paths = {name: resolve_share_path(runtime, name) for name in runtime_files}
    runtime.mkdir(parents=True, exist_ok=True)
    for name, text in runtime_files.items():
        paths[name].parent.mkdir(parents=True, exist_ok=True)
        paths[name].write_text(text, encoding="utf-8")
    (runtime / "robot_description.xml").write_text(robot_description, encoding="utf-8")
    (runtime / "controllers.yaml").write_text(
        yaml.safe_dump(controllers, sort_keys=False), encoding="utf-8"
    )
    (runtime / "controller_order.yaml").write_text(
        yaml.safe_dump(order, sort_keys=False), encoding="utf-8"
    )
    return Composed(
        robot_description,
        controllers,
        order,
        runtime,
        systems,
        use_mock_hardware,
        data["safety_policy"],
    )


def preflight(composed, *, run=None):
    if composed.use_mock_hardware:
        return
    for system in composed.systems:
        renderer = select_renderer({"owner": system["owner"]}, "owner")
        renderer.preflight(system["result"]["expected_topology"], run=run)
