"""Compose offline. Protocol owners own private facts; policies own all safety values."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

from .bindings import bind_parameters
from .model import ARTIFACT, Composed
from .parameter_sources import controller_parameters
from .renderer_registry import render_checked, select_renderer
from .schema.cross_checks import (
    manifest_links,
    merge_systems,
    resolve_enable_groups,
    runtime_admission,
)
from .schema.manifest_load import load_machine
from .schema.primitives import load_and_validate, load_yaml, resolve_share_path


RECORDER_NAME = "runtime_log_recorder"
RECORDER_PLUGIN = "runtime_log_recorder/RecorderController"
ENABLE_MANAGER_PLUGIN = "enable_manager/EnableManagerController"


def recorded_state_interfaces(robot):
    """Every state interface of every ros2_control system, in description order."""
    names = []
    for system in robot.findall("ros2_control"):
        for element in system:
            if element.tag in ("joint", "sensor", "gpio"):
                names.extend(
                    f"{element.attrib['name']}/{state.attrib['name']}"
                    for state in element.findall("state_interface")
                )
    return names


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
    """Pure composition: returns every output file as text; no writes, bus command or preflight."""
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
    # Lifecycle "active" of a sensor system is a precondition for actuators, never readiness.
    hardware_order = [
        system["result"]["hardware_system_id"]
        for state_only in (True, False)
        for system in systems
        if (not system["result"]["managed_joints"]) == state_only
    ]
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
        schema_share = Path(package_share(reference.get("schema_package", reference["package"])))
        path = resolve_share_path(share, reference["file"])
        return load_and_validate(path, reference["schema"], schema_share / "schema", share)

    selected_policies = scope.get("policies", [])
    overrides = scope.get("policy_overrides", {})
    policies = {name: policy(overrides.get(name, name)) for name in selected_policies}
    for loaded in policies.values():
        if "enable_groups" in loaded:
            resolve_enable_groups(loaded, manifest["modules"], rendered_modules, managed)
    order = deepcopy(scope["controller_order"])
    bindings = {entry["name"]: entry for entry in data["controllers"].values()}
    if jtc_only:
        if "jtc_only_excluded" not in scope:
            raise ValueError("The selected scope does not declare a jtc_only view")
        order["inactive"] = [
            name for name in order["inactive"] if name not in scope["jtc_only_excluded"]
        ]
    selected_names = order["inactive"] + order["active"]
    unknown = set(selected_names).difference(bindings)
    if unknown:
        raise ValueError(f"Controller order references unknown controllers: {sorted(unknown)}")
    defaults = load_yaml(integration_share / "config/controller_defaults.yaml")
    manager = {
        "update_rate": bus_rates[0],
        **defaults["controller_manager"],
    }
    staged = bool(scope.get("staged_hardware_activation", False))
    if staged:
        # The scope launcher activates each system in hardware_order after its owner preflight.
        manager["hardware_components_initial_state"] = {"unconfigured": list(hardware_order)}
    controllers = {"controller_manager": {"ros__parameters": manager}}
    runtime_parameters = {}
    environment = {
        "context": {"mode": mode},
        "policies": policies,
        "policy_refs": manifest.get("policy_refs", {}),
        "package_share": package_share,
        "rendered_modules": rendered_modules,
        "selected": selected_names,
        "tags": {name: bindings[name].get("tags", []) for name in selected_names},
        "naming": naming,
        "runtime_values": {"startup_id": startup_id},
        "runtime_parameters": runtime_parameters,
        "runtime_dir": str(runtime),
        "runtime_files": runtime_files,
        "managed_joints": managed,
        "state_ready": [
            f"{sensor.attrib['name']}/ready"
            for module in scope["required_state_modules"]
            for sensor in rendered_modules[module].get("sensors", [])
            if sensor.find("state_interface[@name='ready']") is not None
        ],
    }
    for name in selected_names:
        binding = bindings[name]
        manager[name] = {"type": binding["plugin"]}
        environment["context"] = {"mode": mode, "controller": name}
        controllers[name] = {"ros__parameters": controller_parameters(name, binding, environment)}
    if "controller_file" in scope:
        reference = scope["controller_file"]
        share = Path(package_share(reference["package"]))
        controllers = yaml.safe_load(resolve_share_path(share, reference["file"]).read_text())
        controllers["controller_manager"]["ros__parameters"]["update_rate"] = bus_rates[0]
        runtime_parameters.clear()
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
            parameter_bindings.get(name, {}),
            controllers[name]["ros__parameters"],
            rendered_modules,
            mock=use_mock_hardware,
        )
    # fault_dependencies remain owned by the separate machine safety policy. These
    # migrated entry points did not emit them; validation does not add runtime fields.
    recorders = []
    recorder = deployment.get("runtime_log_recorder", {})
    if recorder.get("enabled") and staged:
        # Optional, outside controller_order: a missing plugin or failed load never blocks the scope.
        recorders = [RECORDER_NAME]
        controllers["controller_manager"]["ros__parameters"][RECORDER_NAME] = {
            "type": RECORDER_PLUGIN
        }
        controllers[RECORDER_NAME] = {
            "ros__parameters": {
                "interfaces": recorded_state_interfaces(robot),
                "slots": recorder.get("slots", 2000),
            }
        }
        # The enable manager writes its own state-machine trace next to the recorder segment.
        for name in selected_names:
            if bindings[name]["plugin"] == ENABLE_MANAGER_PLUGIN:
                controllers[name]["ros__parameters"]["runtime_log_trace"] = True
    robot_description = ET.tostring(robot, encoding="unicode")
    # Paths are validated here; writing belongs to the explicit deploy-time artifact step.
    reserved = {
        "controllers.yaml",
        "controller_order.yaml",
        "robot_description.xml",
        "lifecycle.yaml",
        ARTIFACT,
    }
    if reserved.intersection(runtime_files):
        raise ValueError("Owner runtime path collides with an integration output")
    for name in runtime_files:
        resolve_share_path(runtime, name)
    files = {
        **runtime_files,
        "robot_description.xml": robot_description,
        "controllers.yaml": yaml.safe_dump(controllers, sort_keys=False),
        "controller_order.yaml": yaml.safe_dump(order, sort_keys=False),
    }
    params_of = [s["result"]["hardware_params"] for s in systems]
    lifecycle = {
        "control_scope": control_scope,
        "use_mock_hardware": use_mock_hardware,
        "staged_hardware_activation": staged,
        "hardware_activation_order": list(hardware_order),
        "motion_controllers": list(order["inactive"]),
        "lifecycle_controllers": list(order["active"]),
        **({"recorder_controllers": recorders} if recorders else {}),
        # One process owns an IgH master or a CAN interface at a time (supervisor locks).
        "bus_locks": sorted(
            {f"ethercat-master-{p['master_id']}" for p in params_of if "master_id" in p}
            | {f"can-{p['can_interface_name']}" for p in params_of if "can_interface_name" in p}
        ),
        # Operator-visible capabilities; the scope runtime publishes them on /rt/capabilities.
        "capabilities": [
            {**deepcopy(item), "controller": name}
            for name in selected_names
            for item in bindings[name].get("capabilities", [])
        ]
        + (
            [{"name": "enable", "service": "/control/set_enabled", "type": "SetControlEnabled"}]
            if any("enable" in bindings[name].get("tags", []) for name in selected_names)
            else []
        ),
        "hardware": [
            {"name": s["result"]["hardware_system_id"], "plugin": s["result"]["hardware_plugin"]}
            for s in systems
        ],
        # Realtime CPU layout and identity of this unit's host (deployment data).
        "host": deepcopy(deployment.get("host", {})),
        "transports": sorted({system["owner"] for system in systems}),
    }
    if staged:
        # Consumed by the generic scope launcher and its orderly shutdown.
        files["lifecycle.yaml"] = yaml.safe_dump(lifecycle, sort_keys=False)
    return Composed(
        robot_description,
        controllers,
        order,
        runtime,
        systems,
        use_mock_hardware,
        data["safety_policy"],
        files,
        runtime_parameters,
        tuple(hardware_order),
        staged,
        lifecycle,
    )


def preflight(composed, *, run=None):
    if composed.use_mock_hardware:
        return
    for system in composed.systems:
        renderer = select_renderer({"owner": system["owner"]}, "owner")
        renderer.preflight(system["result"]["expected_topology"], run=run)
