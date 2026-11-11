"""Select optional passive devices and remap one validated physical EtherCAT layout."""

import subprocess
from .optional_topology import resolve_sensor_topology


def select_startup(request, families):
    modules = request["modules"]
    declarations = request["declarations"]
    by_kind = {}
    for name, machine in modules.items():
        kind = families[machine["spec"]["family"]].get("startup_kind")
        if kind:
            by_kind.setdefault(kind, []).append(name)
    if len(by_kind.get("actuators", [])) != 1 or len(by_kind.get("passive_sensors", [])) > 1:
        raise ValueError("EtherCAT startup selection requires one declared actuator assembly")
    arm_name = by_kind["actuators"][0]
    force_name = next(iter(by_kind.get("passive_sensors", [])), None)
    arm = modules[arm_name]
    sensors = modules[force_name]["spec"]["sensors"] if force_name else []
    topology = next(
        system["result"]["expected_topology"]
        for system in request["systems"]
        if system["result"]["hardware_system_id"] == arm["master_system"]
    )
    scan = request.get("scan")
    if scan is None:
        if request["mock"]:
            raise ValueError("Mock startup selection requires an explicit EtherCAT scan")
        scan = subprocess.check_output(
            ["ethercat", "slaves", "-m", str(topology["master_id"]), "-v"], text=True, timeout=10
        )
    optional = (
        sensors
        if force_name and declarations[force_name]["startup"]["policy"] == "optional"
        else []
    )
    mapping, missing = resolve_sensor_topology(topology, optional, scan)
    choice = request["choices"].get(arm_name, "auto")
    if choice not in ("auto", "required"):
        raise ValueError("Required actuator assembly cannot be excluded")
    arm["spec"]["extra_responders"] = [
        item for item in arm["spec"]["extra_responders"] if item["ring_position"] not in missing
    ]
    for item in arm["spec"]["extra_responders"]:
        item["ring_position"] = mapping[item["ring_position"]]
    calibration = request["calibrations"][arm_name]
    for axis in calibration["spec"]["axes"]:
        axis["ring_position"] = mapping[axis["ring_position"]]
    rows = []
    for side, branch in arm["spec"]["branches"].items():
        for axis in branch["axes"]:
            axis["ring_position"] = mapping[axis["ring_position"]]
        domain = branch.get("pdo_domain")
        rows.append(
            _row(
                arm_name,
                side,
                declarations[arm_name],
                True,
                True,
                "present",
                domain,
                [axis["robot_model_joint"] for axis in branch["axes"]],
            )
        )
    arm["spec"]["topology_preflight"] = True
    removed = []
    excluded_controllers = []
    if force_name:
        force = modules[force_name]
        scope_sides = {s["side"] for s in sensors if s["side"] in arm["spec"]["branches"]}
        available = {
            s["side"]
            for s in sensors
            if s["side"] in scope_sides and s["ring_position"] not in missing
        }
        mode = request["choices"].get(force_name, "auto")
        selected = {"auto": available, "none": set(), "required": scope_sides}.get(
            mode, set(mode.split(","))
        )
        if not selected.issubset(available):
            raise ValueError(
                f"Requested sensors unavailable in this scope: {sorted(selected - available)}"
            )
        for sensor in sensors:
            side = sensor["side"]
            in_scope = side in arm["spec"]["branches"]
            reason = (
                "out_of_scope"
                if not in_scope
                else "absent"
                if sensor["ring_position"] in missing
                else "excluded"
                if side not in selected
                else "present"
            )
            rows.append(
                _row(
                    force_name,
                    side,
                    declarations[force_name],
                    sensor["ring_position"] not in missing,
                    side in selected,
                    reason,
                    sensor.get("pdo_domain"),
                    [],
                )
            )
            if side not in selected:
                excluded_controllers.append(sensor["controller_name"])
        force["spec"]["follow_arm_branches"] = False
        force["spec"]["sensors"] = [
            dict(s, ring_position=mapping[s["ring_position"]])
            for s in sensors
            if s["side"] in selected
        ]
        force["spec"]["bench_extra_responders"] = [
            mapping[p] for p in force["spec"]["bench_extra_responders"] if p in mapping
        ]
        if not selected:
            removed.append(force_name)
    return dict(
        modules=modules,
        calibrations={arm_name: calibration},
        removed=removed,
        excluded_controllers=excluded_controllers,
        rows=rows,
        position_mapping=mapping,
        scan=scan,
    )


def _row(name, component, declaration, present, selected, reason, domain, joints):
    startup = declaration["startup"]
    return dict(
        module=name,
        component=component,
        option=startup.get("option", ""),
        label=startup.get("labels", {}).get(component, startup["label"]),
        policy=startup["policy"],
        present=present,
        selected=selected,
        reason=reason,
        domain=domain,
        joints=joints,
        health_interfaces=[
            f"ethercat_domain_{domain}/data_valid",
            f"ethercat_domain_{domain}/process_data_age_ms",
        ]
        if domain is not None
        else [],
    )
