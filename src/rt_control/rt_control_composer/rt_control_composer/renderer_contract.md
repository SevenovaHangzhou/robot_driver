# Protocol renderer contract draft — API 1

Source: KKOZIA-5, SPEC v1.1 §5.3. This draft defines the future Mapping boundary;
it provides no renderer and authorizes no hardware access.

Each protocol owner exports `<owner_package>.rt_control_renderer`. The registry
selects the owner using manifest `owner_refs[transport]`; transport names are
data, not an enumeration. Protocol packages must not import the composer.
Composer-private models must not cross this boundary.

```python
from typing import Mapping

RENDERER_API_VERSION = 1

def validate_module(
    module: Mapping[str, object], data: Mapping[str, object]
) -> Mapping[str, object]: ...

def validate_unit(
    module: Mapping[str, object], data: Mapping[str, object] | None
) -> Mapping[str, object]: ...

def validate_device(
    data: Mapping[str, object], *, render_mode: str
) -> Mapping[str, object]: ...

def render(request: Mapping[str, object]) -> Mapping[str, object]: ...

def preflight(
    expected_topology: Mapping[str, object], *, run=None
) -> None: ...
```

Reject API versions other than 1 before invoking a renderer. Find shared and
owner-private schemas through the ament index in their installed share
directories. Resolve references only within approved installed schema
directories, never through the network or a source-tree fallback.

## Validation

`validate_module` validates owner-specific module data using its private schema.
`validate_unit` checks calibration; `None` is valid only if the module requires
no calibration. Return a Mapping on success and raise on failure. Do not mutate
the caller's mappings or substitute guessed hardware facts.

`validate_device` declares and checks facts required by `render_mode`. On the
real path, required `TBD` or false verification is an error. Non-required facts
may remain `TBD`; the composer does not choose this field set for the owner.
Output-position control requires positive finite output encoder counts per
revolution. It must not require a reduction ratio for output-position scaling.

Mock selection is explicit in the request. No validation function probes a bus
to fill missing facts. Global manifest/profile readiness remains M2d work;
this document does not introduce an additional global admission gate.

## Request and result

The required request keys are `module`, `machine`, `devices`, `unit`,
`robot_model` (including `joint_limits`), `use_mock_hardware`, `bus_cycle_hz`,
`runtime_dir`, and `render_mode` (`normal` or `stationary_enable`). `sync_mode`
and `deployment` are optional. `deployment` references the existing
`unit_deployment.schema.json` and carries host/bus deployment facts separately
from calibration. Owners that require a deployed bus consume this Mapping.
`unit` may be `None` only when unnecessary. `bus_cycle_hz` already
includes an explicit scope override. `runtime_dir` only generates path strings.

`hardware_system_id` is optionally supplied in the request after scope selection,
so the protocol owner can distinguish a bench topology from an installed system.
The result still carries the selected system identity.

Optional result `controller_parameters` carries owner-specific sensor parameter
values. Integration adds topics/startup identity and controller bindings. No
protocol owner declares ROS launch nodes or activation order.

The renderer's head CAN parameters preserve the expanded old Xacro fields;
there is no EtherCAT-style `control_frequency` parameter in that system. The
shared hardware parameter schema therefore does not require that field for all
transports. EtherCAT owners continue to supply it from `bus_cycle_hz`.

The result requires all of:

| Key | Meaning |
| --- | --- |
| `hardware_system_id` | Module `master_system`, with explicit scope override |
| `hardware_plugin` | Owner plugin identifier |
| `hardware_params` | Protocol-owned parameters; EtherCAT includes `control_frequency = bus_cycle_hz`; head CAN keeps its old parameter set |
| `joints` | Ordered standard-library `ElementTree.Element` list |
| `sensors` | Ordered standard-library `ElementTree.Element` list |
| `gpios` (optional) | Ordered standard-library `ElementTree.Element` list; absent means empty |
| `runtime_files` | Relative path to text mapping; no writes |
| `managed_joints` | Axes exposing `control_word`; EtherCAT ring order, otherwise declared input order |
| `expected_topology` | Owner topology for separate read-only preflight |

Optional result `actuator_groups` maps declared group names to ordered resource
names. Optional `resource_claims` contains opaque owner-generated strings that
must be unique within a hardware system. These fields allow X7b and X3/X9
without integration inspecting owner-private device fields. A renderer used by
policy composition supplies both fields; older projection fixtures may omit them.

Manifest `policy_refs` names owner package/file/schema data. Scope `render_mode`
and `controller_order` select explicit output views. Controller `role` selects
integration wiring without plugin-name dispatch. These optional shared fields
express existing selection and policy facts; they do not derive new policies.

The composer registry validates the pure Mapping request against
`render_request.schema.json` once before invoking the selected owner. The owner
validates its private module, calibration and required device facts. This keeps
the shared envelope at the integration boundary without a protocol dependency
on composer code. The composer alone projects result Elements to
`{tag, attrib, text, children}` before validating `render_result.schema.json`.
M2d implements this in `renderer_registry.py`; conformance reuses that function.
Protocol packages must neither implement the projection nor import composer.

Check raw values before projecting: require standard-library ET.Element objects,
reject namespaced tags, non-whitespace `tail`, and non-whitespace `text` outside
`param`. Copy every attribute and child in original order; never filter unknown
nodes or attributes into an apparently valid result. Preserve `param` text;
empty param text projects to an empty string. Whitespace-only other text may
remain a string or None. Nonempty tail is not discarded before checking.

The result schema defines `xml_param`, `xml_interface`, `xml_joint`,
`xml_sensor`, and `xml_gpio`. Only the `name` attribute is allowed and required.
Joint/gpio children are interface or param; sensor children are state_interface
or param (never command_interface); interface children are param; param has no
children. The optional `gpios` list preserves GPIO hardware declarations
(ADR-20). Read-only sensors remain outside `managed_joints`.

For non-EtherCAT modules, `managed_joints` follows the declared machine input
order after selecting axes exposing `control_word`. Do not invent ring positions
for other transports. Policy/batch order is independent and stays explicit.

Runtime paths must not be absolute or contain `..`. Render neither writes files
nor accesses devices. The composer writes `runtime_files` after validation.
Neither reads the observation archive named `evidence/`.

Equal system IDs merge only if plugin, master number, cycle and timeout agree.
The composer aggregates topology and preflights each master once. Renderers
must not adjust conflicting parameters to permit merging. Enable batches and
tolerances always come from explicit policies, never renderer derivation.

## Conversion and effects

For output counts per revolution `Nout`:

```text
command_counts = zero_counts + direction * q_rad * Nout / (2 * pi)
feedback_rad = direction * (actual_counts - zero_counts) * (2 * pi) / Nout
```

Reduction ratio enters neither expression. Motor-side conversion and protocol
scales remain owner-specific and sourced. This skeleton implements no formula
and changes no rounding or overflow behavior in pinned drivers.

`preflight` is separate from `render`. It runs only approved read-only checks
and raises on mismatch; `run` allows fake command results in T0/T1 tests. Mock
and offline export must not invoke it. A render request is not authorization
for hardware preflight. This task performs no hardware preflight.

See `test/conformance/README.md` for the future owner adapters. The projection schema is tested in M0b; actual renderer and raw Element
projection conformance remain M2d/protocol migration work.
