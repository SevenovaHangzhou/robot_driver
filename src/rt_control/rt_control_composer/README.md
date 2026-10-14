# RT-Control composer

The package provides offline composition, nine installed Draft 7 schemas,
renderer API 1 validation, and frozen comparisons with the old entry points.
ROS 2 Humble and Python 3.10 are the supported runtime.

`compose()` accepts `robot_variant`, `machine_unit`, `physical_profile`,
`control_scope`, `runtime_dir`, `use_mock_hardware`, and `jtc_only`. It loads
machine data, deployment and explicit owner policies through package shares.
It returns a `Composed` handoff and writes only validated output files below
`runtime_dir`. It does not access devices or invoke preflight.

`preflight(composed)` invokes each selected owner's preflight once per merged
hardware system. Mock returns without invoking an owner. The caller must run
real preflight before constructing any ROS Node. This API is not authorization
to access hardware.

## Data and ownership

- Machine data has no code API dependency. Consumers read installed assets.
- `owner_refs[transport]` chooses the protocol package dynamically.
- The registry validates the shared request envelope and XML result projection.
- Protocol owners validate private module/calibration facts and render fragments.
- Enable batches and numerical policies come from their explicit owner files.
- Policy batches are flattened to ROS parameter names/sizes without changing order.
- Motion arrays retain policy order; the composer does not derive tolerances.
- The machine safety policy is returned unchanged in `Composed.safety_policy`.
  Migrated entries do not add new safety-policy parameters to old output files.

The manifest's optional `policy_refs` identify a package, file and schema.
Scope `render_mode` and `controller_order` identify the output view and explicit
activation order. Controller `role` selects integration wiring without plugin
name dispatch. Request `deployment` separates bus/host data from calibration.
Result `actuator_groups` and opaque `resource_claims` support cross-file binding
and duplicate-resource checks without inspecting protocol-private fields.

The shared definitions and their consumers are in
`rt_control_composer/renderer_contract.md`. Schema references stay inside the
installed store. No network resolver or source-tree fallback is provided.

## Validation and runtime admission

```bash
ros2 run rt_control_composer validate_config_files.py \
  --share-package rt_control_machine_config --schema manifest.schema.json \
  machines/Example/manifest.yaml
```

Use `--schema-package` for an owner-private schema. The machine/policy YAML
boundary rejects duplicate/merge keys, anchors/aliases, multiple documents,
non-finite numbers and package-share escape. Observation archives cannot be
referenced as runtime inputs. Protocol profiles retain their owner loader,
including permitted non-finite PDO defaults. Model files retain their owner
loader, including permitted anchors and merge keys.

Shape validation does not certify a real device. `draft`, `verified: false`
and `TBD` remain representable. On migrated scopes, the protocol owner applies
old entry checks plus its required fields. Manifest readiness is enforced only
for a manifest-validation request or a profile marked ready. Inactive draft
modules are not passed to a protocol renderer.

## Equivalence and verification limits

The six old outputs in `test/golden/` come from a separate, pinned old checkout.
They include runtime profiles, robot description, controller parameters and
spawner order. The manifest records source and output hashes and exact renames.
After freezing, implementation differences must be fixed in the implementation.

Canonical comparison permits XML canonicalization, YAML key sorting, the
runtime-directory placeholder, and complete-value/file renames in that map.
It does not remove fields, reorder elements/lists or normalize numeric values.
Real fixtures use isolated test calibration and do not certify production data.

The four runtime launch entries (including the independent JTC-only arm view)
now use the composer by default after the user's stop-point approval. The
`use_composer`, old `calibration_file` and old `head_can_config` switches are
removed. Machine/unit assets own runtime selection and calibration; the signal
gate retains its shutdown sequence and chooses only an entry point.

The old builders and their tests are retired after freezing all ten outputs.
Head protocol parameters are recorded as user-confirmed; the direction/model
zero and other missing hardware facts remain explicit. Real arm calibration
still rejects when unverified. Mock verification, CI and real hardware
validation remain separate in the domain records.
