# RT-Control composer

KKOZIA-5 / SPEC v1.1 M0b: nine Draft 7 schemas, offline YAML validation,
renderer API 1 contract, and synthetic A/B/C fixtures. This package provides
no compose implementation or launch entry. A0 approval is pending review.

The schemas install in `share/rt_control_composer/schema`. The validator uses
ament index lookup, never a source-tree fallback or network schema fetch.
Tests remain in the source tree and validate installed schemas. Protocol
packages consume schema resources but must not import composer Python code.

```bash
ros2 run rt_control_composer validate_config_files.py \
  --share-package rt_control_machine_config --schema manifest.schema.json \
  machines/Example/manifest.yaml
```

Use `--schema-package <owner>` for an installed owner-private schema. Inputs
are relative to the selected input package share. The validator rejects
symlink escape, absolute paths, parent traversal, duplicate/merge keys,
anchors/aliases, multiple documents, and non-finite numbers. Quote timestamps
so YAML does not turn them into non-JSON objects. This boundary is for machine
configuration and policies only; protocol profiles and model configuration
retain their owner loaders.

Common envelope definitions live in `module_envelope.schema.json`; each root
lists its own properties and rejects unknown top-level fields. `spec` is an
explicit owner-private object in module/calibration envelopes and must also
be checked by the selected owner's private schema. Device models require the
six model-level envelope fields plus `family`; machine and unit documents
add `robot_variant`, and module/calibration envelopes also require `module`.
Physical device properties are optional positive values or explicit `TBD`.

`x-share-relative-path` marks package-share path strings in schemas. The
validator checks these without reading referenced files; owner schemas must
mark private path fields too. Cross-package references retain their explicit
package owner. The supplied share root applies to the input package; external
package existence and cross-file reference closure remain M2d checks.

Schema success is structural validation, not runtime admission. In particular,
`draft`, `verified: false`, and `TBD` are representable. The protocol renderer
checks render-mode required facts, and M2d implements the approved readiness
policy. No schema default invents a hardware fact, enable batch or tolerance.

`render_result.schema.json` validates the composer's XML projection, not raw
Element objects. The projection implementation and actual renderer adapters
belong to M2d and subsequent protocol migration issues. See the installed
`rt_control_composer/renderer_contract.md` for that boundary.

The fixtures under `test/fixtures/sample_{a,b,c}` use package-share directory
layouts and synthetic facts. `test/fixture_index.yaml` binds each input to its
schema; `test/conformance/` holds request/result projection examples and the
future renderer suite contract. `test/golden/` is excluded from R5 (ADR-19).
The draft rename map is the only golden data; no output baseline was generated.
