# Draft 7 shared contract

The nine `*.schema.json` files form one installed schema set. Common envelope
and positive/TBD definitions live in `module_envelope.schema.json`. Each root
lists its fields explicitly; no envelope uses `allOf`. `format` is not used.

Closed records use `additionalProperties: false`. Named dictionaries use
`patternProperties` or a typed value schema. The explicit owner-private
`spec`, robot-model limit mapping and expected topology are opaque here:
the protocol/model owner must validate those before runtime admission.

Device envelopes contain six model-level fields; `robot_variant` applies to
machine/unit documents, and `module` also applies to module/calibration files.
These distinctions keep device model files independent of a particular machine.

`render_result` describes the validation projection of ET.Element nodes,
including optional GPIOs. It does not redefine the Python return type. The
composer-side projection and raw-element checks are M2d responsibilities.
Sensor children exclude command interfaces. All node attributes and children
remain visible to schema validation; no filtering is allowed.

`x-share-relative-path` is an explicit extension checked by the supplied
validator, including symlink containment in the input package share. It is not
a JSON Schema `format`. Private owner schemas must annotate path fields too.
Only installed sibling schema files and fragments are supported by the offline
resolver. URI scopes (`$id`), network references and source-tree fallback are
rejected. Schema validation alone does not check cross-file closure, ownership,
unique addresses, real readiness or required physical facts for a render mode.
