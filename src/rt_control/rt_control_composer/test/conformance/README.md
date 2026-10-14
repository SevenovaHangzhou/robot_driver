# Renderer conformance fixture skeleton

No renderer implementation exists in M0b, and none has passed this suite.
`render_request.yaml` and `render_result.yaml` are executable shared-schema
positives in the package tests; the latter is an XML validation projection.
M2d supplies raw Element projection and the registry; protocol issues supply
renderer adapters. The suite discovers adapters with importlib within the
composer test process; protocol packages do not depend on composer.
Synthetic source inputs live in `../fixtures/`. They are not hardware facts.

A future owner adapter supplies the renderer module, a request constructed
using its private schema, the render-mode required field set, and an explicitly
synthetic real-path request. It identifies a required field to replace with
`TBD`, a required verification to set false, and the output-position conversion
to compare after changing only the device reduction ratio.

| Future parameterized case | Required result |
| --- | --- |
| Legal synthetic request | Valid result shape and allowed XML fragment |
| Real request with required `TBD` | Exception before a usable result |
| Real request with required verification false | Exception before a usable result |
| Mock request | No device access and no preflight |
| Render under write-denying hooks | No write attempt; content in `runtime_files` |
| Render with archive reads trapped | No observation archive read |
| Import with composer import denied | Owner imports without composer |
| Output encoder reduction ratio changed | Both position conversions unchanged |
| Raw Element with non-whitespace tail, namespace or illegal text | Composer projection rejects without dropping content (M2d) |
| Unsupported API version | Registry rejects before invocation (M2d) |
| Fake preflight mismatch | Exception without corrective bus write |

Guards must cover the owner's actual I/O surface; source-string checks are not
runtime evidence. Use no real devices, network endpoints, sleeps or hardware
commands. Do not monkeypatch away the safety check under test. Add executable
cases with each protocol renderer in its assigned issue. An empty or skipped
suite does not establish conformance.
