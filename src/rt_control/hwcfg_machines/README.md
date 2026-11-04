# RT-Control machine configuration

KKOZIA-5 / SPEC v1.1 M0b data-only package skeleton.

- `devices/`: sourced model properties; unknown facts must remain `TBD`.
- `machines/Alfa_V3/`: machine wiring, manifest and explicit policies.
- `units/Alfa_V3-01/`: installation and calibration for the placeholder unit.
- `evidence/`: read-only observations, never a composer or renderer input.

The directory name `Alfa_V3-01` is a placeholder approved by the SPEC; it is
not a confirmed serial number. No hardware values or production policies are
provided by this skeleton. Hardware-configuration owners populate these
assets after A0. Only share assets are installed; there is no code API.

Schemas are owned by `integ_composer` and the relevant protocol packages.
