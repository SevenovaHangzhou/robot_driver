# Synthetic fixture package-share layouts

Each sample contains `machines/<variant>/`, `devices/`, and
`units/<variant>-01/`. All facts are synthetic and unverified. The manifest,
controller and safety-policy documents and module/calibration envelopes are
schema positives, not production admission positives. Model and protocol package
names are synthetic dependencies for future injected composer tests.

- A: EtherCAT 3 CSP + 1 PP at ring positions 2/5/7/9, branch responder at 0.
- B: EtherCAT 2 steering CSP + 2 drive CSV, plus 2 CANopen encoders.
- C: 3 CAN position/velocity motors.

`../fixture_index.yaml` maps each document to its shared schema. Owner-specific
`spec` validation and full composition are tested when those implementations
exist. No synthetic value is an authority for a real device.
