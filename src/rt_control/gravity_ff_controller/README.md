# Gravity feedforward controller

ELECTRI-136 provides one controller instance per seven-axis arm. `shadow` is the
default: it claims no command interfaces and publishes calculated `gravity_nm`,
raw `torque_actual_permille`, and validation flags. `active` claims only each
joint's `effort` command; the hardware mapping owns the N.m-to-permille conversion.
Shadow does not need effort calibration, `max_effort_nm`, or `max_slew_nm_per_s`;
it reports invalid feedback without issuing a command. Active mode requires
explicit positive limits for every joint. For active without a mapped 0x6077,
set `torque_actual_interface` to an empty string; shadow requires raw 0x6077.

Active configuration fails unless the SHA-256 of the original URDF string matches
a verified model record and every effort calibration is verified, finite, positive,
and sourced (a `TBD` source is rejected). The current V3 model and calibration are unverified, so the installed
draft cannot enter active mode.

Before the first enable, normal non-enabled states continuously preload scaled
gravity. During normal disable, output keeps updating while any axis still reports
Operation Enabled, holds after the group leaves Operation Enabled, and resumes
preload after all axes reach Switch On Disabled or Not Ready.
Fault Reaction Active holds the last valid output. Fault writes zero and latches.
Invalid feedback/model data ramps to zero at `fault_slew_nm_per_s` if the last valid
state was Operation Enabled; otherwise it writes zero. A latch is cleared only by
controller deactivate/reactivate. `fault_slew_nm_per_s=1000` is a fast software
default for offline development, not a commissioned support-removal rate.

`on_deactivate` writes zero immediately. Deactivating active feedforward under load
can therefore create a torque step: reduce `scale` to zero first, confirm output zero,
then deactivate. Brake timing and proof that 0x60B2 is accepted before enable remain
ELECTRI-135 commissioning work.
