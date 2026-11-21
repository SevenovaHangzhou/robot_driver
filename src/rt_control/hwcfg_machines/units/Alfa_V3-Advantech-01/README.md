# Alfa_V3-Advantech-01

Advantech MIC-770V3W chassis bench (ELECTRI-127): four Kinco steering/drive pairs on IgH
master 0, four BRT steering encoders and the LPMS-NAV3 chassis IMU (node 1, receive-only) on
`can3`. The DaMiao head gimbal is on `can2` (1 Mbit/s, ELECTRI-234); its zero is the motor
encoder zero set on this unit. The left/right batteries are on `can0`/`can1`.
Arm calibration of `Alfa_V3-01` does not belong to this unit.

Host CAN bring-up at boot: `hostsetup/advantech-zlg-can.sh` and `.service`.
