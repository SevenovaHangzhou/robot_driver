from rt_control_interfaces.msg import PlcIoState


def test_plc_io_state_exposes_only_confirmed_semantic_state() -> None:
    message = PlcIoState()

    message.hardware_configured = False
    message.connected = True
    message.data_fresh = True
    message.left_valve_output_valid = True
    message.left_solenoid_on = True
    message.right_valve_output_valid = True
    message.right_solenoid_on = False
    message.pump_output_valid = True
    message.vacuum_pump_on = True
    message.left_pressure_valid = False
    message.right_pressure_valid = False
    message.io_alarm = 2
    message.error = ""

    assert not message.hardware_configured
    assert message.connected
    assert message.data_fresh
    assert message.left_valve_output_valid
    assert message.left_solenoid_on
    assert message.right_valve_output_valid
    assert not message.right_solenoid_on
    assert message.pump_output_valid
    assert message.vacuum_pump_on
    assert not message.left_pressure_valid
    assert not message.right_pressure_valid
    assert message.io_alarm == 2
    assert message.error == ""
