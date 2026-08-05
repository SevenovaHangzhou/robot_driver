from rt_control_interfaces.msg import PlcIoState


def test_plc_io_state_exposes_only_confirmed_semantic_state() -> None:
    message = PlcIoState()

    message.connected = True
    message.data_fresh = True
    message.left_vacuum_established = True
    message.right_vacuum_established = False
    message.left_solenoid_on = True
    message.right_solenoid_on = False
    message.vacuum_pump_on = True
    message.io_alarm = 2
    message.error = ""

    assert message.connected
    assert message.data_fresh
    assert message.left_vacuum_established
    assert not message.right_vacuum_established
    assert message.left_solenoid_on
    assert not message.right_solenoid_on
    assert message.vacuum_pump_on
    assert message.io_alarm == 2
    assert message.error == ""
