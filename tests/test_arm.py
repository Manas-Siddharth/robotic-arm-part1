"""Tests for BaseArm interface, factory, and hardware compatibility checks."""

import pytest
from backend.arm import create_arm, BaseArm, DynamixelArm
from backend.config import ArmConfig
from simulation.simulated_arm import SimulatedArm
from backend.exceptions import HardwareConnectionError

def test_arm_factory_simulation():
    config = ArmConfig(simulation=True)
    arm = create_arm(config)
    assert isinstance(arm, BaseArm)
    assert isinstance(arm, SimulatedArm)
    assert arm.is_connected() is True
    arm.disconnect()

def test_arm_factory_hardware_windows_fallback():
    # On Windows without real serial port, DynamixelArm should raise HardwareConnectionError
    config = ArmConfig(simulation=False, serial_port="/dev/ttyUSB0")
    with pytest.raises(HardwareConnectionError):
        arm = DynamixelArm(config)
        arm.connect()

def test_arm_interface_methods():
    config = ArmConfig(simulation=True)
    arm = create_arm(config)
    
    # Test ping
    assert arm.ping_servo(1) is True
    assert arm.ping_servo(99) is False

    # Test state dict format
    states = arm.get_state()
    assert len(states) == 5
    for sid in [1, 2, 3, 4, 5]:
        assert sid in states
        s = states[sid]
        assert hasattr(s, "present_position")
        assert hasattr(s, "goal_position")
        assert hasattr(s, "voltage")
        assert hasattr(s, "temperature")

    arm.disconnect()
