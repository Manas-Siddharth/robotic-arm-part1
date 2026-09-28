"""Tests for SimulatedArm implementation."""

import pytest
import time
from simulation.simulated_arm import SimulatedArm
from backend.config import ArmConfig
from backend.exceptions import (
    JointLimitExceededError,
    InvalidCommandError,
    EmergencyStopError,
    SafetyFaultError
)

def test_simulation_startup():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    assert arm.connect() is True
    assert arm.is_connected() is True
    assert len(arm.servos) == 5
    arm.disconnect()
    assert arm.is_connected() is False

def test_position_conversion():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    
    # 0 deg -> 0 raw, 300 deg -> 1023 raw, 150 deg -> ~512 raw
    assert arm.degrees_to_raw(0.0) == 0
    assert arm.degrees_to_raw(300.0) == 1023
    assert 511 <= arm.degrees_to_raw(150.0) <= 513
    
    assert arm.raw_to_degrees(0) == 0.0
    assert arm.raw_to_degrees(1023) == 300.0
    assert abs(arm.raw_to_degrees(512) - 150.15) < 0.5

def test_joint_movement():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    arm.connect()

    target_angles = {1: 100.0, 2: 120.0, 3: 90.0, 4: 150.0, 5: 200.0}
    assert arm.move_joints(target_angles, speed=500) is True

    # Advance simulation step
    arm.update_simulation_step(dt=0.5)
    states = arm.get_state()

    assert states[1].goal_position == 100.0
    assert states[2].goal_position == 120.0
    arm.disconnect()

def test_joint_limit_enforcement():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    arm.connect()

    # J2 limit is [15, 285]
    with pytest.raises(JointLimitExceededError):
        arm.move_joints({2: 5.0})  # Below min limit 15°

    with pytest.raises(JointLimitExceededError):
        arm.move_joints({2: 295.0}) # Above max limit 285°

    arm.disconnect()

def test_invalid_joint_id():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    arm.connect()

    with pytest.raises(InvalidCommandError):
        arm.move_joints({99: 150.0}) # Joint 99 doesn't exist

    arm.disconnect()

def test_torque_control():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    arm.connect()

    arm.enable_torque(False)
    states = arm.get_state()
    for s in states.values():
        assert s.torque_enabled is False

    arm.enable_torque(True, joint_id=1)
    states = arm.get_state()
    assert states[1].torque_enabled is True
    assert states[2].torque_enabled is False
    arm.disconnect()
