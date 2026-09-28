"""Tests for Safety Monitoring System (Watchdog, Temp Protection, E-STOP, Fault Latch)."""

import pytest
import time
from simulation.simulated_arm import SimulatedArm
from backend.safety import SafetyMonitor
from backend.config import ArmConfig
from backend.exceptions import EmergencyStopError, SafetyFaultError

def test_estop_and_fault_latch():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Trigger E-STOP
    safety.trigger_estop()
    assert safety.fault_latched is True
    assert arm.estop_active is True

    # Movement should be rejected during E-STOP
    with pytest.raises(EmergencyStopError):
        arm.move_joints({1: 150.0})

    # Clear fault
    assert safety.clear_fault() is True
    assert safety.fault_latched is False
    assert arm.estop_active is False

    # Movement should work again
    assert arm.move_joints({1: 150.0}) is True
    arm.disconnect()

def test_idle_startup_and_delay_before_first_movement():
    # 1. Application starts and remains usable while idle.
    # 2. User can wait > timeout before issuing first movement.
    config = ArmConfig(simulation=True, watchdog_timeout=0.2) # 0.2s timeout
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Idle check initially
    status = safety.check_safety()
    assert status["safe"] is True
    assert safety.fault_latched is False
    assert safety.active_control is False

    # Wait 0.3s (longer than watchdog timeout) while idle
    time.sleep(0.3)
    status = safety.check_safety()
    assert status["safe"] is True
    assert safety.fault_latched is False
    assert safety.watchdog_tripped is False

    # 3. First valid movement is accepted
    safety.record_command()
    assert arm.move_joints({1: 150.0}) is True
    assert safety.active_control is True

    arm.disconnect()

def test_watchdog_timeout_during_active_control():
    # 4. Watchdog still triggers when appropriate during active operation.
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Initiate active control
    safety.record_command()
    assert safety.check_safety()["safe"] is True

    # Wait >0.2s during active control without a command
    time.sleep(0.3)
    status = safety.check_safety()
    assert status["safe"] is False
    assert safety.fault_latched is True
    assert safety.watchdog_tripped is True
    assert "WATCHDOG TIMEOUT" in safety.fault_reason

    # 5. Watchdog torque-off behavior remains functional
    assert any(s.torque_enabled for s in arm.get_state().values()) is False

    arm.disconnect()

def test_watchdog_cannot_be_bypassed_by_enabling_torque():
    # 8. A watchdog fault cannot be bypassed by simply enabling torque.
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    safety.record_command()
    time.sleep(0.3)
    safety.check_safety()
    assert safety.fault_latched is True

    # Attempting to enable torque directly or record command while fault latched must fail
    with pytest.raises((SafetyFaultError, EmergencyStopError)):
        arm.enable_torque(True)

    assert safety.fault_latched is True

    # Must clear fault first
    assert safety.clear_fault() is True
    assert safety.fault_latched is False

    # Now torque can be enabled
    assert arm.enable_torque(True) is True
    arm.disconnect()

def test_temperature_protection():
    config = ArmConfig(simulation=True, maximum_temperature=60.0)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Artificially raise temperature on servo 3 to 65°C
    arm.servos[3]["temperature"] = 65.0

    status = safety.check_safety()
    assert status["safe"] is False
    assert safety.fault_latched is True
    assert "OVER-TEMPERATURE" in safety.fault_reason

    # Attempting clear_fault while temp is still high should fail
    with pytest.raises(SafetyFaultError):
        safety.clear_fault()

    # Cool down servo 3
    arm.servos[3]["temperature"] = 35.0
    assert safety.clear_fault() is True
    assert safety.fault_latched is False

    arm.disconnect()
