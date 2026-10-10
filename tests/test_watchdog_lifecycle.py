"""Regression tests for watchdog lifecycle, motion completion, control sessions, and safety interlocks."""

import time
import pytest
from fastapi.testclient import TestClient
from backend.main import app, init_system
from simulation.simulated_arm import SimulatedArm
from backend.safety import SafetyMonitor
from backend.config import ArmConfig
from backend.exceptions import SafetyFaultError, EmergencyStopError

client = TestClient(app)

class ControlledClock:
    """Deterministic mock clock for controlling time in safety tests."""
    def __init__(self, start_time: float = 1000.0):
        self.current_time = float(start_time)

    def __call__(self) -> float:
        return self.current_time

    def advance(self, seconds: float) -> None:
        self.current_time += float(seconds)


def test_startup_idle_period_longer_than_watchdog_timeout():
    """1. Startup followed by idle time must NOT trip watchdog."""
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Startup state check
    assert safety.check_safety()["safe"] is True
    assert safety.active_control is False

    # Sleep longer than watchdog timeout while idle
    time.sleep(0.3)

    # Must remain safe and untripped
    status = safety.check_safety()
    assert status["safe"] is True
    assert safety.fault_latched is False
    assert safety.watchdog_tripped is False

    arm.disconnect()


def test_movement_lasting_longer_than_watchdog_timeout():
    """2. Movement lasting longer than watchdog timeout must complete cleanly without watchdog fault."""
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Start a movement
    safety.record_command()
    arm.move_joints({1: 250.0}, speed=10) # Slow speed move

    # Simulate steps over 0.4 seconds (>0.2s timeout)
    for _ in range(4):
        time.sleep(0.1)
        status = safety.check_safety()
        assert status["safe"] is True
        assert safety.fault_latched is False

    arm.disconnect()


def test_completed_movement_followed_by_idle_period():
    """3. Completed movement followed by an idle period must NOT trip watchdog even without manual stop."""
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Initiate movement
    safety.record_command()
    arm.move_joints({1: 160.0}, speed=1023) # Fast move

    # Step simulation to finish motion
    for _ in range(5):
        arm.update_simulation_step(dt=0.1)

    # Motion has settled; safety monitor automatically transitions active_control to False
    status = safety.check_safety()
    assert status["safe"] is True
    assert safety.active_control is False

    # Sleep idle longer than watchdog timeout
    time.sleep(0.3)
    idle_status = safety.check_safety()
    assert idle_status["safe"] is True
    assert safety.fault_latched is False
    assert safety.watchdog_tripped is False

    arm.disconnect()


def test_multiple_sequential_movement_commands():
    """4. Multiple sequential movement commands execute safely."""
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    for target_angle in [170.0, 140.0, 180.0, 150.0]:
        safety.record_command()
        assert arm.move_joints({1: target_angle}, speed=500) is True
        time.sleep(0.05)
        assert safety.check_safety()["safe"] is True

    arm.disconnect()


def test_active_control_session_heartbeat_loss():
    """5. Active control session heartbeat loss triggers watchdog timeout."""
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Explicitly start active control session
    safety.start_control_session()
    assert safety.active_control is True

    # Send heartbeat
    time.sleep(0.1)
    safety.record_heartbeat()
    assert safety.check_safety()["safe"] is True

    # Stop sending heartbeats for >0.2s
    time.sleep(0.3)
    status = safety.check_safety()
    assert status["safe"] is False
    assert safety.fault_latched is True
    assert safety.watchdog_tripped is True
    assert "WATCHDOG TIMEOUT" in safety.fault_reason

    arm.disconnect()


def test_genuine_watchdog_timeout_and_fault_latching():
    """6 & 7. Genuine watchdog timeout latches fault and rejects subsequent commands."""
    config = ArmConfig(simulation=True, watchdog_timeout=0.2)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    safety.start_control_session()
    time.sleep(0.3)
    safety.check_safety()

    assert safety.fault_latched is True

    # Subsequent move commands must be rejected
    with pytest.raises((SafetyFaultError, EmergencyStopError)):
        arm.move_joints({1: 150.0})

    # Clear fault recovery
    assert safety.clear_fault() is True
    assert safety.fault_latched is False
    assert arm.move_joints({1: 150.0}) is True

    arm.disconnect()


def test_estop_and_torque_interlocks():
    """8. E-STOP and torque interlocks prevent movement and torque enable while faulted."""
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)

    # Trigger E-STOP
    safety.trigger_estop()
    assert safety.fault_latched is True

    # Torque enable rejected
    with pytest.raises(SafetyFaultError):
        arm.enable_torque(True)

    # Move rejected
    with pytest.raises(EmergencyStopError):
        arm.move_joints({1: 150.0})

    # Clear fault
    assert safety.clear_fault() is True
    assert arm.enable_torque(True) is True

    arm.disconnect()


def test_heartbeat_api_endpoints():
    """9. REST heartbeat and control session endpoints work cleanly."""
    init_system(ArmConfig(simulation=True))

    # Start control session
    res_start = client.post("/api/control_session/start")
    assert res_start.status_code == 200

    # Send heartbeat
    res_hb = client.post("/api/heartbeat")
    assert res_hb.status_code == 200

    # Stop control session
    res_stop = client.post("/api/control_session/stop")
    assert res_stop.status_code == 200


# DETERMINISTIC REGRESSION TESTS USING CONTROLLED TIME

def test_deterministic_idle_for_20_seconds():
    """Deterministic: Leaving the system idle for 20 seconds must not cause a false fault."""
    clock = ControlledClock(1000.0)
    config = ArmConfig(simulation=True, watchdog_timeout=5.0)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config, clock_fn=clock)

    assert safety.check_safety()["safe"] is True
    assert safety.active_control is False

    # Advance clock by 20.0 seconds while idle
    clock.advance(20.0)

    status = safety.check_safety()
    assert status["safe"] is True
    assert status["fault_latched"] is False
    assert status["watchdog_tripped"] is False
    assert safety.active_control is False
    arm.disconnect()


def test_deterministic_movement_over_five_seconds():
    """Deterministic: A finite movement lasting >5s must not fault solely due to no new command."""
    clock = ControlledClock(1000.0)
    config = ArmConfig(simulation=True, watchdog_timeout=5.0)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config, clock_fn=clock)

    # Command a long slow movement from 150 deg to 280 deg
    safety.record_command()
    arm.move_joints({1: 280.0}, speed=30) # ~20 deg/sec, takes >6 seconds

    # Advance 8 seconds in 0.5s steps (>5.0s watchdog timeout)
    for _ in range(16):
        clock.advance(0.5)
        arm.update_simulation_step(dt=0.5)
        status = safety.check_safety()
        assert status["safe"] is True
        assert status["fault_latched"] is False

    arm.disconnect()


def test_deterministic_completion_followed_by_idle_20_seconds():
    """Deterministic: Movement completes, settles, and remains idle for 20s without fault."""
    clock = ControlledClock(1000.0)
    config = ArmConfig(simulation=True, watchdog_timeout=5.0)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config, clock_fn=clock)

    safety.record_command()
    arm.move_joints({1: 180.0}, speed=1023)

    # Step simulation until position reached
    for _ in range(5):
        clock.advance(0.1)
        arm.update_simulation_step(dt=0.1)

    # Verify movement settled and active control ended
    status = safety.check_safety()
    assert status["safe"] is True
    assert safety.active_control is False

    # Leave idle for 20 seconds
    clock.advance(20.0)
    idle_status = safety.check_safety()
    assert idle_status["safe"] is True
    assert idle_status["fault_latched"] is False
    assert idle_status["watchdog_tripped"] is False
    arm.disconnect()


def test_deterministic_sequential_moves_with_idle():
    """Deterministic: Multiple sequential movements with idle periods execute normally."""
    clock = ControlledClock(1000.0)
    config = ArmConfig(simulation=True, watchdog_timeout=5.0)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config, clock_fn=clock)

    targets = [170.0, 140.0, 190.0, 150.0]
    for target in targets:
        safety.record_command()
        arm.move_joints({1: target}, speed=1023)
        for _ in range(5):
            clock.advance(0.1)
            arm.update_simulation_step(dt=0.1)
        status = safety.check_safety()
        assert status["safe"] is True
        assert safety.active_control is False

        # Idle between commands for 10 seconds
        clock.advance(10.0)
        assert safety.check_safety()["safe"] is True

    arm.disconnect()


def test_deterministic_heartbeat_loss_and_fault_latch():
    """Deterministic: Genuine loss of active-control heartbeat latches fault and rejects moves."""
    clock = ControlledClock(1000.0)
    config = ArmConfig(simulation=True, watchdog_timeout=5.0)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config, clock_fn=clock)

    # Explicit control session started
    safety.start_control_session()
    assert safety.active_control is True

    # Send heartbeat after 2 seconds
    clock.advance(2.0)
    safety.record_heartbeat()
    assert safety.check_safety()["safe"] is True

    # Stop sending heartbeats for 5.1s (> 5.0s timeout)
    clock.advance(5.1)
    status = safety.check_safety()
    assert status["safe"] is False
    assert status["watchdog_tripped"] is True
    assert status["fault_latched"] is True
    assert "WATCHDOG TIMEOUT" in status["fault_reason"]

    # Subsequent commands rejected while faulted
    with pytest.raises((SafetyFaultError, EmergencyStopError)):
        arm.move_joints({1: 160.0})

    # Clear fault restores normal operation
    assert safety.clear_fault() is True
    assert safety.fault_latched is False
    assert arm.move_joints({1: 160.0}) is True

    arm.disconnect()


def test_browser_workflow_via_api():
    """Simulate complete browser interaction lifecycle through FastAPI REST API."""
    init_system(ArmConfig(simulation=True, watchdog_timeout=5.0))

    # 1. Dashboard startup: initial state is READY and safe
    state1 = client.get("/api/state").json()
    assert state1["system_status"] in ("READY", "IDLE")
    assert state1["fault_latched"] is False

    # 2. User moves joints from dashboard
    res_move = client.post("/api/move", json={"angles": {"1": 180.0, "2": 150.0}, "speed": 1023})
    assert res_move.status_code == 200
    assert res_move.json()["success"] is True

    # 3. Simulate time advancing for motion completion and idle holding
    from backend.main import arm_instance, safety_monitor
    for _ in range(10):
        arm_instance.update_simulation_step(dt=0.1)

    safety_status = safety_monitor.check_safety()
    assert safety_status["safe"] is True
    assert safety_monitor.active_control is False

    # 4. Dashboard sits idle for long period; state remains READY without fault
    state2 = client.get("/api/state").json()
    assert state2["fault_latched"] is False
    assert state2["watchdog_tripped"] is False

    # 5. Subsequent move command is accepted and executes cleanly
    res_move2 = client.post("/api/move", json={"angles": {"1": 150.0}, "speed": 1023})
    assert res_move2.status_code == 200
    assert res_move2.json()["success"] is True
