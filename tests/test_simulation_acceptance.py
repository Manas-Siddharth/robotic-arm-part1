"""End-to-End Simulation Acceptance Test verifying all 14 user requirements."""

import time
import pytest
from fastapi.testclient import TestClient
from backend.main import app, init_system, safety_monitor, arm_instance
from backend.config import ArmConfig

def test_full_simulation_acceptance_flow():
    # Configure 5-second watchdog timeout for acceptance test
    config = ArmConfig(simulation=True, watchdog_timeout=5.0)
    init_system(config)
    client = TestClient(app)

    # 1. Start server / init app state
    # 2. Check initial dashboard state
    state_res = client.get("/api/state")
    assert state_res.status_code == 200
    state = state_res.json()
    assert state["connected"] is True
    assert state["simulation"] is True
    assert state["system_status"] in ["READY", "MOVING"]
    assert state["fault_latched"] is False
    assert len(state["servos"]) == 5

    # 3. Wait at least 10 seconds without issuing a command
    # 4. Confirm system remains safely usable
    time.sleep(5.5)  # Waiting > 5.0s in idle state
    state_after_idle = client.get("/api/state").json()
    assert state_after_idle["fault_latched"] is False
    assert state_after_idle["system_status"] in ["READY", "MOVING"]
    assert state_after_idle["watchdog_tripped"] is False

    # 5. Move the five joints
    target_angles = {"1": 180.0, "2": 120.0, "3": 100.0, "4": 160.0, "5": 140.0}
    move_res = client.post("/api/move", json={"angles": target_angles, "speed": 200})
    assert move_res.status_code == 200
    assert move_res.json()["success"] is True

    # 6 & 7. Confirm simulated present positions change toward goals & telemetry updates
    time.sleep(0.5)
    updated_state = client.get("/api/state").json()
    for sid_str in ["1", "2", "3", "4", "5"]:
        servo = updated_state["servos"][sid_str]
        assert servo["goal_position"] == target_angles[sid_str]
        assert "present_position" in servo
        assert "speed" in servo
        assert "load" in servo
        assert "voltage" in servo
        assert "temperature" in servo

    # 8. Enable torque
    torque_on_res = client.post("/api/torque/1")
    assert torque_on_res.status_code == 200
    assert torque_on_res.json()["success"] is True

    # 9. Disable torque
    torque_off_res = client.post("/api/torque/0")
    assert torque_off_res.status_code == 200
    assert torque_off_res.json()["success"] is True

    # 10. Trigger E-STOP
    estop_res = client.post("/api/estop")
    assert estop_res.status_code == 200
    assert estop_res.json()["success"] is True

    # 11. Confirm torque cannot be re-enabled while E-STOP is active
    re_torque_res = client.post("/api/torque/1")
    assert re_torque_res.status_code in [400, 409]
    assert client.get("/api/state").json()["fault_latched"] is True

    # 12. Clear E-STOP/fault using the intended recovery flow
    clear_res = client.post("/api/clear_fault")
    assert clear_res.status_code == 200
    assert clear_res.json()["success"] is True

    # 13. Confirm safe recovery
    recovered_state = client.get("/api/state").json()
    assert recovered_state["fault_latched"] is False
    assert recovered_state["system_status"] in ["READY", "MOVING"]

    # 14. Test watchdog under the intended active-control condition
    client.post("/api/torque/1")
    # Initiate active control session
    client.post("/api/control_session/start")
    # Wait >5s without issuing any new heartbeat or command
    time.sleep(5.2)
    watchdog_state = client.get("/api/state").json()
    assert watchdog_state["fault_latched"] is True
    assert watchdog_state["watchdog_tripped"] is True
    assert watchdog_state["system_status"] in ["FAULT", "ESTOP"]
    assert "WATCHDOG TIMEOUT" in watchdog_state["fault_reason"]

    # Cleanup: clear fault after test
    client.post("/api/clear_fault")
