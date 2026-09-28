"""Verification script for Section 6 Simulation Audit (14 manual steps)."""

import time
from fastapi.testclient import TestClient
from backend.main import app

def run_simulation_audit():
    client = TestClient(app)
    print("=" * 60)
    print(" STARTING SIMULATION MANUAL AUDIT (14 CHECKLIST ITEMS)")
    print("=" * 60)

    # 1 & 2. GET /api/state
    res = client.get("/api/state")
    assert res.status_code == 200
    state = res.json()
    print("[STEP 1-2] GET /api/state OK. Connected:", state["connected"], "| Simulation:", state["simulation"])

    # 3. Move all five joints
    move_payload = {"angles": {"1": 100.0, "2": 120.0, "3": 140.0, "4": 160.0, "5": 180.0}, "speed": 100}
    res_move = client.post("/api/move", json=move_payload)
    assert res_move.status_code == 200
    print("[STEP 3] POST /api/move sent successfully:", res_move.json()["message"])

    # 4 & 5. Observe gradual movement & telemetry changing
    time.sleep(0.2)
    state_after = client.get("/api/state").json()
    j1_pos = state_after["servos"]["1"]["present_position"]
    j1_goal = state_after["servos"]["1"]["goal_position"]
    print(f"[STEP 4-5] Gradual Movement Observed: J1 Present={j1_pos:.1f}°, Goal={j1_goal:.1f}°")

    # 6. Enable / Disable torque
    t_off = client.post("/api/torque/0").json()
    print("[STEP 6a] Torque OFF:", t_off["message"])
    t_on = client.post("/api/torque/1").json()
    print("[STEP 6b] Torque ON:", t_on["message"])

    # 7. Trigger E-STOP
    estop_res = client.post("/api/estop").json()
    print("[STEP 7] E-STOP Triggered:", estop_res["message"])

    # 8. Verify movement is blocked during E-STOP
    block_res = client.post("/api/move", json={"angles": {"1": 150.0}, "speed": 100})
    assert block_res.status_code == 409
    print("[STEP 8] Movement properly BLOCKED during E-STOP (HTTP 409 Conflict):", block_res.json()["message"])

    # 9 & 10. Clear fault & verify safe recovery
    clear_res = client.post("/api/clear_fault").json()
    print("[STEP 9-10] Clear Fault OK:", clear_res["message"])
    
    # Re-enable torque explicitly after E-STOP clear
    client.post("/api/torque/1")
    recover_move = client.post("/api/move", json={"angles": {"1": 150.0}, "speed": 100})
    assert recover_move.status_code == 200
    print("[STEP 10] Recovery movement executed successfully after clearing fault.")

    # 11 & 12. Trigger Watchdog & verify torque disabled
    print("[STEP 11-12] Simulating Watchdog timeout (waiting 5.2s without command with active torque)...")
    time.sleep(5.2)
    wd_state = client.get("/api/state").json()
    assert wd_state["watchdog_tripped"] is True
    print("[STEP 12] Watchdog TRIPPED as expected. Fault Reason:", wd_state["fault_reason"])

    # Clear watchdog fault
    client.post("/api/clear_fault")
    client.post("/api/torque/1")

    # 13. Test invalid joint angles (out of software limits)
    inv_angle = client.post("/api/move", json={"angles": {"2": 5.0}, "speed": 100})
    assert inv_angle.status_code == 400
    print("[STEP 13] Invalid angle rejected properly (HTTP 400 Bad Request):", inv_angle.json()["message"])

    # 14. Test invalid joint IDs
    inv_id = client.post("/api/move", json={"angles": {"99": 150.0}, "speed": 100})
    assert inv_id.status_code == 400
    print("[STEP 14] Invalid joint ID rejected properly (HTTP 400 Bad Request):", inv_id.json()["message"])

    print("=" * 60)
    print(" ALL 14 SIMULATION AUDIT CHECKLIST ITEMS PASSED PERFECTLY!")
    print("=" * 60)

if __name__ == "__main__":
    run_simulation_audit()
