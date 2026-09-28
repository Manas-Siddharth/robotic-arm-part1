"""Tests for FastAPI REST endpoints."""

import pytest
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_dashboard_endpoint():
    response = client.get("/")
    assert response.status_code == 200
    assert "DYNAMIXEL ARM" in response.text

def test_get_system_state():
    response = client.get("/api/state")
    assert response.status_code == 200
    data = response.json()
    assert data["connected"] is True
    assert data["simulation"] is True
    assert "servos" in data
    assert len(data["servos"]) == 5

def test_move_joints_api():
    payload = {
        "angles": {
            "1": 150,
            "2": 120,
            "3": 90,
            "4": 100,
            "5": 180
        },
        "speed": 100
    }
    response = client.post("/api/move", json=payload)
    assert response.status_code == 200
    res = response.json()
    assert res["success"] is True

def test_torque_api():
    # Torque OFF
    response = client.post("/api/torque/0")
    assert response.status_code == 200
    assert response.json()["success"] is True

    # Torque ON
    response = client.post("/api/torque/1")
    assert response.status_code == 200
    assert response.json()["success"] is True

def test_estop_and_clear_fault_api():
    # Emergency Stop
    response = client.post("/api/estop")
    assert response.status_code == 200
    assert response.json()["success"] is True

    # State should reflect fault/estop
    state_res = client.get("/api/state").json()
    assert state_res["fault_latched"] is True

    # Move during E-STOP should fail with 409 Conflict
    move_res = client.post("/api/move", json={"angles": {"1": 150}, "speed": 100})
    assert move_res.status_code == 409

    # Clear fault
    clear_res = client.post("/api/clear_fault")
    assert clear_res.status_code == 200
    assert clear_res.json()["success"] is True

def test_poses_api():
    # Save pose
    pose_payload = {
        "name": "TestHomePose",
        "angles": {"1": 150, "2": 150, "3": 150, "4": 150, "5": 150},
        "speed": 200
    }
    save_res = client.post("/api/poses", json=pose_payload)
    assert save_res.status_code == 200

    # List poses
    list_res = client.get("/api/poses")
    assert list_res.status_code == 200
    poses = list_res.json()["data"]
    assert any(p["name"] == "TestHomePose" for p in poses)

    # Execute pose
    exec_res = client.post("/api/poses/TestHomePose/execute")
    assert exec_res.status_code == 200

    # Delete pose
    del_res = client.delete("/api/poses/TestHomePose")
    assert del_res.status_code == 200
