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

def test_kinematics_validate_api():
    # Test valid reachable Cartesian coordinates within software joint limits
    req_valid = {"x": 180.0, "y": 0.0, "z": 200.0, "pitch": 0.0}
    res_valid = client.post("/api/kinematics/validate", json=req_valid)
    assert res_valid.status_code == 200
    json_valid = res_valid.json()
    assert json_valid["success"] is True
    assert json_valid["data"]["reachable"] is True
    assert "angles" in json_valid["data"]


    # Test unreachable Cartesian coordinates
    req_unreachable = {"x": 800.0, "y": 800.0, "z": 800.0, "pitch": 0.0}
    res_unreachable = client.post("/api/kinematics/validate", json=req_unreachable)
    assert res_unreachable.status_code == 200
    json_unreachable = res_unreachable.json()
    assert json_unreachable["success"] is False
    assert json_unreachable["data"]["reachable"] is False

def test_sequences_api():
    # 1. Save a pose to use in sequence
    client.post("/api/poses", json={
        "name": "SeqPose1",
        "angles": {"1": 150, "2": 150, "3": 150, "4": 150, "5": 150},
        "speed": 100
    })

    # 2. Save sequence
    seq_payload = {
        "name": "TestAutoSequence",
        "steps": [
            {"pose_name": "SeqPose1", "speed": 150, "delay_seconds": 0.1}
        ]
    }
    save_res = client.post("/api/sequences", json=seq_payload)
    assert save_res.status_code == 200
    assert save_res.json()["success"] is True

    # 3. List sequences
    list_res = client.get("/api/sequences")
    assert list_res.status_code == 200
    seqs = list_res.json()["data"]
    assert any(s["name"] == "TestAutoSequence" for s in seqs)

    # 4. Execute sequence
    exec_res = client.post("/api/sequences/TestAutoSequence/execute")
    assert exec_res.status_code == 200

    # 5. Delete sequence & pose
    del_seq = client.delete("/api/sequences/TestAutoSequence")
    assert del_seq.status_code == 200
    client.delete("/api/poses/SeqPose1")

