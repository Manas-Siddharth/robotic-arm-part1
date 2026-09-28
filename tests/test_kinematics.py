"""Tests for Kinematics (FK/IK) module and XYZ movement endpoints."""

import pytest
from backend.kinematics import KinematicsSolver, KinematicsController
from simulation.simulated_arm import SimulatedArm
from backend.safety import SafetyMonitor
from backend.config import ArmConfig
from backend.exceptions import InvalidCommandError
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_forward_and_inverse_kinematics_solver():
    solver = KinematicsSolver()

    # Home position (150° for all joints)
    home_angles = {1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}
    fk_res = solver.forward_kinematics(home_angles)
    
    assert "x" in fk_res
    assert "y" in fk_res
    assert "z" in fk_res

    # Test IK for achievable target
    ik_angles = solver.inverse_kinematics(x=200.0, y=0.0, z=200.0, pitch_deg=0.0)
    assert len(ik_angles) == 5
    assert 0.0 <= ik_angles[1] <= 300.0

def test_unreachable_ik_position():
    solver = KinematicsSolver()
    with pytest.raises(InvalidCommandError):
        # 2000mm is way outside physical arm reach (~445mm max)
        solver.inverse_kinematics(x=2000.0, y=0.0, z=2000.0)

def test_kinematics_controller_integration():
    config = ArmConfig(simulation=True)
    arm = SimulatedArm(config)
    arm.connect()
    safety = SafetyMonitor(arm, config)
    controller = KinematicsController(arm, safety)

    # Execute Cartesian XYZ movement through Arm abstraction
    success = controller.move_xyz(x=200.0, y=0.0, z=150.0, pitch=0.0, speed=100)
    assert success is True
    arm.disconnect()

def test_xyz_api_endpoints():
    # Test GET /api/kinematics/fk
    fk_res = client.get("/api/kinematics/fk")
    assert fk_res.status_code == 200
    data = fk_res.json()["data"]
    assert "x" in data and "z" in data

    # Test POST /api/move_xyz
    move_payload = {"x": 180.0, "y": 0.0, "z": 150.0, "pitch": 0.0, "speed": 150}
    res = client.post("/api/move_xyz", json=move_payload)
    assert res.status_code == 200
    assert res.json()["success"] is True
