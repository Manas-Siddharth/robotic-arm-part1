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

def test_known_fk_configurations():
    solver = KinematicsSolver()

    # 1. Home position (150° for all joints) -> Horizontal stretch along +X
    home_res = solver.forward_kinematics({1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    assert home_res["x"] == 300.0  # L2(105) + L3(105) + L4(90) = 300
    assert home_res["y"] == 0.0
    assert home_res["z"] == 145.0  # L1 = 145
    assert home_res["pitch"] == 0.0

    # 2. Vertical Up position (J2 = 240°, J1=J3=J4=J5=150°)
    vert_res = solver.forward_kinematics({1: 150.0, 2: 240.0, 3: 150.0, 4: 150.0, 5: 150.0})
    assert vert_res["x"] == 0.0
    assert vert_res["y"] == 0.0
    assert vert_res["z"] == 445.0  # L1(145) + L2(105) + L3(105) + L4(90) = 445
    assert vert_res["pitch"] == 90.0

def test_fk_ik_round_trip():
    solver = KinematicsSolver()

    # Start with target XYZ
    target_x, target_y, target_z, target_pitch = 180.0, 50.0, 180.0, 0.0
    ik_angles = solver.inverse_kinematics(target_x, target_y, target_z, target_pitch)
    
    # Run FK on solved angles
    fk_result = solver.forward_kinematics(ik_angles)

    # Output should match original target within rounding tolerance
    assert abs(fk_result["x"] - target_x) < 1.0
    assert abs(fk_result["y"] - target_y) < 1.0
    assert abs(fk_result["z"] - target_z) < 1.0
    assert abs(fk_result["pitch"] - target_pitch) < 1.0

