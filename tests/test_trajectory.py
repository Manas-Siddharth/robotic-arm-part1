"""Automated tests for Phase 1: Trajectory Planning module, executor, and API endpoints.

Covers:
- Joint-space trajectories for 5 joints.
- Linear, cubic, quintic, and cubic-spline interpolation profiles.
- Position, velocity, and acceleration calculations.
- Synchronized multi-joint motion.
- Multi-waypoint continuity and boundary conditions.
- Joint-limit, duration, velocity, and acceleration validations.
- Deterministic trajectory execution (no real-time sleeps).
- Cancellation, E-STOP, and safety fault interlocks.
- Watchdog behavior during and after execution.
- REST API endpoints (/validate, /preview, /execute, /cancel, /status).
"""

import math
import asyncio
import pytest
from fastapi.testclient import TestClient

from backend.config import ArmConfig, get_config
from backend.exceptions import InvalidCommandError, SafetyFaultError, EmergencyStopError
from backend.main import app, init_system
from backend.safety import SafetyMonitor
from backend.trajectory import (
    InterpolationType,
    TrajectoryExecutor,
    TrajectoryPlan,
    TrajectoryPlanner,
    Waypoint,
)
from simulation.simulated_arm import SimulatedArm


class ControlledClock:
    """Deterministic virtual clock for safety and timing tests."""
    def __init__(self, start_time: float = 1000.0):
        self.current_time = float(start_time)

    def __call__(self) -> float:
        return self.current_time

    def advance(self, seconds: float) -> None:
        self.current_time += float(seconds)


@pytest.fixture
def arm_config():
    return ArmConfig(
        simulation=True,
        watchdog_timeout=5.0,
        trajectory_sample_rate=20,
        trajectory_velocity_limits={1: 180.0, 2: 150.0, 3: 150.0, 4: 180.0, 5: 200.0},
        trajectory_acceleration_limits={1: 360.0, 2: 300.0, 3: 300.0, 4: 360.0, 5: 400.0}
    )


@pytest.fixture
def planner(arm_config):
    return TrajectoryPlanner(arm_config)


@pytest.fixture
def simulated_arm(arm_config):
    arm = SimulatedArm(arm_config)
    arm.connect()
    arm.enable_torque(True)
    yield arm
    arm.disconnect()


@pytest.fixture
def safety_monitor(simulated_arm, arm_config):
    return SafetyMonitor(simulated_arm, arm_config)


@pytest.fixture
def executor(simulated_arm, safety_monitor, arm_config):
    return TrajectoryExecutor(simulated_arm, safety_monitor, arm_config)


@pytest.fixture
def test_client():
    init_system()
    import backend.main as main_mod
    if main_mod.safety_monitor and main_mod.safety_monitor.fault_latched:
        main_mod.safety_monitor.clear_fault()
    if main_mod.arm_instance:
        if getattr(main_mod.arm_instance, "estop_active", False):
            main_mod.arm_instance.clear_estop()
        main_mod.arm_instance.enable_torque(True)
    return TestClient(app)


# ============================================================================
# 1. WAYPOINT & PLANNER BASICS
# ============================================================================

def test_waypoint_dataclass_and_validation(planner):
    """Waypoint validation checks joint counts and angle ranges."""
    wp_valid = Waypoint(
        angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0},
        duration=2.0,
        name="Home"
    )
    assert wp_valid.duration == 2.0
    assert len(wp_valid.angles) == 5

    # Missing a joint
    wp_missing = Waypoint(
        angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0},
        duration=2.0
    )
    plan = planner.plan([wp_valid, wp_missing])
    assert plan.is_valid is False
    assert any("missing joint" in err.lower() for err in plan.validation_errors)


def test_trajectory_minimum_waypoints(planner):
    """At least 2 waypoints are required to form a trajectory."""
    wp = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=1.0)
    plan = planner.plan([wp])
    assert plan.is_valid is False
    assert any("at least 2 waypoints" in err.lower() for err in plan.validation_errors)


# ============================================================================
# 2. INTERPOLATION PROFILES & ANALYTICAL MATH
# ============================================================================

def test_linear_interpolation(planner):
    """Linear interpolation provides constant velocity and exact endpoints."""
    wp0 = Waypoint(angles={1: 100.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp1 = Waypoint(angles={1: 160.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=2.0)

    plan = planner.plan([wp0, wp1], interpolation=InterpolationType.LINEAR, sample_rate=20)
    assert plan.is_valid is True
    assert plan.total_duration == pytest.approx(2.0)

    # Start position and end position
    assert plan.positions[1][0] == pytest.approx(100.0, abs=1e-3)
    assert plan.positions[1][-1] == pytest.approx(160.0, abs=1e-3)

    # Midpoint position at t = 1.0s (u = 0.5)
    mid_idx = len(plan.time_points) // 2
    assert plan.positions[1][mid_idx] == pytest.approx(130.0, abs=0.5)

    # Linear velocity is (160 - 100) / 2.0 = 30 deg/s throughout
    for v in plan.velocities[1][1:-1]:
        assert v == pytest.approx(30.0, abs=1e-2)

    # Acceleration is zero
    for a in plan.accelerations[1][1:-1]:
        assert a == pytest.approx(0.0, abs=1e-2)


def test_cubic_interpolation(planner):
    """Cubic interpolation s(u) = 3u^2 - 2u^3 gives zero endpoint velocities."""
    wp0 = Waypoint(angles={1: 100.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp1 = Waypoint(angles={1: 160.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=2.0)

    plan = planner.plan([wp0, wp1], interpolation=InterpolationType.CUBIC, sample_rate=20)
    assert plan.is_valid is True

    # Endpoint velocities must be 0
    assert plan.velocities[1][0] == pytest.approx(0.0, abs=1e-4)
    assert plan.velocities[1][-1] == pytest.approx(0.0, abs=1e-4)

    # Midpoint velocity: s'(0.5) = 6(0.5) - 6(0.25) = 1.5 -> v_mid = 1.5 * (60/2) = 45 deg/s
    mid_idx = len(plan.time_points) // 2
    assert plan.velocities[1][mid_idx] == pytest.approx(45.0, abs=1.0)
    assert plan.peak_velocities[1] == pytest.approx(45.0, abs=1e-2)

    # Acceleration at midpoint u=0.5 is s''(0.5) = 6 - 12(0.5) = 0
    assert plan.accelerations[1][mid_idx] == pytest.approx(0.0, abs=1.0)


def test_quintic_interpolation(planner):
    """Quintic interpolation s(u) = 10u^3 - 15u^4 + 6u^5 gives zero endpoint velocity & acceleration."""
    delta_q = 60.0
    T = 2.0
    wp0 = Waypoint(angles={1: 100.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp1 = Waypoint(angles={1: 100.0 + delta_q, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=T)

    plan = planner.plan([wp0, wp1], interpolation=InterpolationType.QUINTIC, sample_rate=50)
    assert plan.is_valid is True

    # Start and end positions
    assert plan.positions[1][0] == pytest.approx(100.0, abs=1e-4)
    assert plan.positions[1][-1] == pytest.approx(160.0, abs=1e-4)

    # Start and end velocities must be exactly 0
    assert plan.velocities[1][0] == pytest.approx(0.0, abs=1e-4)
    assert plan.velocities[1][-1] == pytest.approx(0.0, abs=1e-4)

    # Start and end accelerations must be exactly 0
    assert plan.accelerations[1][0] == pytest.approx(0.0, abs=1e-4)
    assert plan.accelerations[1][-1] == pytest.approx(0.0, abs=1e-4)

    # Analytical peak velocity for quintic:
    # s'(u) = 30u^2(1 - u)^2. Max at u = 0.5 is 30 * 0.25 * 0.25 = 1.875
    # peak_v = 1.875 * (delta_q / T) = 1.875 * (60.0 / 2.0) = 56.25 deg/s
    expected_peak_v = 1.875 * (delta_q / T)
    assert plan.peak_velocities[1] == pytest.approx(expected_peak_v, abs=1e-2)

    # Analytical peak acceleration for quintic:
    # s''(u) = 60u(1 - u)(1 - 2u). Max occurs at u = (3 - sqrt(3))/6 approx 0.2113
    # |s''(u)|_max = 10 / sqrt(3) approx 5.7735027
    # peak_a = 5.7735 * (delta_q / T^2) = 5.7735027 * (60 / 4) approx 86.6025 deg/s^2
    expected_peak_a = (10.0 / math.sqrt(3.0)) * (delta_q / (T * T))
    assert plan.peak_accelerations[1] == pytest.approx(expected_peak_a, abs=1e-2)


# ============================================================================
# 3. SYNCHRONIZATION ACROSS ALL FIVE JOINTS
# ============================================================================

def test_synchronization_of_all_five_joints(planner):
    """All 5 joints arrive at waypoints at the exact same time with synchronized progress."""
    wp0 = Waypoint(angles={1: 150.0, 2: 120.0, 3: 90.0, 4: 140.0, 5: 150.0})
    wp1 = Waypoint(angles={1: 180.0, 2: 160.0, 3: 130.0, 4: 100.0, 5: 200.0}, duration=3.0)

    plan = planner.plan([wp0, wp1], interpolation=InterpolationType.QUINTIC, sample_rate=20)
    assert plan.is_valid is True

    # All joints must start at t=0 and end at t=3.0
    for sid in range(1, 6):
        assert plan.positions[sid][0] == pytest.approx(wp0.angles[sid], abs=1e-3)
        assert plan.positions[sid][-1] == pytest.approx(wp1.angles[sid], abs=1e-3)
        assert plan.velocities[sid][0] == pytest.approx(0.0, abs=1e-3)
        assert plan.velocities[sid][-1] == pytest.approx(0.0, abs=1e-3)

    # At midpoint t=1.5s (u=0.5), normalized progress s(0.5) is 0.5
    # Each joint should be exactly at wp0 + 0.5 * (wp1 - wp0)
    mid_idx = len(plan.time_points) // 2
    for sid in range(1, 6):
        expected_mid = wp0.angles[sid] + 0.5 * (wp1.angles[sid] - wp0.angles[sid])
        assert plan.positions[sid][mid_idx] == pytest.approx(expected_mid, abs=0.5)


# ============================================================================
# 4. MULTI-WAYPOINT CONTINUITY & SEGMENT BOUNDARY BEHAVIOR
# ============================================================================

def test_multi_waypoint_continuity_and_boundary(planner):
    """Verify position continuity across multi-waypoint transitions."""
    wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp1 = Waypoint(angles={1: 180.0, 2: 130.0, 3: 120.0, 4: 160.0, 5: 180.0}, duration=1.5)
    wp2 = Waypoint(angles={1: 120.0, 2: 170.0, 3: 140.0, 4: 110.0, 5: 150.0}, duration=2.0)
    wp3 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=1.0)

    plan = planner.plan([wp0, wp1, wp2, wp3], interpolation=InterpolationType.QUINTIC, sample_rate=20)
    assert plan.is_valid is True
    assert plan.total_duration == pytest.approx(4.5)

    # Check segment transition timestamps: t = 1.5s, t = 3.5s
    # Sample at t=1.5 should match wp1 within sampling tolerance
    idx_wp1 = min(range(len(plan.time_points)), key=lambda i: abs(plan.time_points[i] - 1.5))
    for sid in range(1, 6):
        assert plan.positions[sid][idx_wp1] == pytest.approx(wp1.angles[sid], abs=0.5)

    idx_wp2 = min(range(len(plan.time_points)), key=lambda i: abs(plan.time_points[i] - 3.5))
    for sid in range(1, 6):
        assert plan.positions[sid][idx_wp2] == pytest.approx(wp2.angles[sid], abs=0.5)

    # Final position matches wp3
    for sid in range(1, 6):
        assert plan.positions[sid][-1] == pytest.approx(wp3.angles[sid], abs=1e-3)


# ============================================================================
# 5. VALIDATIONS: JOINT LIMITS, DURATIONS, VELOCITY & ACCELERATION LIMITS
# ============================================================================

def test_validation_joint_limit_violation(planner):
    """Waypoints exceeding configured joint angle limits are rejected."""
    # Joint 1 software range is 0 to 300
    wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp_invalid = Waypoint(angles={1: 350.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=1.0)

    plan = planner.plan([wp0, wp_invalid])
    assert plan.is_valid is False
    assert any("exceeds" in err.lower() and "joint 1" in err.lower() for err in plan.validation_errors)


def test_validation_invalid_durations(planner):
    """Zero or negative durations are rejected with clear error messages."""
    wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp_zero = Waypoint(angles={1: 160.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=0.0)

    plan = planner.plan([wp0, wp_zero])
    assert plan.is_valid is False
    assert any("invalid duration" in err.lower() for err in plan.validation_errors)

    wp_neg = Waypoint(angles={1: 160.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=-1.5)
    plan_neg = planner.plan([wp0, wp_neg])
    assert plan_neg.is_valid is False
    assert any("invalid duration" in err.lower() for err in plan_neg.validation_errors)


def test_validation_velocity_limit_violation(planner):
    """Large displacements in very small duration violate velocity limits."""
    # Joint 1 limit is 180 deg/s. Moving 100 deg in 0.2s quintic requires > 500 deg/s
    wp0 = Waypoint(angles={1: 100.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp1 = Waypoint(angles={1: 200.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=0.2)

    plan = planner.plan([wp0, wp1], interpolation=InterpolationType.QUINTIC)
    assert plan.is_valid is False
    assert any("velocity limit exceeded" in err.lower() and "joint 1" in err.lower() for err in plan.validation_errors)


def test_validation_acceleration_limit_violation(planner):
    """Extremely rapid acceleration triggers acceleration limit violation."""
    # Joint 2 acceleration limit is 300 deg/s^2.
    # Moving 80 deg in 0.3s quintic: peak_a = 5.7735 * (80 / 0.09) = 5132 deg/s^2 > 300
    wp0 = Waypoint(angles={1: 150.0, 2: 100.0, 3: 150.0, 4: 150.0, 5: 150.0})
    wp1 = Waypoint(angles={1: 150.0, 2: 180.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=0.3)

    plan = planner.plan([wp0, wp1], interpolation=InterpolationType.QUINTIC)
    assert plan.is_valid is False
    assert any("acceleration limit exceeded" in err.lower() for err in plan.validation_errors)


# ============================================================================
# 6. DETERMINISTIC EXECUTION (NO REAL-TIME SLEEPS)
# ============================================================================

def test_deterministic_executor_execution(simulated_arm, safety_monitor, arm_config):
    """Executor smoothly runs trajectory to completion using deterministic fast sleep."""
    async def _test():
        executor = TrajectoryExecutor(simulated_arm, safety_monitor, arm_config)
        planner = TrajectoryPlanner(arm_config)

        wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
        wp1 = Waypoint(angles={1: 180.0, 2: 120.0, 3: 100.0, 4: 160.0, 5: 180.0}, duration=2.0)

        plan = planner.plan([wp0, wp1], interpolation=InterpolationType.QUINTIC, sample_rate=20)
        assert plan.is_valid is True

        # Deterministic dummy sleep function (0 wall-clock delay)
        sleep_calls = 0
        async def instant_sleep(dt: float):
            nonlocal sleep_calls
            sleep_calls += 1

        status = await executor.execute(plan, sleep_fn=instant_sleep)
        assert status["state"] == "COMPLETED"
        assert status["progress_percent"] == 100.0
        assert sleep_calls == len(plan.time_points) - 1

        # Arm joints should now match final waypoint
        states = simulated_arm.get_state()
        for sid in range(1, 6):
            assert states[sid].present_position == pytest.approx(wp1.angles[sid], abs=0.1)

    asyncio.run(_test())


def test_executor_validation_failure_prevents_motion(simulated_arm, safety_monitor, arm_config):
    """Attempting to execute an invalid trajectory raises error and performs zero movement."""
    async def _test():
        executor = TrajectoryExecutor(simulated_arm, safety_monitor, arm_config)
        planner = TrajectoryPlanner(arm_config)

        # Invalid waypoint outside limits
        wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
        wp_bad = Waypoint(angles={1: 340.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=1.0)
        bad_plan = planner.plan([wp0, wp_bad])
        assert bad_plan.is_valid is False

        with pytest.raises(InvalidCommandError):
            await executor.execute(bad_plan)

        assert executor.state == "FAULT"
        # Arm remains at original position 150 deg
        assert simulated_arm.get_state()[1].present_position == pytest.approx(150.0, abs=0.1)

    asyncio.run(_test())


def test_executor_cancellation(simulated_arm, safety_monitor, arm_config):
    """Cancellation terminates execution mid-trajectory and records CANCELLED state."""
    async def _test():
        executor = TrajectoryExecutor(simulated_arm, safety_monitor, arm_config)
        planner = TrajectoryPlanner(arm_config)

        wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
        wp1 = Waypoint(angles={1: 180.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=2.0)
        plan = planner.plan([wp0, wp1], sample_rate=20)

        step_count = 0
        async def cancel_at_step_5(dt: float):
            nonlocal step_count
            step_count += 1
            if step_count == 5:
                executor.cancel()

        status = await executor.execute(plan, sleep_fn=cancel_at_step_5)
        assert status["state"] == "CANCELLED"
        assert status["progress_percent"] < 100.0

    asyncio.run(_test())


def test_executor_aborts_on_safety_fault_and_estop(simulated_arm, safety_monitor, arm_config):
    """Safety fault or emergency stop immediately aborts trajectory execution."""
    async def _test():
        executor = TrajectoryExecutor(simulated_arm, safety_monitor, arm_config)
        planner = TrajectoryPlanner(arm_config)

        wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
        wp1 = Waypoint(angles={1: 180.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=2.0)
        plan = planner.plan([wp0, wp1], sample_rate=20)

        step_count = 0
        async def estop_at_step_3(dt: float):
            nonlocal step_count
            step_count += 1
            if step_count == 3:
                safety_monitor.trigger_estop()

        status = await executor.execute(plan, sleep_fn=estop_at_step_3)
        assert status["state"] == "FAULT"
        assert "safety fault" in status["message"].lower() or "estop" in status["message"].lower()

    asyncio.run(_test())


# ============================================================================
# 7. WATCHDOG BEHAVIOR DURING & AFTER TRAJECTORY
# ============================================================================

def test_watchdog_during_and_after_long_trajectory():
    """Watchdog is fed continuously during a 10s trajectory and does NOT trip during or after."""
    async def _test():
        clock = ControlledClock(1000.0)
        config = ArmConfig(simulation=True, watchdog_timeout=2.0)  # 2-second timeout
        arm = SimulatedArm(config)
        arm.connect()
        arm.enable_torque(True)
        safety = SafetyMonitor(arm, config, clock_fn=clock)
        executor = TrajectoryExecutor(arm, safety, config)
        planner = TrajectoryPlanner(config)

        # 10.0-second long multi-waypoint trajectory (> 5x watchdog timeout)
        wp0 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0})
        wp1 = Waypoint(angles={1: 170.0, 2: 140.0, 3: 130.0, 4: 150.0, 5: 150.0}, duration=5.0)
        wp2 = Waypoint(angles={1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0}, duration=5.0)

        plan = planner.plan([wp0, wp1, wp2], sample_rate=10)
        assert plan.total_duration == pytest.approx(10.0)

        async def advance_clock_sleep(dt: float):
            clock.advance(dt)
            arm.update_simulation_step(dt=dt)
            status = safety.check_safety()
            assert status["safe"] is True
            assert status["watchdog_tripped"] is False

        res = await executor.execute(plan, sleep_fn=advance_clock_sleep)
        assert res["state"] == "COMPLETED"

        # Step simulation until arm finishes settling
        for _ in range(10):
            clock.advance(0.1)
            arm.update_simulation_step(dt=0.1)

        safety.end_movement_if_settled()
        assert safety.active_control is False

        # Advance clock while idle for 10.0s (> 2.0s watchdog timeout)
        clock.advance(10.0)
        status_idle = safety.check_safety()
        assert status_idle["safe"] is True
        assert status_idle["watchdog_tripped"] is False
        assert status_idle["fault_latched"] is False

        arm.disconnect()

    asyncio.run(_test())


# ============================================================================
# 8. REST API INTEGRATION TESTS
# ============================================================================

def test_api_trajectory_validate_endpoint(test_client):
    """POST /api/trajectory/validate validates waypoints and limits without moving the arm."""
    payload = {
        "waypoints": [
            {"angles": {"1": 150, "2": 150, "3": 150, "4": 150, "5": 150}, "duration": 0.0},
            {"angles": {"1": 180, "2": 130, "3": 120, "4": 160, "5": 170}, "duration": 2.0}
        ],
        "interpolation": "quintic",
        "sample_rate": 20
    }
    response = test_client.post("/api/trajectory/validate", json=payload)
    assert response.status_code == 200
    res_json = response.json()
    assert res_json["success"] is True
    data = res_json["data"]
    assert data["is_valid"] is True
    assert data["total_duration"] == pytest.approx(2.0)
    assert len(data["errors"]) == 0


def test_api_trajectory_validate_rejection(test_client):
    """POST /api/trajectory/validate reports joint limit violations."""
    payload = {
        "waypoints": [
            {"angles": {"1": 150, "2": 150, "3": 150, "4": 150, "5": 150}, "duration": 0.0},
            {"angles": {"1": 350, "2": 130, "3": 120, "4": 160, "5": 170}, "duration": 2.0}
        ],
        "interpolation": "quintic"
    }
    response = test_client.post("/api/trajectory/validate", json=payload)
    assert response.status_code == 200
    res_json = response.json()
    assert res_json["success"] is False
    data = res_json["data"]
    assert data["is_valid"] is False
    assert len(data["errors"]) > 0
    assert any("joint 1" in err.lower() for err in data["errors"])


def test_api_trajectory_preview_endpoint(test_client):
    """POST /api/trajectory/preview returns sampled time series and peak metrics without movement."""
    payload = {
        "waypoints": [
            {"angles": {"1": 150, "2": 150, "3": 150, "4": 150, "5": 150}, "duration": 0.0},
            {"angles": {"1": 170, "2": 140, "3": 130, "4": 160, "5": 180}, "duration": 1.5}
        ],
        "interpolation": "cubic",
        "sample_rate": 10
    }
    response = test_client.post("/api/trajectory/preview", json=payload)
    assert response.status_code == 200
    res_json = response.json()
    assert res_json["success"] is True
    data = res_json["data"]
    assert data["is_valid"] is True
    assert "time_points" in data
    assert "positions" in data
    assert "velocities" in data
    assert "peak_velocities" in data
    assert len(data["time_points"]) > 0
    assert len(data["positions"]["1"]) == len(data["time_points"])


def test_api_trajectory_execute_and_cancel_endpoints(test_client):
    """POST /api/trajectory/execute starts trajectory and GET /api/trajectory/status checks state."""
    payload = {
        "waypoints": [
            {"angles": {"1": 150, "2": 150, "3": 150, "4": 150, "5": 150}, "duration": 0.0},
            {"angles": {"1": 160, "2": 145, "3": 140, "4": 155, "5": 160}, "duration": 1.0}
        ],
        "interpolation": "linear",
        "sample_rate": 20
    }
    exec_res = test_client.post("/api/trajectory/execute", json=payload)
    assert exec_res.status_code == 200
    assert exec_res.json()["success"] is True

    # Check status
    status_res = test_client.get("/api/trajectory/status")
    assert status_res.status_code == 200
    status_data = status_res.json()["data"]
    assert status_data["state"] in ["RUNNING", "COMPLETED"]

    # Cancel endpoint
    cancel_res = test_client.post("/api/trajectory/cancel")
    assert cancel_res.status_code == 200
    assert "success" in cancel_res.json()

    # Allow cancellation task to conclude and settle arm simulation
    import time
    time.sleep(0.1)
    import backend.main as main_mod
    if main_mod.arm_instance:
        for _ in range(10):
            main_mod.arm_instance.update_simulation_step(dt=0.1)
    if main_mod.safety_monitor:
        main_mod.safety_monitor.end_movement_if_settled()
        main_mod.safety_monitor.stop_control_session()
