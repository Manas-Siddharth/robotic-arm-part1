"""Trajectory Planning and Execution Module for 5-DOF Dynamixel Robotic Arm.

Supports:
- Joint-space trajectories for all five arm joints (IDs 1-5).
- Multiple waypoints with configurable segment durations.
- Linear, Cubic, and Quintic interpolation profiles.
- Continuous multi-segment waypoint joining.
- Analytical position, velocity, and acceleration evaluation.
- Full feasibility validation (joint limits, timing, velocity & acceleration limits).
- Discrete sampling for previews and execution.
- Safe integration with SimulatedArm and SafetyMonitor with cancellation and fault handling.
"""

import time
import math
import logging
import asyncio
from enum import Enum
from typing import List, Dict, Tuple, Optional, Any, Callable
from dataclasses import dataclass, field

from backend.arm import BaseArm
from backend.safety import SafetyMonitor
from backend.config import ArmConfig, get_config
from backend.exceptions import (
    InvalidCommandError,
    SafetyFaultError,
    EmergencyStopError,
    JointLimitExceededError
)

logger = logging.getLogger(__name__)


class InterpolationType(str, Enum):
    """Supported trajectory interpolation profiles."""
    LINEAR = "linear"
    CUBIC = "cubic"
    QUINTIC = "quintic"
    CUBIC_SPLINE = "cubic_spline"


@dataclass
class Waypoint:
    """A waypoint in joint space with segment transition duration."""
    angles: Dict[int, float]
    duration: float = 2.0  # Duration in seconds to reach this waypoint from previous
    name: Optional[str] = None

    def __post_init__(self):
        # Ensure integer keys and float values
        self.angles = {int(k): float(v) for k, v in self.angles.items()}
        self.duration = float(self.duration)


@dataclass
class TrajectorySample:
    """State of all joints at an instantaneous time point."""
    time: float
    positions: Dict[int, float]
    velocities: Dict[int, float]
    accelerations: Dict[int, float]


@dataclass
class TrajectoryPlan:
    """Complete computed trajectory plan with metadata and samples."""
    waypoints: List[Waypoint]
    interpolation: InterpolationType
    sample_rate: float
    total_duration: float
    segment_durations: List[float]
    segment_start_times: List[float]
    time_points: List[float]
    positions: Dict[int, List[float]]
    velocities: Dict[int, List[float]]
    accelerations: Dict[int, List[float]]
    peak_velocities: Dict[int, float]
    peak_accelerations: Dict[int, float]
    is_valid: bool
    validation_errors: List[str] = field(default_factory=list)
    validation_warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Convert plan to serializable dictionary for API responses."""
        return {
            "waypoints": [
                {"angles": w.angles, "duration": w.duration, "name": w.name}
                for w in self.waypoints
            ],
            "interpolation": self.interpolation.value,
            "sample_rate": self.sample_rate,
            "total_duration": round(self.total_duration, 4),
            "segment_durations": [round(d, 4) for d in self.segment_durations],
            "time_points": [round(t, 4) for t in self.time_points],
            "positions": {str(k): [round(p, 2) for p in v] for k, v in self.positions.items()},
            "velocities": {str(k): [round(vel, 2) for vel in v] for k, v in self.velocities.items()},
            "accelerations": {str(k): [round(acc, 2) for acc in v] for k, v in self.accelerations.items()},
            "peak_velocities": {str(k): round(v, 2) for k, v in self.peak_velocities.items()},
            "peak_accelerations": {str(k): round(v, 2) for k, v in self.peak_accelerations.items()},
            "is_valid": self.is_valid,
            "validation_errors": self.validation_errors,
            "validation_warnings": self.validation_warnings,
        }


class TrajectoryPlanner:
    """Plans, validates, and samples multi-joint trajectories."""

    DEFAULT_SERVO_IDS = [1, 2, 3, 4, 5]
    
    # Conservative physical defaults for Dynamixel AX-12A (deg/s and deg/s^2)
    DEFAULT_VELOCITY_LIMITS = {1: 180.0, 2: 150.0, 3: 150.0, 4: 180.0, 5: 200.0}
    DEFAULT_ACCEL_LIMITS = {1: 360.0, 2: 300.0, 3: 300.0, 4: 360.0, 5: 400.0}

    def __init__(self, config: Optional[ArmConfig] = None):
        self.config = config or get_config()
        self.servo_ids = list(getattr(self.config, "servo_ids", self.DEFAULT_SERVO_IDS))
        
        # Load joint limits from configuration
        self.joint_limits = {}
        if hasattr(self.config, "joint_limits") and self.config.joint_limits:
            for sid, lim in self.config.joint_limits.items():
                self.joint_limits[int(sid)] = (float(lim.min), float(lim.max))
        else:
            for sid in self.servo_ids:
                self.joint_limits[sid] = (0.0, 300.0)

        # Load velocity / acceleration limits with defaults
        self.velocity_limits = dict(getattr(self.config, "trajectory_velocity_limits", self.DEFAULT_VELOCITY_LIMITS))
        self.accel_limits = dict(getattr(self.config, "trajectory_acceleration_limits", self.DEFAULT_ACCEL_LIMITS))

    def validate_waypoints(
        self,
        waypoints: List[Waypoint],
        velocity_limits: Optional[Dict[int, float]] = None,
        acceleration_limits: Optional[Dict[int, float]] = None,
        interpolation: InterpolationType = InterpolationType.QUINTIC
    ) -> Tuple[bool, List[str], List[str]]:
        """Validate waypoints for joint bounds, timing, velocity, and acceleration limits.
        
        Returns:
            Tuple of (is_valid, errors, warnings).
        """
        errors: List[str] = []
        warnings: List[str] = []

        v_limits = velocity_limits or self.velocity_limits
        a_limits = acceleration_limits or self.accel_limits

        if not waypoints or len(waypoints) < 2:
            errors.append("Trajectory requires at least 2 waypoints (start and target).")
            return False, errors, warnings

        # 1. Joint presence & software limits for all waypoints
        for idx, wp in enumerate(waypoints):
            for sid in self.servo_ids:
                if sid not in wp.angles:
                    errors.append(f"Waypoint {idx} missing joint ID {sid}.")
                else:
                    angle = wp.angles[sid]
                    if sid in self.joint_limits:
                        j_min, j_max = self.joint_limits[sid]
                        if angle < j_min or angle > j_max:
                            errors.append(
                                f"Waypoint {idx} Joint {sid} angle {angle:.1f}° exceeds software limits [{j_min:.1f}°, {j_max:.1f}°]."
                            )

            # Segment timing (for idx > 0)
            if idx > 0:
                if wp.duration <= 0.0:
                    errors.append(f"Segment {idx} (Waypoint {idx-1} -> {idx}): Invalid duration ({wp.duration}s). Duration must be strictly positive (> 0).")
                elif wp.duration < 0.05:
                    warnings.append(f"Segment {idx} duration {wp.duration}s is very short (< 50ms); may require high acceleration.")

        if errors:
            return False, errors, warnings

        # 2. Check kinematic feasibility across each segment
        for idx in range(1, len(waypoints)):
            wp_prev = waypoints[idx - 1]
            wp_curr = waypoints[idx]
            T = wp_curr.duration

            for sid in self.servo_ids:
                delta = abs(wp_curr.angles[sid] - wp_prev.angles[sid])
                v_max = v_limits.get(sid, 180.0)
                a_max = a_limits.get(sid, 360.0)

                # Peak velocity calculations
                if interpolation == InterpolationType.LINEAR:
                    v_peak = delta / T
                    a_peak = 0.0
                    if idx == 1:
                        warnings.append("Linear interpolation has velocity step discontinuities at segment boundaries.")
                elif interpolation == InterpolationType.CUBIC:
                    # s(u) = 3u^2 - 2u^3 => max s'(u) = 1.5 at u=0.5
                    v_peak = 1.5 * delta / T
                    # max |s''(u)| = 6.0 at u=0 or 1
                    a_peak = 6.0 * delta / (T ** 2)
                elif interpolation == InterpolationType.QUINTIC:
                    # s(u) = 10u^3 - 15u^4 + 6u^5 => max s'(u) = 1.875 at u=0.5
                    v_peak = 1.875 * delta / T
                    # max |s''(u)| = 10 / sqrt(3) ≈ 5.7735 at u = (3 ± sqrt(3))/6
                    a_peak = (10.0 / math.sqrt(3.0)) * delta / (T ** 2)
                elif interpolation == InterpolationType.CUBIC_SPLINE:
                    # Conservative estimate for spline peak velocity
                    v_peak = 1.75 * delta / T
                    a_peak = 6.5 * delta / (T ** 2)
                else:
                    v_peak = delta / T
                    a_peak = 0.0

                if v_peak > v_max + 1e-4:
                    errors.append(
                        f"Segment {idx} Joint {sid}: Velocity limit exceeded. Required peak velocity {v_peak:.1f}°/s exceeds limit {v_max:.1f}°/s. "
                        f"Increase duration to at least {self.estimate_min_duration(delta, v_max, a_max, interpolation):.2f}s."
                    )

                if a_peak > a_max + 1e-4:
                    errors.append(
                        f"Segment {idx} Joint {sid}: Acceleration limit exceeded. Required peak acceleration {a_peak:.1f}°/s² exceeds limit {a_max:.1f}°/s². "
                        f"Increase duration to at least {self.estimate_min_duration(delta, v_max, a_max, interpolation):.2f}s."
                    )

        is_valid = len(errors) == 0
        return is_valid, errors, warnings

    @staticmethod
    def estimate_min_duration(
        delta: float,
        v_max: float,
        a_max: float,
        interpolation: InterpolationType = InterpolationType.QUINTIC
    ) -> float:
        """Estimate minimum required segment duration for given delta and limits."""
        if delta <= 0.0:
            return 0.1

        if interpolation == InterpolationType.QUINTIC:
            t_v = 1.875 * delta / v_max
            t_a = math.sqrt((10.0 / math.sqrt(3.0)) * delta / a_max)
        elif interpolation == InterpolationType.CUBIC:
            t_v = 1.5 * delta / v_max
            t_a = math.sqrt(6.0 * delta / a_max)
        else:
            t_v = delta / v_max
            t_a = 0.05

        return max(0.1, max(t_v, t_a))

    def plan(
        self,
        waypoints: List[Waypoint],
        interpolation: InterpolationType = InterpolationType.QUINTIC,
        sample_rate: float = 20.0,
        velocity_limits: Optional[Dict[int, float]] = None,
        acceleration_limits: Optional[Dict[int, float]] = None
    ) -> TrajectoryPlan:
        """Compute full trajectory plan with analytical sampling."""
        sample_rate = max(1.0, min(200.0, float(sample_rate)))

        is_valid, errors, warnings = self.validate_waypoints(
            waypoints,
            velocity_limits=velocity_limits,
            acceleration_limits=acceleration_limits,
            interpolation=interpolation
        )

        num_segments = len(waypoints) - 1
        segment_durations = [waypoints[i].duration for i in range(1, len(waypoints))]
        segment_start_times = [0.0]
        for d in segment_durations[:-1]:
            segment_start_times.append(segment_start_times[-1] + d)

        total_duration = sum(segment_durations) if segment_durations else 0.0

        if not is_valid or total_duration <= 0.0:
            # Return empty/invalid plan
            return TrajectoryPlan(
                waypoints=waypoints,
                interpolation=interpolation,
                sample_rate=sample_rate,
                total_duration=total_duration,
                segment_durations=segment_durations,
                segment_start_times=segment_start_times,
                time_points=[],
                positions={sid: [] for sid in self.servo_ids},
                velocities={sid: [] for sid in self.servo_ids},
                accelerations={sid: [] for sid in self.servo_ids},
                peak_velocities={sid: 0.0 for sid in self.servo_ids},
                peak_accelerations={sid: 0.0 for sid in self.servo_ids},
                is_valid=False,
                validation_errors=errors,
                validation_warnings=warnings
            )

        # Generate discrete time sampling points
        dt = 1.0 / sample_rate
        num_samples = int(math.ceil(total_duration * sample_rate)) + 1
        time_points = [round(i * dt, 6) for i in range(num_samples)]
        if time_points[-1] < total_duration:
            time_points.append(total_duration)
        elif time_points[-1] > total_duration:
            time_points[-1] = total_duration

        # Compute positions, velocities, accelerations for all joints
        positions: Dict[int, List[float]] = {sid: [] for sid in self.servo_ids}
        velocities: Dict[int, List[float]] = {sid: [] for sid in self.servo_ids}
        accelerations: Dict[int, List[float]] = {sid: [] for sid in self.servo_ids}

        peak_v: Dict[int, float] = {sid: 0.0 for sid in self.servo_ids}
        peak_a: Dict[int, float] = {sid: 0.0 for sid in self.servo_ids}

        # Precompute spline velocities if CUBIC_SPLINE requested
        spline_velocities: Optional[Dict[int, List[float]]] = None
        if interpolation == InterpolationType.CUBIC_SPLINE and num_segments >= 1:
            spline_velocities = self._compute_spline_velocities(waypoints, segment_durations)

        for t in time_points:
            sample = self._evaluate_at_time(
                t, waypoints, segment_durations, segment_start_times, total_duration, interpolation, spline_velocities
            )
            for sid in self.servo_ids:
                p = sample.positions[sid]
                v = sample.velocities[sid]
                a = sample.accelerations[sid]

                positions[sid].append(p)
                velocities[sid].append(v)
                accelerations[sid].append(a)

                if abs(v) > peak_v[sid]:
                    peak_v[sid] = abs(v)
                if abs(a) > peak_a[sid]:
                    peak_a[sid] = abs(a)

        return TrajectoryPlan(
            waypoints=waypoints,
            interpolation=interpolation,
            sample_rate=sample_rate,
            total_duration=total_duration,
            segment_durations=segment_durations,
            segment_start_times=segment_start_times,
            time_points=time_points,
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
            peak_velocities=peak_v,
            peak_accelerations=peak_a,
            is_valid=True,
            validation_errors=[],
            validation_warnings=warnings
        )

    def _compute_spline_velocities(
        self, waypoints: List[Waypoint], segment_durations: List[float]
    ) -> Dict[int, List[float]]:
        """Compute boundary-matched intermediate velocities for cubic splines."""
        num_wp = len(waypoints)
        vels: Dict[int, List[float]] = {sid: [0.0] * num_wp for sid in self.servo_ids}

        # Rest at endpoints: v_0 = 0, v_N = 0
        for sid in self.servo_ids:
            vels[sid][0] = 0.0
            vels[sid][-1] = 0.0

            for k in range(1, num_wp - 1):
                h_prev = segment_durations[k - 1]
                h_curr = segment_durations[k]
                q_prev = waypoints[k - 1].angles[sid]
                q_curr = waypoints[k].angles[sid]
                q_next = waypoints[k + 1].angles[sid]

                # Centered difference weighted by segment times
                slope_prev = (q_curr - q_prev) / h_prev
                slope_curr = (q_next - q_curr) / h_curr

                # Standard Catmull-Rom style slope
                vels[sid][k] = (q_next - q_prev) / (h_prev + h_curr)

                # Clamp if direction changes to prevent extreme overshooting
                if slope_prev * slope_curr <= 0:
                    vels[sid][k] = 0.0

        return vels

    def _evaluate_at_time(
        self,
        t: float,
        waypoints: List[Waypoint],
        segment_durations: List[float],
        segment_start_times: List[float],
        total_duration: float,
        interpolation: InterpolationType,
        spline_velocities: Optional[Dict[int, List[float]]] = None
    ) -> TrajectorySample:
        """Evaluate analytical position, velocity, acceleration across trajectory at time t."""
        num_segments = len(segment_durations)

        if t <= 0.0:
            # Start position
            wp0 = waypoints[0]
            return TrajectorySample(
                time=0.0,
                positions={sid: wp0.angles[sid] for sid in self.servo_ids},
                velocities={sid: 0.0 for sid in self.servo_ids},
                accelerations={sid: 0.0 for sid in self.servo_ids}
            )

        if t >= total_duration:
            # End position
            wp_end = waypoints[-1]
            return TrajectorySample(
                time=total_duration,
                positions={sid: wp_end.angles[sid] for sid in self.servo_ids},
                velocities={sid: 0.0 for sid in self.servo_ids},
                accelerations={sid: 0.0 for sid in self.servo_ids}
            )

        # Locate segment index
        seg_idx = 0
        for k in range(num_segments - 1, -1, -1):
            if t >= segment_start_times[k]:
                seg_idx = k
                break

        t_start = segment_start_times[seg_idx]
        T = segment_durations[seg_idx]
        u = max(0.0, min(1.0, (t - t_start) / T))

        wp_from = waypoints[seg_idx]
        wp_to = waypoints[seg_idx + 1]

        positions: Dict[int, float] = {}
        velocities: Dict[int, float] = {}
        accelerations: Dict[int, float] = {}

        for sid in self.servo_ids:
            q0 = wp_from.angles[sid]
            q1 = wp_to.angles[sid]
            delta = q1 - q0

            if interpolation == InterpolationType.LINEAR:
                # s(u) = u
                pos = q0 + delta * u
                vel = delta / T
                acc = 0.0

            elif interpolation == InterpolationType.CUBIC:
                # s(u) = 3u^2 - 2u^3
                s = 3.0 * (u ** 2) - 2.0 * (u ** 3)
                ds = 6.0 * u - 6.0 * (u ** 2)
                d2s = 6.0 - 12.0 * u

                pos = q0 + delta * s
                vel = (delta / T) * ds
                acc = (delta / (T ** 2)) * d2s

            elif interpolation == InterpolationType.QUINTIC:
                # s(u) = 10u^3 - 15u^4 + 6u^5
                u2 = u * u
                u3 = u2 * u
                u4 = u3 * u
                u5 = u4 * u

                s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
                ds = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
                d2s = 60.0 * u - 180.0 * u2 + 120.0 * u3

                pos = q0 + delta * s
                vel = (delta / T) * ds
                acc = (delta / (T ** 2)) * d2s

            elif interpolation == InterpolationType.CUBIC_SPLINE:
                # Hermite cubic with continuous velocities
                v0 = spline_velocities[sid][seg_idx] if spline_velocities else 0.0
                v1 = spline_velocities[sid][seg_idx + 1] if spline_velocities else 0.0

                u2 = u * u
                u3 = u2 * u

                h00 = 2.0 * u3 - 3.0 * u2 + 1.0
                h10 = u3 - 2.0 * u2 + u
                h01 = -2.0 * u3 + 3.0 * u2
                h11 = u3 - u2

                dh00 = 6.0 * u2 - 6.0 * u
                dh10 = 3.0 * u2 - 4.0 * u + 1.0
                dh01 = -6.0 * u2 + 6.0 * u
                dh11 = 3.0 * u2 - 2.0 * u

                d2h00 = 12.0 * u - 6.0
                d2h10 = 6.0 * u - 4.0
                d2h01 = -12.0 * u + 6.0
                d2h11 = 6.0 * u - 2.0

                pos = h00 * q0 + h10 * T * v0 + h01 * q1 + h11 * T * v1
                vel = (dh00 * q0 + dh10 * T * v0 + dh01 * q1 + dh11 * T * v1) / T
                acc = (d2h00 * q0 + d2h10 * T * v0 + d2h01 * q1 + d2h11 * T * v1) / (T ** 2)

            else:
                # Default linear
                pos = q0 + delta * u
                vel = delta / T
                acc = 0.0

            positions[sid] = pos
            velocities[sid] = vel
            accelerations[sid] = acc

        return TrajectorySample(
            time=t,
            positions=positions,
            velocities=velocities,
            accelerations=accelerations
        )


class TrajectoryExecutor:
    """Safely executes planned trajectories on the arm simulation with cancellation and safety monitoring."""

    def __init__(self, arm: BaseArm, safety_monitor: SafetyMonitor, config: Optional[ArmConfig] = None):
        self.arm = arm
        self.safety_monitor = safety_monitor
        self.config = config or get_config()

        self.state: str = "IDLE"  # "IDLE", "RUNNING", "COMPLETED", "CANCELLED", "FAULT"
        self.current_time: float = 0.0
        self.total_duration: float = 0.0
        self.progress_percent: float = 0.0
        self.current_waypoint_index: int = 0
        self.message: str = "Ready"

        self._active_task: Optional[asyncio.Task] = None
        self._cancel_event = asyncio.Event()
        self._lock = asyncio.Lock()

    def get_status(self) -> Dict[str, Any]:
        """Return current trajectory execution status."""
        return {
            "state": self.state,
            "current_time": round(self.current_time, 3),
            "total_duration": round(self.total_duration, 3),
            "progress_percent": round(self.progress_percent, 1),
            "current_waypoint_index": self.current_waypoint_index,
            "message": self.message,
            "is_running": self.state == "RUNNING"
        }

    def cancel(self) -> bool:
        """Cancel currently active trajectory execution immediately."""
        if self.state == "RUNNING":
            logger.info("Trajectory cancellation requested by user.")
            self._cancel_event.set()
            self.state = "CANCELLED"
            self.message = "Trajectory execution cancelled by user."
            return True
        return False

    async def execute(
        self,
        plan: TrajectoryPlan,
        clock_fn: Optional[Callable[[], float]] = None,
        sleep_fn: Optional[Callable[[float], Any]] = None
    ) -> Dict[str, Any]:
        """Execute trajectory asynchronously step-by-step with real-time safety monitoring."""
        async with self._lock:
            # 1. Validation checks before moving
            if not plan.is_valid:
                err_msg = "; ".join(plan.validation_errors) if plan.validation_errors else "Trajectory is invalid."
                self.state = "FAULT"
                self.message = f"Validation failed: {err_msg}"
                raise InvalidCommandError(self.message)

            if self.safety_monitor.fault_latched:
                self.state = "FAULT"
                self.message = f"Cannot execute trajectory: System in FAULT state ({self.safety_monitor.fault_reason})"
                raise SafetyFaultError(self.message)

            if not self.arm.is_connected():
                self.state = "FAULT"
                self.message = "Cannot execute trajectory: Arm is disconnected."
                raise InvalidCommandError(self.message)

            # Ensure torque is enabled before movement
            states = self.arm.get_state()
            if not any(s.torque_enabled for s in states.values()):
                self.arm.enable_torque(True)

            # 2. Reset executor state
            self.state = "RUNNING"
            self.current_time = 0.0
            self.total_duration = plan.total_duration
            self.progress_percent = 0.0
            self.current_waypoint_index = 0
            self.message = f"Executing {plan.interpolation.value} trajectory ({len(plan.waypoints)} waypoints, {plan.total_duration:.1f}s)"
            self._cancel_event.clear()

            # Start active safety control session
            self.safety_monitor.start_control_session()

            time_points = plan.time_points
            num_points = len(time_points)

            if num_points <= 1:
                self.state = "COMPLETED"
                self.progress_percent = 100.0
                self.safety_monitor.stop_control_session()
                return self.get_status()

            dt = 1.0 / plan.sample_rate
            _sleep = sleep_fn or asyncio.sleep

            try:
                for i in range(num_points):
                    # Check cancellation
                    if self._cancel_event.is_set():
                        self.state = "CANCELLED"
                        self.message = "Trajectory execution cancelled."
                        logger.warning("Trajectory execution stopped due to user cancellation.")
                        break

                    # Check safety monitor fault latch or arm emergency stop
                    if self.safety_monitor.fault_latched or getattr(self.arm, "estop_active", False):
                        self.state = "FAULT"
                        self.message = f"Trajectory stopped: Safety fault ({self.safety_monitor.fault_reason})"
                        logger.critical(f"Trajectory execution aborted due to safety fault: {self.safety_monitor.fault_reason}")
                        break

                    t_curr = time_points[i]
                    self.current_time = t_curr
                    self.progress_percent = min(100.0, (t_curr / plan.total_duration) * 100.0)

                    # Determine active segment / waypoint
                    for w_idx in range(len(plan.segment_start_times) - 1, -1, -1):
                        if t_curr >= plan.segment_start_times[w_idx]:
                            self.current_waypoint_index = w_idx + 1
                            break

                    # Step positions
                    step_angles = {sid: plan.positions[sid][i] for sid in plan.positions}

                    # Feed watchdog heartbeat
                    self.safety_monitor.record_command()

                    # Send synchronized servo positions
                    # Convert peak velocity to AX-12A raw speed units (~0.67 deg/s per speed unit)
                    max_step_vel = max(abs(plan.velocities[sid][i]) for sid in plan.velocities) if plan.velocities else 50.0
                    cmd_speed = max(10, min(1023, int(round(max_step_vel / 0.67)) if max_step_vel > 0 else 100))
                    self.arm.move_joints(step_angles, speed=cmd_speed)
                    if hasattr(self.arm, "update_simulation_step"):
                        self.arm.update_simulation_step(dt=dt)

                    # Advance time
                    if i < num_points - 1:
                        await _sleep(dt)

                if self.state == "RUNNING":
                    # Final step reached successfully
                    self.state = "COMPLETED"
                    self.current_time = plan.total_duration
                    self.progress_percent = 100.0
                    self.current_waypoint_index = len(plan.waypoints) - 1
                    self.message = f"Trajectory completed successfully ({plan.total_duration:.1f}s)."
                    logger.info("Trajectory completed successfully.")

                    # Ensure exact final waypoint is set
                    final_angles = plan.waypoints[-1].angles
                    self.arm.move_joints(final_angles)
                    if hasattr(self.arm, "update_simulation_step"):
                        self.arm.update_simulation_step(dt=0.2)

            except Exception as e:
                self.state = "FAULT"
                self.message = f"Execution error: {str(e)}"
                logger.error(f"Unexpected error during trajectory execution: {e}", exc_info=True)
                raise
            finally:
                # Safely conclude control session
                self.safety_monitor.end_movement_if_settled()
                self.safety_monitor.stop_control_session()

            return self.get_status()
