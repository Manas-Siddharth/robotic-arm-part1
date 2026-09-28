"""Simulated Dynamixel Arm implementation for off-hardware development and testing."""

import time
import math
import random
import logging
from typing import Dict, Optional, List
from backend.arm import BaseArm
from backend.models import ServoState
from backend.exceptions import (
    JointLimitExceededError,
    InvalidCommandError,
    EmergencyStopError,
    SafetyFaultError
)

logger = logging.getLogger(__name__)

class SimulatedArm(BaseArm):
    """Simulates a 5-DOF Dynamixel AX-12A Robotic Arm."""

    def __init__(self, config=None):
        from backend.config import get_config
        self.config = config or get_config()
        self.servo_ids: List[int] = list(self.config.servo_ids)
        self.connected: bool = False
        self.torque_enabled: bool = True
        self.estop_active: bool = False
        self.fault_active: bool = False
        self.fault_reason: Optional[str] = None
        self.last_update_time: float = time.time()

        # Servo internal simulation states
        self.servos: Dict[int, dict] = {}
        for sid in self.servo_ids:
            self.servos[sid] = {
                "id": sid,
                "goal_position": 150.0,       # Initial homing angle (150 deg = center)
                "present_position": 150.0,    # degrees
                "speed": self.config.default_speed, # raw 1-1023
                "load": 0.0,                  # percentage (-100 to 100)
                "voltage": 11.4,              # Volts
                "temperature": 34.5,          # Celsius
                "torque_enabled": True,
                "comm_status": "OK",
                "error_flags": 0,
            }

    def connect(self) -> bool:
        """Simulate hardware connection."""
        self.connected = True
        self.estop_active = False
        self.fault_active = False
        self.fault_reason = None
        self.last_update_time = time.time()
        logger.info(f"SimulatedArm connected with servos: {self.servo_ids}")
        return True

    def disconnect(self) -> None:
        """Simulate hardware disconnection."""
        self.connected = False
        logger.info("SimulatedArm disconnected.")

    def is_connected(self) -> bool:
        return self.connected

    def ping_servo(self, joint_id: int) -> bool:
        """Simulate pinging a servo."""
        if not self.connected:
            return False
        return joint_id in self.servo_ids

    def degrees_to_raw(self, degrees: float) -> int:
        """Convert degrees (0-300) to raw position units (0-1023)."""
        clamped_deg = max(0.0, min(300.0, float(degrees)))
        return int(round((clamped_deg / 300.0) * 1023.0))

    def raw_to_degrees(self, raw: int) -> float:
        """Convert raw position units (0-1023) to degrees (0-300)."""
        clamped_raw = max(0, min(1023, int(raw)))
        return round((clamped_raw / 1023.0) * 300.0, 2)

    def validate_joint_limits(self, angles: Dict[int, float]) -> None:
        """Check requested joint angles against configured software limits."""
        for joint_id, angle in angles.items():
            if joint_id not in self.servo_ids:
                raise InvalidCommandError(f"Invalid joint ID: {joint_id}. Configured IDs: {self.servo_ids}")

            limit = self.config.joint_limits.get(joint_id)
            if limit:
                if angle < limit.min or angle > limit.max:
                    raise JointLimitExceededError(
                        f"Joint {joint_id} angle {angle:.1f}° out of bounds [{limit.min:.1f}°, {limit.max:.1f}°]"
                    )

    def enable_torque(self, enable: bool, joint_id: Optional[int] = None) -> bool:
        """Enable or disable torque on one or all servos."""
        if enable and (self.estop_active or self.fault_active):
            raise SafetyFaultError("Cannot enable torque while Emergency Stop or Safety Fault is active.")

        if joint_id is not None:
            if joint_id not in self.servos:
                raise InvalidCommandError(f"Joint ID {joint_id} not found.")
            self.servos[joint_id]["torque_enabled"] = enable
            logger.info(f"Torque {'enabled' if enable else 'disabled'} for joint {joint_id}")
        else:
            self.torque_enabled = enable
            for s in self.servos.values():
                s["torque_enabled"] = enable
            logger.info(f"Torque {'enabled' if enable else 'disabled'} for all joints.")
        return True

    def move_joints(self, angles: Dict[int, float], speed: Optional[int] = None) -> bool:
        """Simulate moving joints with validation and speed scaling."""
        if not self.connected:
            raise InvalidCommandError("Arm is not connected.")
        if self.estop_active:
            raise EmergencyStopError("Movement rejected: Emergency Stop is ACTIVE.")
        if self.fault_active:
            raise SafetyFaultError(f"Movement rejected: System is in FAULT state ({self.fault_reason}).")

        # Validate requested IDs and angles
        self.validate_joint_limits(angles)

        move_speed = speed if speed is not None else self.config.default_speed
        move_speed = max(1, min(1023, int(move_speed)))

        # Apply goal position to simulated servos
        for joint_id, target_angle in angles.items():
            servo = self.servos[int(joint_id)]
            if not servo["torque_enabled"]:
                logger.warning(f"Joint {joint_id} torque is disabled, goal updated but servo won't physically move.")
            servo["goal_position"] = round(float(target_angle), 2)
            servo["speed"] = move_speed

        logger.debug(f"SimulatedArm move_joints target: {angles}, speed: {move_speed}")
        return True

    def update_simulation_step(self, dt: Optional[float] = None) -> None:
        """Advance physical simulation state by elapsed time delta dt."""
        now = time.time()
        if dt is None:
            dt = now - self.last_update_time
        self.last_update_time = now

        dt = max(0.001, min(0.5, dt)) # Bound dt to prevent huge jumps

        for s in self.servos.values():
            if not s["torque_enabled"]:
                s["load"] = 0.0
                # Cool down temperature slightly when torque is off
                s["temperature"] = max(30.0, s["temperature"] - 0.2 * dt)
                continue

            current_pos = s["present_position"]
            goal_pos = s["goal_position"]
            diff = goal_pos - current_pos

            if abs(diff) > 0.1:
                # Speed conversion: raw 1023 corresponds to ~114 RPM = 684 deg/sec
                # speed unit ≈ 0.67 deg/sec
                deg_per_sec = max(5.0, s["speed"] * 0.67)
                step = deg_per_sec * dt

                if abs(diff) <= step:
                    s["present_position"] = goal_pos
                    s["load"] = 0.0
                else:
                    direction = 1.0 if diff > 0 else -1.0
                    s["present_position"] += direction * step
                    # Simulated motor load proportional to speed
                    s["load"] = round(direction * min(85.0, (s["speed"] / 1023.0) * 60.0 + random.uniform(5.0, 15.0)), 1)

                # Temperature increases slightly during movement
                s["temperature"] = min(75.0, s["temperature"] + 0.15 * dt)
            else:
                s["present_position"] = goal_pos
                s["load"] = round(random.uniform(-2.0, 2.0), 1)
                # Cool down towards baseline
                s["temperature"] = max(32.0, s["temperature"] - 0.1 * dt)

            # Simulated voltage around 11.4V with tiny noise
            s["voltage"] = round(11.4 + random.uniform(-0.1, 0.1), 1)

    def get_state(self) -> Dict[int, ServoState]:
        """Get state of all servos."""
        self.update_simulation_step()

        result = {}
        for sid, s in self.servos.items():
            result[sid] = ServoState(
                id=sid,
                goal_position=s["goal_position"],
                present_position=round(s["present_position"], 2),
                raw_position=self.degrees_to_raw(s["present_position"]),
                speed=s["speed"],
                load=s["load"],
                voltage=s["voltage"],
                temperature=round(s["temperature"], 1),
                torque_enabled=s["torque_enabled"],
                comm_status=s["comm_status"],
                error_flags=s["error_flags"],
            )
        return result

    def emergency_stop(self) -> bool:
        """Trigger emergency stop."""
        self.estop_active = True
        self.fault_active = True
        self.torque_enabled = False
        for s in self.servos.values():
            s["torque_enabled"] = False
            s["load"] = 0.0
        logger.warning("SimulatedArm: EMERGENCY STOP ACTIVATED!")
        return True

    def clear_fault(self) -> bool:
        """Reset E-STOP and fault status if safe."""
        # Check temperature before clearing fault
        over_temp = [s["id"] for s in self.servos.values() if s["temperature"] > self.config.maximum_temperature]
        if over_temp:
            raise SafetyFaultError(f"Cannot clear fault: Servos {over_temp} still exceed maximum temperature threshold.")

        self.estop_active = False
        self.fault_active = False
        self.fault_reason = None
        logger.info("SimulatedArm: Fault cleared.")
        return True
