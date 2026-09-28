"""Kinematics module for 5-DOF Dynamixel AX-12A Robotic Arm (FK/IK)."""

import math
import logging
from typing import Dict, Tuple, Optional
from backend.arm import BaseArm
from backend.safety import SafetyMonitor
from backend.exceptions import InvalidCommandError

logger = logging.getLogger(__name__)

# Standard 5-DOF link lengths in millimeters
L1_BASE_HEIGHT = 145.0   # Base to Shoulder joint
L2_UPPER_ARM   = 105.0   # Shoulder to Elbow joint
L3_FOREARM     = 105.0   # Elbow to Wrist joint
L4_WRIST_EFF   = 90.0    # Wrist joint to End Effector tip

class KinematicsSolver:
    """Forward and Inverse Kinematics solver for 5-DOF articulated arm."""

    def __init__(
        self,
        l1: float = L1_BASE_HEIGHT,
        l2: float = L2_UPPER_ARM,
        l3: float = L3_FOREARM,
        l4: float = L4_WRIST_EFF
    ):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    def forward_kinematics(self, angles: Dict[int, float]) -> Dict[str, float]:
        """Compute end-effector Cartesian XYZ coordinates from joint angles (in degrees).

        Angles mapping:
        J1: Base yaw (150° = 0 rad, 0° = -150°, 300° = +150°)
        J2: Shoulder pitch (150° = horizontal, 240° = vertical up)
        J3: Elbow pitch relative to upper arm
        J4: Wrist pitch relative to forearm
        J5: Wrist roll (gripper rotation)
        """
        j1 = math.radians(angles.get(1, 150.0) - 150.0)
        j2 = math.radians(angles.get(2, 150.0) - 150.0)
        j3 = math.radians(angles.get(3, 150.0) - 150.0)
        j4 = math.radians(angles.get(4, 150.0) - 150.0)

        # Planar reach projection in 2D (r, z)
        r = (self.l2 * math.cos(j2) +
             self.l3 * math.cos(j2 + j3) +
             self.l4 * math.cos(j2 + j3 + j4))

        z = self.l1 + (self.l2 * math.sin(j2) +
                       self.l3 * math.sin(j2 + j3) +
                       self.l4 * math.sin(j2 + j3 + j4))

        x = r * math.cos(j1)
        y = r * math.sin(j1)

        return {
            "x": round(x, 2),
            "y": round(y, 2),
            "z": round(z, 2),
            "pitch": round(math.degrees(j2 + j3 + j4), 2)
        }

    def inverse_kinematics(self, x: float, y: float, z: float, pitch_deg: float = 0.0) -> Dict[int, float]:
        """Compute target joint angles (in degrees) for desired Cartesian XYZ coordinates."""
        # Base angle J1
        j1_rad = math.atan2(y, x)
        j1_deg = math.degrees(j1_rad) + 150.0

        r = math.hypot(x, y)
        pitch_rad = math.radians(pitch_deg)

        # Wrist position (r_w, z_w)
        r_w = r - self.l4 * math.cos(pitch_rad)
        z_w = (z - self.l1) - self.l4 * math.sin(pitch_rad)

        d_sq = r_w**2 + z_w**2
        d = math.sqrt(d_sq)

        if d > (self.l2 + self.l3) or d < abs(self.l2 - self.l3):
            raise InvalidCommandError(f"Target position ({x:.1f}, {y:.1f}, {z:.1f}) is out of physical arm reach.")

        # Law of Cosines for J3
        cos_j3 = (d_sq - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_j3 = max(-1.0, min(1.0, cos_j3))
        j3_rad = -math.acos(cos_j3) # Elbow up configuration

        # Angle to wrist from shoulder
        alpha = math.atan2(z_w, r_w)
        beta = math.atan2(self.l3 * math.sin(abs(j3_rad)), self.l2 + self.l3 * math.cos(j3_rad))
        j2_rad = alpha + beta

        # Wrist angle J4
        j4_rad = pitch_rad - (j2_rad + j3_rad)

        # Convert back to joint angle offsets centered at 150.0°
        j2_deg = math.degrees(j2_rad) + 150.0
        j3_deg = math.degrees(j3_rad) + 150.0
        j4_deg = math.degrees(j4_rad) + 150.0
        j5_deg = 150.0 # Default centered roll

        return {
            1: round(max(0.0, min(300.0, j1_deg)), 2),
            2: round(max(0.0, min(300.0, j2_deg)), 2),
            3: round(max(0.0, min(300.0, j3_deg)), 2),
            4: round(max(0.0, min(300.0, j4_deg)), 2),
            5: round(j5_deg, 2)
        }


class KinematicsController:
    """Wrapper that resolves IK targets and routes joint movements through the Arm abstraction layer."""

    def __init__(self, arm: BaseArm, safety_monitor: SafetyMonitor):
        self.arm = arm
        self.safety_monitor = safety_monitor
        self.solver = KinematicsSolver()

    def move_xyz(self, x: float, y: float, z: float, pitch: float = 0.0, speed: Optional[int] = None) -> bool:
        """Compute IK for target XYZ and execute synchronized joint movement through Arm abstraction."""
        target_angles = self.solver.inverse_kinematics(x, y, z, pitch)
        self.safety_monitor.record_command()
        return self.arm.move_joints(target_angles, speed=speed)

    def get_end_effector_position(self) -> Dict[str, float]:
        """Get present end-effector position derived from current joint positions."""
        states = self.arm.get_state()
        angles = {sid: s.present_position for sid, s in states.items()}
        return self.solver.forward_kinematics(angles)
