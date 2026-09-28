"""Teach / Save / Replay Pose System for 5-DOF Dynamixel Robotic Arm."""

import os
import json
import time
import logging
from typing import List, Dict, Optional
from pathlib import Path
from backend.models import PoseModel, PoseSequenceRequest
from backend.arm import BaseArm
from backend.safety import SafetyMonitor
from backend.exceptions import InvalidCommandError

logger = logging.getLogger(__name__)

class PoseManager:
    """Manages saving, listing, loading, deleting, and executing robotic arm poses."""

    def __init__(self, storage_dir: Optional[str] = None):
        if storage_dir is None:
            base_dir = Path(__file__).resolve().parent.parent
            self.storage_dir = base_dir / "data" / "poses"
        else:
            self.storage_dir = Path(storage_dir)

        self.storage_dir.mkdir(parents=True, exist_ok=True)

    def _get_filepath(self, name: str) -> Path:
        # Sanitize filename
        safe_name = "".join([c for c in name if c.isalnum() or c in ('_', '-')]).strip()
        if not safe_name:
            raise InvalidCommandError("Invalid pose name provided.")
        return self.storage_dir / f"{safe_name}.json"

    def save_pose(self, name: str, angles: Dict[int, float], speed: int = 100) -> PoseModel:
        """Save a joint configuration pose to JSON storage."""
        # Convert integer keys to int
        parsed_angles = {int(k): round(float(v), 2) for k, v in angles.items()}
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        pose = PoseModel(
            name=name,
            timestamp=timestamp,
            angles=parsed_angles,
            speed=max(1, min(1023, int(speed)))
        )

        filepath = self._get_filepath(name)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(pose.model_dump(), f, indent=2)

        logger.info(f"Saved pose '{name}' to {filepath}")
        return pose

    def get_pose(self, name: str) -> PoseModel:
        """Load a pose by name."""
        filepath = self._get_filepath(name)
        if not filepath.exists():
            raise InvalidCommandError(f"Pose '{name}' does not exist.")

        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        # Ensure dictionary keys in angles are integers
        if "angles" in data and isinstance(data["angles"], dict):
            data["angles"] = {int(k): float(v) for k, v in data["angles"].items()}

        return PoseModel(**data)

    def list_poses(self) -> List[PoseModel]:
        """List all saved poses."""
        poses = []
        for file in self.storage_dir.glob("*.json"):
            try:
                with open(file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if "angles" in data and isinstance(data["angles"], dict):
                    data["angles"] = {int(k): float(v) for k, v in data["angles"].items()}
                poses.append(PoseModel(**data))
            except Exception as e:
                logger.warning(f"Error reading pose file {file}: {e}")
        return sorted(poses, key=lambda p: p.name)

    def delete_pose(self, name: str) -> bool:
        """Delete a saved pose."""
        filepath = self._get_filepath(name)
        if not filepath.exists():
            raise InvalidCommandError(f"Pose '{name}' not found.")
        filepath.unlink()
        logger.info(f"Deleted pose '{name}'")
        return True

    def execute_pose(self, name: str, arm: BaseArm, safety_monitor: SafetyMonitor) -> bool:
        """Move arm to specified saved pose."""
        pose = self.get_pose(name)
        safety_monitor.record_command()
        return arm.move_joints(pose.angles, speed=pose.speed)
