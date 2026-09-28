"""Configuration manager for the Robotic Arm System."""

import os
import yaml
import logging
from typing import Dict, List, Optional
from pathlib import Path
from pydantic import BaseModel, Field

logger = logging.getLogger(__name__)

class JointLimit(BaseModel):
    min: float = 0.0
    max: float = 300.0

class ArmConfig(BaseModel):
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 1000000
    protocol: float = 1.0
    servo_ids: List[int] = Field(default_factory=lambda: [1, 2, 3, 4, 5])
    simulation: bool = True
    joint_limits: Dict[int, JointLimit] = Field(default_factory=lambda: {
        1: JointLimit(min=0.0, max=300.0),
        2: JointLimit(min=15.0, max=285.0),
        3: JointLimit(min=15.0, max=285.0),
        4: JointLimit(min=10.0, max=290.0),
        5: JointLimit(min=0.0, max=300.0),
    })
    default_speed: int = 100
    watchdog_timeout: float = 5.0
    maximum_temperature: float = 70.0
    telemetry_frequency: float = 10.0
    logging_level: str = "INFO"

def load_config(config_path: Optional[str] = None) -> ArmConfig:
    """Load configuration from config.yaml or return defaults if not found."""
    if config_path is None:
        # Resolve config.yaml relative to project root
        base_dir = Path(__file__).resolve().parent.parent
        config_path = base_dir / "config.yaml"
    else:
        config_path = Path(config_path)

    if not config_path.exists():
        logger.warning(f"Config file not found at {config_path}. Using default configuration.")
        return ArmConfig()

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        # Parse joint limits dictionary keys to integers if they are strings
        if "joint_limits" in data and isinstance(data["joint_limits"], dict):
            raw_limits = data["joint_limits"]
            parsed_limits = {}
            for k, v in raw_limits.items():
                joint_id = int(k)
                parsed_limits[joint_id] = JointLimit(
                    min=float(v.get("min", 0.0)),
                    max=float(v.get("max", 300.0))
                )
            data["joint_limits"] = parsed_limits

        config = ArmConfig(**data)
        logger.info(f"Loaded configuration from {config_path} (Simulation mode: {config.simulation})")
        return config
    except Exception as e:
        logger.error(f"Error loading configuration from {config_path}: {e}. Using defaults.")
        return ArmConfig()

# Global config instance accessor
_global_config: Optional[ArmConfig] = None

def get_config(config_path: Optional[str] = None) -> ArmConfig:
    global _global_config
    if _global_config is None:
        _global_config = load_config(config_path)
    return _global_config

def reload_config(config_path: Optional[str] = None) -> ArmConfig:
    global _global_config
    _global_config = load_config(config_path)
    return _global_config
