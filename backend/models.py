"""Pydantic data models for REST API and WebSocket telemetry."""

from typing import Dict, List, Optional, Any
from pydantic import BaseModel, Field

class ServoState(BaseModel):
    id: int
    goal_position: float = 150.0       # degrees (0 - 300)
    present_position: float = 150.0    # degrees (0 - 300)
    raw_position: int = 512            # raw register units (0 - 1023)
    speed: int = 0                     # moving speed (0 - 1023)
    load: float = 0.0                  # load percentage or raw (-100% to +100%)
    voltage: float = 11.1              # Volts
    temperature: float = 35.0          # Celsius
    torque_enabled: bool = True
    comm_status: str = "OK"
    error_flags: int = 0

class SystemState(BaseModel):
    connected: bool = True
    simulation: bool = True
    system_status: str = "READY"        # "READY", "MOVING", "ESTOP", "FAULT"
    torque_enabled: bool = True
    fault_latched: bool = False
    fault_reason: Optional[str] = None
    servos: Dict[int, ServoState] = Field(default_factory=dict)
    last_command_timestamp: Optional[float] = None
    watchdog_tripped: bool = False

class MoveRequest(BaseModel):
    angles: Dict[int, float] = Field(
        ...,
        description="Dictionary mapping joint ID (1-5) to angle in degrees (0.0 to 300.0)"
    )
    speed: Optional[int] = Field(100, ge=1, le=1023, description="Moving speed (1 - 1023)")

class XYZMoveRequest(BaseModel):
    x: float = Field(..., description="Target X coordinate in mm")
    y: float = Field(..., description="Target Y coordinate in mm")
    z: float = Field(..., description="Target Z coordinate in mm")
    pitch: Optional[float] = Field(0.0, description="Target wrist pitch angle in degrees")
    speed: Optional[int] = Field(100, ge=1, le=1023, description="Moving speed (1 - 1023)")

class TorqueRequest(BaseModel):
    enable: bool
    joint_id: Optional[int] = None

class PoseModel(BaseModel):
    name: str
    timestamp: Optional[str] = None
    angles: Dict[int, float]
    speed: int = 100

class PoseSequenceRequest(BaseModel):
    pose_names: List[str]
    delay_seconds: float = 1.0
    speed: Optional[int] = None

class APIResponse(BaseModel):
    success: bool
    message: str
    data: Optional[Any] = None
