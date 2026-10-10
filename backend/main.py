"""FastAPI Backend Server for 5-DOF Dynamixel AX-12A Robotic Arm."""

import os
import asyncio
import json
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Dict, Any, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request, status, Body
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware

from backend.config import get_config, ArmConfig
from backend.arm import create_arm, BaseArm
from backend.safety import SafetyMonitor
from backend.telemetry import TelemetryManager
from backend.pose_manager import PoseManager
from backend.kinematics import KinematicsController
from backend.trajectory import (
    TrajectoryPlanner, TrajectoryExecutor, Waypoint, InterpolationType
)
from backend.models import (
    SystemState, MoveRequest, XYZMoveRequest, TorqueRequest, PoseModel,
    SequenceModel, SequenceStep, PoseSequenceRequest, APIResponse,
    WaypointModel, TrajectoryPreviewRequest, TrajectoryExecuteRequest
)
from backend.exceptions import (
    RoboticArmException, JointLimitExceededError, SafetyFaultError,
    EmergencyStopError, InvalidCommandError, HardwareConnectionError,
    ServoCommunicationError
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("robotic_arm_backend")

# Global instances
global_config: ArmConfig = get_config()
arm_instance: Optional[BaseArm] = None
safety_monitor: Optional[SafetyMonitor] = None
telemetry_manager: Optional[TelemetryManager] = None
pose_manager: Optional[PoseManager] = None
kinematics_controller: Optional[KinematicsController] = None
trajectory_planner: Optional[TrajectoryPlanner] = None
trajectory_executor: Optional[TrajectoryExecutor] = None

def init_system(config: Optional[ArmConfig] = None):
    global arm_instance, safety_monitor, telemetry_manager, pose_manager, kinematics_controller, global_config
    global trajectory_planner, trajectory_executor
    if config:
        global_config = config
    else:
        global_config = get_config()

    logging.getLogger().setLevel(global_config.logging_level.upper())

    if arm_instance is None:
        try:
            arm_instance = create_arm(global_config)
        except Exception as e:
            logger.error(f"Failed to initialize arm: {e}. Falling back to Simulation Mode.")
            global_config.simulation = True
            arm_instance = create_arm(global_config)

    if safety_monitor is None:
        safety_monitor = SafetyMonitor(arm_instance, global_config)
    if telemetry_manager is None:
        telemetry_manager = TelemetryManager(arm_instance, safety_monitor, global_config)
    if pose_manager is None:
        pose_manager = PoseManager()
    if kinematics_controller is None:
        kinematics_controller = KinematicsController(arm_instance, safety_monitor)
    if trajectory_planner is None:
        trajectory_planner = TrajectoryPlanner(global_config)
    if trajectory_executor is None:
        trajectory_executor = TrajectoryExecutor(arm_instance, safety_monitor, global_config)

# Initialize immediately for direct API access and unit tests
init_system()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application startup and shutdown lifespan context manager."""
    init_system()
    telemetry_manager.start()
    logger.info(f"Robotic Arm System Ready (Simulation={global_config.simulation})")

    yield

    # Shutdown
    logger.info("Shutting down Robotic Arm Backend...")
    if trajectory_executor:
        trajectory_executor.cancel()
    if telemetry_manager:
        telemetry_manager.stop()
    if arm_instance:
        arm_instance.disconnect()
    logger.info("Backend shutdown complete.")


app = FastAPI(
    title="5-DOF Dynamixel AX-12A Robotic Arm Controller",
    description="REST API and WebSocket Telemetry Server",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Prevent stale browser caching of frontend static assets during development
@app.middleware("http")
async def add_no_cache_headers(request: Request, call_next):
    response = await call_next(request)
    if request.url.path.startswith("/js") or request.url.path.startswith("/css") or request.url.path == "/":
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response

# Exception Handlers
@app.exception_handler(RoboticArmException)
async def robotic_arm_exception_handler(request: Request, exc: RoboticArmException):
    logger.warning(f"API Error [{exc.__class__.__name__}]: {exc}")
    status_code = status.HTTP_400_BAD_REQUEST
    if isinstance(exc, (EmergencyStopError, SafetyFaultError)):
        status_code = status.HTTP_409_CONFLICT
    elif isinstance(exc, (HardwareConnectionError, ServoCommunicationError)):
        status_code = status.HTTP_503_SERVICE_UNAVAILABLE

    return JSONResponse(
        status_code=status_code,
        content=APIResponse(success=False, message=str(exc)).model_dump()
    )

@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled server error: {exc}", exc_info=True)
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content=APIResponse(success=False, message="An internal server error occurred.").model_dump()
    )


# Serve Dashboard Static Files
BASE_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = BASE_DIR / "frontend"

if FRONTEND_DIR.exists():
    app.mount("/css", StaticFiles(directory=FRONTEND_DIR / "css"), name="css")
    app.mount("/js", StaticFiles(directory=FRONTEND_DIR / "js"), name="js")

@app.get("/", response_class=HTMLResponse)
async def get_dashboard():
    index_file = FRONTEND_DIR / "index.html"
    if index_file.exists():
        return FileResponse(index_file)
    return HTMLResponse("<h1>Robotic Arm Backend Server Running</h1><p>Frontend index.html not found.</p>")


# Core REST API Endpoints
@app.get("/api/state", response_model=SystemState)
async def get_system_state():
    """Return complete current system and servo state."""
    return telemetry_manager.get_current_system_state()

@app.get("/api/config")
async def get_system_config():
    """Return current configuration options."""
    return APIResponse(success=True, message="Configuration retrieved", data=global_config.model_dump())

@app.post("/api/move", response_model=APIResponse)
async def move_joints(request: MoveRequest):
    """Synchronized joint movement command."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot move: System is in FAULT state ({safety_monitor.fault_reason}).")

    safety_monitor.record_command()
    success = arm_instance.move_joints(request.angles, speed=request.speed)
    safety_monitor.end_movement_if_settled()
    return APIResponse(
        success=success,
        message=f"Joint movement command sent successfully (speed={request.speed}).",
        data={"angles": request.angles, "speed": request.speed}
    )

@app.get("/api/kinematics/fk", response_model=APIResponse)
async def get_forward_kinematics():
    """Return present end-effector Cartesian XYZ position computed via FK."""
    pos = kinematics_controller.get_end_effector_position()
    return APIResponse(success=True, message="End-effector position calculated", data=pos)

@app.post("/api/kinematics/validate", response_model=APIResponse)
async def validate_kinematics(req: XYZMoveRequest):
    """Validate target Cartesian coordinates and return calculated joint angles without moving."""
    result = kinematics_controller.validate_ik(req.x, req.y, req.z, pitch=req.pitch or 0.0)
    return APIResponse(
        success=result["reachable"],
        message=result["message"],
        data=result
    )

@app.post("/api/move_xyz", response_model=APIResponse)
async def move_xyz(req: XYZMoveRequest):
    """Move arm to target Cartesian XYZ coordinates using Inverse Kinematics."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot move: System is in FAULT state ({safety_monitor.fault_reason}).")

    success = kinematics_controller.move_xyz(req.x, req.y, req.z, pitch=req.pitch, speed=req.speed)
    return APIResponse(
        success=success,
        message=f"Cartesian movement command sent for XYZ ({req.x:.1f}, {req.y:.1f}, {req.z:.1f}).",
        data={"x": req.x, "y": req.y, "z": req.z, "pitch": req.pitch, "speed": req.speed}
    )

@app.post("/api/torque/0", response_model=APIResponse)
async def torque_off():
    """Disable torque on all servos."""
    arm_instance.enable_torque(False)
    return APIResponse(success=True, message="Torque disabled for all joints.")

@app.post("/api/torque/1", response_model=APIResponse)
async def torque_on():
    """Enable torque on all servos."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot enable torque: System is in FAULT state ({safety_monitor.fault_reason}).")
    arm_instance.enable_torque(True)
    return APIResponse(success=True, message="Torque enabled for all joints.")

@app.post("/api/torque", response_model=APIResponse)
async def set_torque(req: TorqueRequest):
    """Enable/disable torque on specific joint or all joints."""
    if req.enable and safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot enable torque: System is in FAULT state ({safety_monitor.fault_reason}).")
    arm_instance.enable_torque(req.enable, req.joint_id)
    target = f"joint {req.joint_id}" if req.joint_id else "all joints"
    return APIResponse(success=True, message=f"Torque set to {req.enable} for {target}.")

@app.post("/api/estop", response_model=APIResponse)
async def emergency_stop():
    """Trigger immediate Emergency Stop."""
    safety_monitor.trigger_estop()
    return APIResponse(success=True, message="EMERGENCY STOP ACTIVATED! All torque disabled.")

@app.post("/api/clear_fault", response_model=APIResponse)
async def clear_fault():
    """Attempt to clear safety fault latch."""
    safety_monitor.clear_fault()
    return APIResponse(success=True, message="Fault latch cleared successfully.")

@app.post("/api/heartbeat", response_model=APIResponse)
async def heartbeat():
    """Receive explicit control session heartbeat."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot process heartbeat: System in FAULT state ({safety_monitor.fault_reason}).")
    safety_monitor.record_heartbeat()
    return APIResponse(success=True, message="Heartbeat acknowledged.")

@app.post("/api/control_session/start", response_model=APIResponse)
async def start_control_session():
    """Explicitly start active control session requiring heartbeats."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot start control session: System in FAULT state.")
    safety_monitor.start_control_session()
    return APIResponse(success=True, message="Active control session started.")

@app.post("/api/control_session/stop", response_model=APIResponse)
async def stop_control_session():
    """Stop active control session and return to IDLE."""
    safety_monitor.stop_control_session()
    return APIResponse(success=True, message="Active control session stopped.")



# Teach / Pose API Endpoints
@app.get("/api/poses", response_model=APIResponse)
async def list_poses():
    """List all saved arm poses."""
    poses = pose_manager.list_poses()
    return APIResponse(success=True, message="Poses retrieved", data=[p.model_dump() for p in poses])

@app.post("/api/poses", response_model=APIResponse)
async def save_pose(pose: PoseModel):
    """Save custom pose."""
    saved = pose_manager.save_pose(pose.name, pose.angles, pose.speed)
    return APIResponse(success=True, message=f"Pose '{pose.name}' saved.", data=saved.model_dump())

@app.post("/api/poses/save_current", response_model=APIResponse)
async def save_current_position(name: str, speed: int = 100):
    """Capture current physical/simulated servo positions as a new pose."""
    states = arm_instance.get_state()
    current_angles = {sid: s.present_position for sid, s in states.items()}
    saved = pose_manager.save_pose(name, current_angles, speed)
    return APIResponse(success=True, message=f"Current position saved as pose '{name}'.", data=saved.model_dump())

@app.get("/api/poses/{name}", response_model=APIResponse)
async def get_pose(name: str):
    """Get single pose by name."""
    pose = pose_manager.get_pose(name)
    return APIResponse(success=True, message="Pose loaded", data=pose.model_dump())

@app.delete("/api/poses/{name}", response_model=APIResponse)
async def delete_pose(name: str):
    """Delete a saved pose."""
    pose_manager.delete_pose(name)
    return APIResponse(success=True, message=f"Pose '{name}' deleted.")

@app.post("/api/poses/{name}/execute", response_model=APIResponse)
async def execute_pose(name: str):
    """Move arm to saved pose."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot execute pose: System is in FAULT state ({safety_monitor.fault_reason}).")

    pose_manager.execute_pose(name, arm_instance, safety_monitor)
    return APIResponse(success=True, message=f"Executing pose '{name}'.")

@app.post("/api/poses/sequence", response_model=APIResponse)
async def execute_sequence(req: PoseSequenceRequest):
    """Execute sequence of poses with delays asynchronously."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError("Cannot execute sequence: System in FAULT state.")

    async def _run_sequence():
        steps = req.steps or []
        if not steps and req.pose_names:
            steps = [
                {"pose_name": name, "delay_seconds": req.delay_seconds, "speed": req.speed}
                for name in req.pose_names
            ]

        for step in steps:
            if safety_monitor.fault_latched:
                logger.warning("Sequence aborted due to safety fault.")
                break
            try:
                pose_name = step.pose_name if hasattr(step, "pose_name") else step.get("pose_name")
                delay = step.delay_seconds if hasattr(step, "delay_seconds") else step.get("delay_seconds", 1.0)
                step_speed = step.speed if hasattr(step, "speed") else step.get("speed")

                pose = pose_manager.get_pose(pose_name)
                speed = step_speed or req.speed or pose.speed
                safety_monitor.record_command()
                arm_instance.move_joints(pose.angles, speed=speed)
                safety_monitor.end_movement_if_settled()
                await asyncio.sleep(delay)
            except Exception as e:
                logger.error(f"Error executing sequence step: {e}")
                break

    asyncio.create_task(_run_sequence())
    num_steps = len(req.steps) if req.steps else len(req.pose_names or [])
    return APIResponse(success=True, message=f"Started executing sequence of {num_steps} poses.")

# Motion Sequence Storage Endpoints
@app.get("/api/sequences", response_model=APIResponse)
async def list_sequences():
    """List all saved sequences."""
    seqs = pose_manager.list_sequences()
    return APIResponse(success=True, message="Sequences retrieved", data=[s.model_dump() for s in seqs])

@app.post("/api/sequences", response_model=APIResponse)
async def save_sequence(seq: SequenceModel = Body(...)):
    """Save custom motion sequence."""
    saved = pose_manager.save_sequence(seq.name, seq.steps)
    return APIResponse(success=True, message=f"Sequence '{seq.name}' saved.", data=saved.model_dump())


@app.get("/api/sequences/{name}", response_model=APIResponse)
async def get_sequence(name: str):
    """Get single sequence by name."""
    seq = pose_manager.get_sequence(name)
    return APIResponse(success=True, message="Sequence loaded", data=seq.model_dump())

@app.delete("/api/sequences/{name}", response_model=APIResponse)
async def delete_sequence(name: str):
    """Delete a saved sequence."""
    pose_manager.delete_sequence(name)
    return APIResponse(success=True, message=f"Sequence '{name}' deleted.")

@app.post("/api/sequences/{name}/execute", response_model=APIResponse)
async def execute_saved_sequence(name: str):
    """Execute a saved sequence by name."""
    seq = pose_manager.get_sequence(name)
    req = PoseSequenceRequest(steps=seq.steps)
    return await execute_sequence(req)



# Trajectory Planning API Endpoints (Phase 1)
@app.post("/api/trajectory/validate", response_model=APIResponse)
async def validate_trajectory(req: TrajectoryPreviewRequest):
    """Validate multi-waypoint trajectory feasibility before execution."""
    try:
        interp = InterpolationType(req.interpolation.lower())
    except ValueError:
        raise InvalidCommandError(f"Unsupported interpolation type: {req.interpolation}. Choose linear, cubic, quintic, or cubic_spline.")

    wps = [Waypoint(angles=w.angles, duration=w.duration, name=w.name) for w in req.waypoints]
    is_valid, errors, warnings = trajectory_planner.validate_waypoints(
        wps,
        velocity_limits=req.velocity_limits,
        acceleration_limits=req.acceleration_limits,
        interpolation=interp
    )

    return APIResponse(
        success=is_valid,
        message="Trajectory is feasible." if is_valid else f"Validation failed ({len(errors)} error(s)).",
        data={
            "valid": is_valid,
            "is_valid": is_valid,
            "errors": errors,
            "warnings": warnings,
            "total_duration": sum(w.duration for w in wps[1:]) if len(wps) > 1 else 0.0
        }
    )

@app.post("/api/trajectory/preview", response_model=APIResponse)
async def preview_trajectory(req: TrajectoryPreviewRequest):
    """Compute and sample trajectory for 3D visualization and charts without moving the arm."""
    try:
        interp = InterpolationType(req.interpolation.lower())
    except ValueError:
        raise InvalidCommandError(f"Unsupported interpolation type: {req.interpolation}. Choose linear, cubic, quintic, or cubic_spline.")

    wps = [Waypoint(angles=w.angles, duration=w.duration, name=w.name) for w in req.waypoints]
    plan = trajectory_planner.plan(
        wps,
        interpolation=interp,
        sample_rate=req.sample_rate or 20.0,
        velocity_limits=req.velocity_limits,
        acceleration_limits=req.acceleration_limits
    )

    return APIResponse(
        success=plan.is_valid,
        message="Trajectory preview generated." if plan.is_valid else f"Preview error: {'; '.join(plan.validation_errors)}",
        data=plan.to_dict()
    )

@app.post("/api/trajectory/execute", response_model=APIResponse)
async def execute_trajectory(req: TrajectoryExecuteRequest):
    """Validate and start executing planned trajectory on the simulated arm."""
    if safety_monitor.fault_latched:
        raise SafetyFaultError(f"Cannot execute trajectory: System in FAULT state ({safety_monitor.fault_reason}).")

    try:
        interp = InterpolationType(req.interpolation.lower())
    except ValueError:
        raise InvalidCommandError(f"Unsupported interpolation type: {req.interpolation}. Choose linear, cubic, quintic, or cubic_spline.")

    wps = [Waypoint(angles=w.angles, duration=w.duration, name=w.name) for w in req.waypoints]
    plan = trajectory_planner.plan(
        wps,
        interpolation=interp,
        sample_rate=req.sample_rate or 20.0,
        velocity_limits=req.velocity_limits,
        acceleration_limits=req.acceleration_limits
    )

    if not plan.is_valid:
        raise InvalidCommandError(f"Cannot execute invalid trajectory: {'; '.join(plan.validation_errors)}")

    # Stop any prior trajectory in progress before launching new one
    if trajectory_executor.state == "RUNNING":
        trajectory_executor.cancel()
        await asyncio.sleep(0.05)

    asyncio.create_task(trajectory_executor.execute(plan))

    return APIResponse(
        success=True,
        message=f"Trajectory execution started ({plan.total_duration:.1f}s, {len(plan.waypoints)} waypoints).",
        data=trajectory_executor.get_status()
    )

@app.post("/api/trajectory/cancel", response_model=APIResponse)
async def cancel_trajectory():
    """Cancel currently executing trajectory immediately."""
    cancelled = trajectory_executor.cancel()
    return APIResponse(
        success=True,
        message="Trajectory execution cancelled." if cancelled else "No trajectory was actively running.",
        data=trajectory_executor.get_status()
    )

@app.get("/api/trajectory/status", response_model=APIResponse)
async def get_trajectory_status():
    """Get current trajectory execution state and progress."""
    return APIResponse(
        success=True,
        message="Trajectory status retrieved.",
        data=trajectory_executor.get_status()
    )



# WebSocket Endpoint
@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    """WebSocket endpoint for continuous live telemetry streaming."""
    await telemetry_manager.connect(websocket)
    try:
        # Send initial state snapshot on connection
        initial_state = telemetry_manager.get_current_system_state()
        await websocket.send_text(json.dumps(initial_state.model_dump()))

        while True:
            data_text = await websocket.receive_text()
            if data_text:
                try:
                    payload = json.loads(data_text)
                    act = payload.get("action")
                    if act == "ping":
                        await websocket.send_text(json.dumps({"event": "pong"}))
                    elif act == "heartbeat":
                        safety_monitor.record_heartbeat()
                        await websocket.send_text(json.dumps({"event": "heartbeat_ack"}))
                except Exception:
                    pass
    except (WebSocketDisconnect, RuntimeError):
        pass
    except Exception as e:
        logger.debug(f"WebSocket client error: {e}")
    finally:
        telemetry_manager.disconnect(websocket)
