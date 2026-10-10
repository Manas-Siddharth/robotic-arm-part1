"""WebSocket Telemetry Manager for real-time robotic arm monitoring."""

import asyncio
import json
import logging
import time
from typing import Set, Dict, Any, Optional
from fastapi import WebSocket, WebSocketDisconnect
from backend.arm import BaseArm
from backend.safety import SafetyMonitor
from backend.config import ArmConfig, get_config
from backend.models import SystemState, ServoState

logger = logging.getLogger(__name__)

class TelemetryManager:
    """Manages WebSocket connections and streams live arm state broadcast."""

    def __init__(self, arm: BaseArm, safety_monitor: SafetyMonitor, config: Optional[ArmConfig] = None):
        self.arm = arm
        self.safety_monitor = safety_monitor
        self.config = config or get_config()
        self.active_connections: Set[WebSocket] = set()
        self.is_running: bool = False
        self._bg_task: Optional[asyncio.Task] = None

    async def connect(self, websocket: WebSocket) -> None:
        """Accept WebSocket connection."""
        await websocket.accept()
        self.active_connections.add(websocket)
        logger.info(f"WebSocket client connected. Total active clients: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket) -> None:
        """Remove WebSocket connection."""
        self.active_connections.discard(websocket)
        logger.info(f"WebSocket client disconnected. Total active clients: {len(self.active_connections)}")

    def get_current_system_state(self) -> SystemState:
        """Fetch current consolidated arm and safety state."""
        # Run safety check
        safety_info = self.safety_monitor.check_safety()

        # Determine overall system status
        if self.safety_monitor.fault_latched:
            if "EMERGENCY" in (self.safety_monitor.fault_reason or ""):
                sys_status = "ESTOP"
            else:
                sys_status = "FAULT"
        elif not self.arm.is_connected():
            sys_status = "DISCONNECTED"
        else:
            sys_status = "READY"

        try:
            servos_dict = self.arm.get_state()
            # If any servo is enabled and moving, status is MOVING (unless ESTOP/FAULT/DISCONNECTED)
            if sys_status == "READY":
                is_moving = any(s.torque_enabled and abs(s.present_position - s.goal_position) > 0.5 for s in servos_dict.values())
                if is_moving:
                    sys_status = "MOVING"
        except Exception as e:
            logger.error(f"Error reading arm state for telemetry: {e}")
            servos_dict = {}

        any_torque = any(s.torque_enabled for s in servos_dict.values()) if servos_dict else False

        return SystemState(
            connected=self.arm.is_connected(),
            simulation=getattr(self.config, "simulation", True),
            system_status=sys_status,
            torque_enabled=any_torque,
            fault_latched=self.safety_monitor.fault_latched,
            fault_reason=self.safety_monitor.fault_reason,
            servos=servos_dict,
            last_command_timestamp=self.safety_monitor.last_command_timestamp,
            watchdog_tripped=self.safety_monitor.watchdog_tripped,
            active_control=self.safety_monitor.active_control,
            telemetry_timestamp=time.time()
        )

    async def broadcast_state(self) -> None:
        """Broadcast state JSON to all active WebSocket clients."""
        # Always run health/safety check on every tick
        state = self.get_current_system_state()

        if not self.active_connections:
            return

        data = state.model_dump()
        payload = json.dumps(data)

        dead_connections = set()
        for connection in list(self.active_connections):
            try:
                await connection.send_text(payload)
            except (WebSocketDisconnect, RuntimeError, Exception):
                dead_connections.add(connection)

        for conn in dead_connections:
            self.disconnect(conn)

    async def _telemetry_loop(self) -> None:
        """Background loop pushing telemetry at configured frequency."""
        self.is_running = True
        freq = max(1.0, min(50.0, float(self.config.telemetry_frequency)))
        interval = 1.0 / freq
        logger.info(f"Telemetry broadcast loop started at {freq} Hz ({interval:.3f}s interval)")

        while self.is_running:
            try:
                await self.broadcast_state()
            except Exception as e:
                logger.error(f"Unhandled exception in telemetry loop: {e}")
            await asyncio.sleep(interval)

    def start(self) -> None:
        """Start the background telemetry streaming loop."""
        if not self.is_running:
            self.is_running = True
            self._bg_task = asyncio.create_task(self._telemetry_loop())

    def stop(self) -> None:
        """Stop the background telemetry streaming loop."""
        self.is_running = False
        if self._bg_task:
            self._bg_task.cancel()
            self._bg_task = None
        logger.info("Telemetry manager stopped.")
