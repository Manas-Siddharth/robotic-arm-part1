"""Safety Monitoring System for 5-DOF Dynamixel Robotic Arm."""

import time
import logging
from typing import Optional, Dict, Any
from backend.arm import BaseArm
from backend.config import ArmConfig, get_config
from backend.exceptions import SafetyFaultError, EmergencyStopError

logger = logging.getLogger(__name__)

class SafetyMonitor:
    """Monitors arm health, watchdog timer, temperature limits, and fault states."""

    def __init__(self, arm: BaseArm, config: Optional[ArmConfig] = None, clock_fn=None):
        self.arm = arm
        self.config = config or get_config()
        self._clock_fn = clock_fn or time.time
        self.last_command_timestamp: Optional[float] = None
        self.active_control: bool = False
        self.control_session_explicit: bool = False
        self.fault_latched: bool = False
        self.fault_reason: Optional[str] = None
        self.watchdog_tripped: bool = False
        self.watchdog_enabled: bool = True
        self._motion_active: bool = False
        self._prev_positions: Optional[Dict[int, float]] = None
        self._last_progress_time: float = self._now()

    def _now(self) -> float:
        """Return current time from configured clock function."""
        return float(self._clock_fn())

    def record_command(self) -> None:
        """Update last command timestamp to reset the watchdog timer during active control."""
        if not self.fault_latched:
            self.last_command_timestamp = self._now()
            self.active_control = True
            if self.watchdog_tripped:
                self.watchdog_tripped = False

    def record_heartbeat(self) -> None:
        """Update heartbeat timestamp for active control session."""
        if not self.fault_latched and self.active_control:
            self.last_command_timestamp = self._now()

    def start_control_session(self) -> None:
        """Explicitly start an active control session requiring heartbeat monitoring."""
        if not self.fault_latched:
            self.control_session_explicit = True
            self.active_control = True
            self.last_command_timestamp = self._now()
            self._last_progress_time = self._now()

    def stop_control_session(self) -> None:
        """Explicitly end active control session and return to idle."""
        self.control_session_explicit = False
        self.active_control = False
        self._motion_active = False

    def end_movement_if_settled(self) -> None:
        """End active control if servos are already settled and no explicit session is active."""
        if not self.control_session_explicit:
            try:
                states = self.arm.get_state()
                is_moving = any(s.torque_enabled and abs(s.present_position - s.goal_position) > 0.5 for s in states.values())
                if not is_moving:
                    self.active_control = False
                    self._motion_active = False
            except Exception as e:
                logger.debug(f"Could not check arm state in end_movement_if_settled: {e}")

    def enable_watchdog(self, enable: bool = True) -> None:
        """Enable or disable watchdog timer checking."""
        self.watchdog_enabled = enable

    def check_safety(self) -> Dict[str, Any]:
        """Perform periodic health check on the arm.

        Checks:
        1. Over-temperature threshold on all servos.
        2. Active movement progress vs motion completion.
        3. Watchdog timeout duration during active control or explicit control session.
        """
        now = self._now()
        status = {
            "safe": not self.fault_latched,
            "fault_latched": self.fault_latched,
            "fault_reason": self.fault_reason,
            "watchdog_tripped": self.watchdog_tripped,
            "overtemp_servos": []
        }

        if not self.arm.is_connected():
            return status

        try:
            states = self.arm.get_state()

            # 1. Temperature Protection Check
            overtemp = []
            for sid, servo in states.items():
                if servo.temperature > self.config.maximum_temperature:
                    overtemp.append((sid, servo.temperature))

            if overtemp:
                status["safe"] = False
                status["overtemp_servos"] = overtemp
                reason = f"OVER-TEMPERATURE DETECTED on servo(s): {', '.join([f'ID {s} ({t}°C)' for s, t in overtemp])}"
                self._trigger_safety_fault(reason)

            # 2. Check if servos are actively moving toward goal positions
            any_torque_on = any(s.torque_enabled for s in states.values())
            is_moving = any(s.torque_enabled and abs(s.present_position - s.goal_position) > 0.5 for s in states.values())

            # Evaluate movement progress (position changes) to differentiate progress from stall
            positions_progressing = False
            if is_moving:
                if self._prev_positions is not None:
                    for sid, s in states.items():
                        if sid in self._prev_positions and abs(s.present_position - self._prev_positions[sid]) > 0.02:
                            positions_progressing = True
                            break
                else:
                    positions_progressing = True

            self._prev_positions = {sid: s.present_position for sid, s in states.items()}

            if is_moving:
                if positions_progressing:
                    self._last_progress_time = now

                # While movement is actively progressing towards goal, maintain command freshness
                # so long finite movements finish cleanly without false faults.
                if self.active_control and (now - self._last_progress_time) < self.config.watchdog_timeout:
                    self.last_command_timestamp = now
            else:
                # Servos are settled; end active control unless in an explicit control session
                if not self.control_session_explicit:
                    self.active_control = False

            # 3. Watchdog Timeout Check (only during active control session with torque active)
            if (self.watchdog_enabled and 
                self.active_control and 
                self.last_command_timestamp is not None and 
                any_torque_on):
                
                elapsed = now - self.last_command_timestamp
                if elapsed > self.config.watchdog_timeout:
                    status["safe"] = False
                    self.watchdog_tripped = True
                    status["watchdog_tripped"] = True
                    reason = f"WATCHDOG TIMEOUT: No command or heartbeat received for {elapsed:.1f}s (timeout: {self.config.watchdog_timeout}s)"
                    self._trigger_safety_fault(reason)
                    status["fault_latched"] = self.fault_latched
                    status["fault_reason"] = self.fault_reason

        except Exception as e:
            logger.error(f"Error during safety check: {e}")

        return status

    def _trigger_safety_fault(self, reason: str) -> None:
        """Internal helper to latch fault and disable torque."""
        if not self.fault_latched:
            self.fault_latched = True
            self.fault_reason = reason
            logger.critical(f"SAFETY SYSTEM FAULT TRIGGERED: {reason}")
            try:
                self.arm.emergency_stop()
            except Exception as e:
                logger.error(f"Failed to execute emergency stop during fault: {e}")

    def trigger_estop(self) -> None:
        """Trigger immediate emergency stop."""
        self.fault_latched = True
        self.fault_reason = "EMERGENCY STOP ACTIVATED BY USER/SYSTEM"
        self.arm.emergency_stop()
        logger.warning("Emergency Stop explicitly triggered via SafetyMonitor.")

    def clear_fault(self) -> bool:
        """Clear latched fault if safe conditions are met."""
        # Inspect state before clearing
        try:
            states = self.arm.get_state()
            overtemp = [f"ID {sid} ({s.temperature}°C)" for sid, s in states.items() if s.temperature > self.config.maximum_temperature]
            if overtemp:
                raise SafetyFaultError(f"Cannot clear fault: Over-temperature still active on {', '.join(overtemp)}")
        except Exception as e:
            if isinstance(e, SafetyFaultError):
                raise
            logger.warning(f"Could not check servo states during clear_fault: {e}")

        self.arm.clear_fault()
        self.fault_latched = False
        self.fault_reason = None
        self.watchdog_tripped = False
        self.active_control = False
        self.control_session_explicit = False
        self._motion_active = False
        self._prev_positions = None
        self._last_progress_time = self._now()
        self.last_command_timestamp = None
        logger.info("SafetyMonitor fault cleared successfully.")
        return True
