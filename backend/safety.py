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

    def __init__(self, arm: BaseArm, config: Optional[ArmConfig] = None):
        self.arm = arm
        self.config = config or get_config()
        self.last_command_timestamp: Optional[float] = None
        self.active_control: bool = False
        self.fault_latched: bool = False
        self.fault_reason: Optional[str] = None
        self.watchdog_tripped: bool = False
        self.watchdog_enabled: bool = True

    def record_command(self) -> None:
        """Update last command timestamp to reset the watchdog timer during active control."""
        if not self.fault_latched:
            self.last_command_timestamp = time.time()
            self.active_control = True
            if self.watchdog_tripped:
                self.watchdog_tripped = False

    def enable_watchdog(self, enable: bool = True) -> None:
        """Enable or disable watchdog timer checking."""
        self.watchdog_enabled = enable

    def check_safety(self) -> Dict[str, Any]:
        """Perform periodic health check on the arm.

        Checks:
        1. Over-temperature threshold on all servos.
        2. Watchdog timeout duration since last command during active control.

        Returns status dict.
        """
        now = time.time()
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

            # 2. Watchdog Timeout Check (only during active control with torque active)
            any_torque_on = any(s.torque_enabled for s in states.values())

            if (self.watchdog_enabled and 
                self.active_control and 
                self.last_command_timestamp is not None and 
                any_torque_on):
                
                elapsed = now - self.last_command_timestamp
                if elapsed > self.config.watchdog_timeout:
                    status["safe"] = False
                    self.watchdog_tripped = True
                    reason = f"WATCHDOG TIMEOUT: No command received for {elapsed:.1f}s (timeout: {self.config.watchdog_timeout}s)"
                    self._trigger_safety_fault(reason)

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
        self.last_command_timestamp = None
        logger.info("SafetyMonitor fault cleared successfully.")
        return True
