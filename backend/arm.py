"""Hardware abstraction layer for 5-DOF Dynamixel AX-12A Robotic Arm."""

import logging
import platform
import threading
from abc import ABC, abstractmethod
from typing import Dict, List, Optional
from backend.config import ArmConfig, get_config
from backend.models import ServoState
from backend.exceptions import (
    HardwareConnectionError,
    ServoCommunicationError,
    JointLimitExceededError,
    InvalidCommandError,
    EmergencyStopError,
    SafetyFaultError
)

logger = logging.getLogger(__name__)

# AX-12A Dynamixel Register Addresses (Protocol 1.0)
ADDR_ID = 3
ADDR_BAUD_RATE = 4
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8
ADDR_TORQUE_ENABLE = 24
ADDR_LED = 25
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

BROADCAST_ID = 0xFE
PROTOCOL_VERSION = 1.0

class BaseArm(ABC):
    """Abstract Base Class for Robotic Arm implementations (Simulation and Hardware)."""

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @abstractmethod
    def enable_torque(self, enable: bool, joint_id: Optional[int] = None) -> bool:
        pass

    @abstractmethod
    def move_joints(self, angles: Dict[int, float], speed: Optional[int] = None) -> bool:
        pass

    @abstractmethod
    def get_state(self) -> Dict[int, ServoState]:
        pass

    @abstractmethod
    def emergency_stop(self) -> bool:
        pass

    @abstractmethod
    def clear_fault(self) -> bool:
        pass

    @abstractmethod
    def ping_servo(self, joint_id: int) -> bool:
        pass


class DynamixelArm(BaseArm):
    """Real Dynamixel AX-12A hardware implementation via ROBOTIS U2D2 / SDK."""

    def __init__(self, config: Optional[ArmConfig] = None):
        self.config = config or get_config()
        self.servo_ids: List[int] = list(self.config.servo_ids)
        self.connected: bool = False
        self.estop_active: bool = False
        self.fault_active: bool = False
        self.fault_reason: Optional[str] = None
        self.port_handler = None
        self.packet_handler = None
        self.sync_write = None
        self._bus_lock = threading.RLock()

        # Dynamixel SDK imports
        try:
            from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS
            self._COMM_SUCCESS = COMM_SUCCESS
            self._GroupSyncWrite = GroupSyncWrite
            self.port_handler = PortHandler(self.config.serial_port)
            self.packet_handler = PacketHandler(self.config.protocol)
        except ImportError as e:
            logger.error(f"Failed to import dynamixel_sdk: {e}")
            raise HardwareConnectionError("dynamixel-sdk package is not installed.")

    def connect(self) -> bool:
        """Open serial port, set baud rate, and ping configured servos."""
        if platform.system() == "Windows" and self.config.serial_port.startswith("/dev/"):
            raise HardwareConnectionError(
                f"Cannot open Linux device path '{self.config.serial_port}' on Windows. "
                "Set 'simulation: true' in config.yaml or specify a valid Windows COM port (e.g. COM3)."
            )

        with self._bus_lock:
            try:
                if not self.port_handler.openPort():
                    raise HardwareConnectionError(f"Failed to open serial port: {self.config.serial_port}")

                if not self.port_handler.setBaudRate(self.config.baud_rate):
                    self.port_handler.closePort()
                    raise HardwareConnectionError(f"Failed to set baud rate: {self.config.baud_rate}")

                # Initialize SyncWrite object for Goal Position + Speed (4 bytes at address 30)
                self.sync_write = self._GroupSyncWrite(
                    self.port_handler,
                    self.packet_handler,
                    ADDR_GOAL_POSITION,
                    4
                )

                # Ping and verify all configured servos
                unresponsive = []
                for sid in self.servo_ids:
                    if not self.ping_servo(sid):
                        unresponsive.append(sid)

                if unresponsive:
                    self._disconnect_internal()
                    raise ServoCommunicationError(f"Servos unresponsive on bus: {unresponsive}")

                self.connected = True
                self.estop_active = False
                self.fault_active = False
                self.fault_reason = None
                logger.info(f"DynamixelArm hardware successfully connected on {self.config.serial_port}")
                return True
            except Exception as e:
                self._disconnect_internal()
                if isinstance(e, (HardwareConnectionError, ServoCommunicationError)):
                    raise
                raise HardwareConnectionError(f"Hardware connection failed: {e}")

    def _disconnect_internal(self) -> None:
        if self.port_handler:
            try:
                self.port_handler.closePort()
            except Exception:
                pass
        self.connected = False

    def disconnect(self) -> None:
        """Safe shutdown and release serial port."""
        with self._bus_lock:
            if self.connected:
                try:
                    # Safe torque disable before closing
                    self.enable_torque(False)
                except Exception as e:
                    logger.warning(f"Error disabling torque on disconnect: {e}")
                finally:
                    self._disconnect_internal()
                    logger.info("DynamixelArm disconnected and port closed.")

    def is_connected(self) -> bool:
        return self.connected

    def ping_servo(self, joint_id: int) -> bool:
        """Ping a specific servo ID."""
        with self._bus_lock:
            if not self.port_handler:
                return False
            model_num, comm_result, dxl_error = self.packet_handler.ping(self.port_handler, joint_id)
            return comm_result == self._COMM_SUCCESS

    def degrees_to_raw(self, degrees: float) -> int:
        """Convert degrees (0-300) to raw units (0-1023)."""
        clamped = max(0.0, min(300.0, float(degrees)))
        return int(round((clamped / 300.0) * 1023.0))

    def raw_to_degrees(self, raw: int) -> float:
        """Convert raw units (0-1023) to degrees (0-300)."""
        clamped = max(0, min(1023, int(raw)))
        return round((clamped / 1023.0) * 300.0, 2)

    def validate_joint_limits(self, angles: Dict[int, float]) -> None:
        """Enforce joint software limits."""
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
        """Enable or disable torque on servos."""
        if not self.connected:
            raise HardwareConnectionError("Hardware arm is not connected.")

        if enable and (self.estop_active or self.fault_active):
            raise SafetyFaultError("Cannot enable torque while Emergency Stop or Safety Fault is active.")

        val = 1 if enable else 0
        with self._bus_lock:
            if joint_id is not None:
                if joint_id not in self.servo_ids:
                    raise InvalidCommandError(f"Invalid joint ID: {joint_id}")
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, joint_id, ADDR_TORQUE_ENABLE, val
                )
                if dxl_comm_result != self._COMM_SUCCESS:
                    raise ServoCommunicationError(f"Failed to set torque for joint {joint_id}")
            else:
                # Broadcast to all or loop through servo IDs
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, BROADCAST_ID, ADDR_TORQUE_ENABLE, val
                )
                if dxl_comm_result != self._COMM_SUCCESS:
                    # Fallback per servo if broadcast fails
                    for sid in self.servo_ids:
                        self.packet_handler.write1ByteTxRx(self.port_handler, sid, ADDR_TORQUE_ENABLE, val)

        logger.info(f"Dynamixel torque {'enabled' if enable else 'disabled'} (joint_id={joint_id})")
        return True

    def move_joints(self, angles: Dict[int, float], speed: Optional[int] = None) -> bool:
        """Perform Coordinated Joint Movement using SyncWrite."""
        if not self.connected:
            raise HardwareConnectionError("Hardware arm is not connected.")
        if self.estop_active:
            raise EmergencyStopError("Movement rejected: Emergency Stop is ACTIVE.")
        if self.fault_active:
            raise SafetyFaultError(f"Movement rejected: System is in FAULT state ({self.fault_reason}).")

        self.validate_joint_limits(angles)

        move_speed = speed if speed is not None else self.config.default_speed
        move_speed = max(1, min(1023, int(move_speed)))

        with self._bus_lock:
            self.sync_write.clearParam()

            for joint_id, target_deg in angles.items():
                raw_pos = self.degrees_to_raw(target_deg)

                # Build 4-byte payload: [Goal Position Low, Goal Position High, Speed Low, Speed High]
                param = [
                    raw_pos & 0xFF,
                    (raw_pos >> 8) & 0xFF,
                    move_speed & 0xFF,
                    (move_speed >> 8) & 0xFF
                ]

                add_result = self.sync_write.addParam(int(joint_id), param)
                if not add_result:
                    self.sync_write.clearParam()
                    raise ServoCommunicationError(f"Failed to add SyncWrite parameter for servo {joint_id}")

            comm_result = self.sync_write.txPacket()
            self.sync_write.clearParam()

            if comm_result != self._COMM_SUCCESS:
                raise ServoCommunicationError(
                    f"SyncWrite movement packet failed: {self.packet_handler.getTxRxResult(comm_result)}"
                )

        logger.debug(f"SyncWrite sent: {angles}, speed: {move_speed}")
        return True

    def get_state(self) -> Dict[int, ServoState]:
        """Read state from all physical AX-12A servos."""
        if not self.connected:
            raise HardwareConnectionError("Arm is not connected.")

        states = {}
        with self._bus_lock:
            for sid in self.servo_ids:
                # Read present position (2 bytes)
                raw_pos, res_pos, err_pos = self.packet_handler.read2ByteTxRx(self.port_handler, sid, ADDR_PRESENT_POSITION)
                # Read present speed (2 bytes)
                speed_raw, res_spd, err_spd = self.packet_handler.read2ByteTxRx(self.port_handler, sid, ADDR_PRESENT_SPEED)
                # Read present load (2 bytes)
                load_raw, res_load, err_load = self.packet_handler.read2ByteTxRx(self.port_handler, sid, ADDR_PRESENT_LOAD)
                # Read present voltage (1 byte)
                volts_raw, res_volt, err_volt = self.packet_handler.read1ByteTxRx(self.port_handler, sid, ADDR_PRESENT_VOLTAGE)
                # Read present temperature (1 byte)
                temp, res_temp, err_temp = self.packet_handler.read1ByteTxRx(self.port_handler, sid, ADDR_PRESENT_TEMPERATURE)
                # Read torque enable (1 byte)
                torque_en, res_torq, err_torq = self.packet_handler.read1ByteTxRx(self.port_handler, sid, ADDR_TORQUE_ENABLE)
                # Read goal position (2 bytes)
                goal_raw, res_goal, err_goal = self.packet_handler.read2ByteTxRx(self.port_handler, sid, ADDR_GOAL_POSITION)

                comm_ok = res_pos == self._COMM_SUCCESS
                comm_status = "OK" if comm_ok else "COMM_ERROR"

                deg_pos = self.raw_to_degrees(raw_pos if comm_ok else 512)
                deg_goal = self.raw_to_degrees(goal_raw if res_goal == self._COMM_SUCCESS else 512)

                # Speed decoding: Bit 10 (0x400) direction, bits 0-9 (0x3FF) magnitude
                speed_val = 0
                if res_spd == self._COMM_SUCCESS:
                    speed_mag = speed_raw & 0x3FF
                    speed_dir = -1 if (speed_raw & 0x400) else 1
                    speed_val = speed_mag * speed_dir

                # Load calculation: bit 10 is direction (0 = CCW, 1 = CW), 0-1023 magnitude
                load_pct = 0.0
                if res_load == self._COMM_SUCCESS:
                    load_mag = (load_raw & 0x3FF) / 1023.0 * 100.0
                    direction = -1.0 if (load_raw & 0x400) else 1.0
                    load_pct = round(load_mag * direction, 1)

                voltage = round(volts_raw / 10.0, 1) if res_volt == self._COMM_SUCCESS else 0.0

                states[sid] = ServoState(
                    id=sid,
                    goal_position=deg_goal,
                    present_position=deg_pos,
                    raw_position=raw_pos if comm_ok else 0,
                    speed=speed_val,
                    load=load_pct,
                    voltage=voltage,
                    temperature=float(temp) if res_temp == self._COMM_SUCCESS else 0.0,
                    torque_enabled=bool(torque_en == 1),
                    comm_status=comm_status,
                    error_flags=err_pos | err_temp
                )

        return states

    def emergency_stop(self) -> bool:
        """Send emergency stop packet to broadcast address 0xFE."""
        self.estop_active = True
        with self._bus_lock:
            if self.connected and self.packet_handler and self.port_handler:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0
                )
        logger.warning("DynamixelArm: EMERGENCY STOP TRANSMITTED TO BUS!")
        return True

    def clear_fault(self) -> bool:
        """Clear emergency stop and fault latch."""
        self.estop_active = False
        self.fault_active = False
        self.fault_reason = None
        logger.info("DynamixelArm: Fault state cleared.")
        return True


def create_arm(config: Optional[ArmConfig] = None) -> BaseArm:
    """Factory method to instantiate either Simulation or Hardware Arm."""
    cfg = config or get_config()
    if cfg.simulation:
        from simulation.simulated_arm import SimulatedArm
        arm = SimulatedArm(cfg)
        arm.connect()
        return arm
    else:
        arm = DynamixelArm(cfg)
        arm.connect()
        return arm
