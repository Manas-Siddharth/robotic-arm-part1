"""Custom exception classes for 5-DOF Dynamixel Robotic Arm Control System."""

class RoboticArmException(Exception):
    """Base exception for all robotic arm errors."""
    pass

class HardwareConnectionError(RoboticArmException):
    """Raised when serial port connection or U2D2 initialization fails."""
    pass

class ServoCommunicationError(RoboticArmException):
    """Raised when communication with a Dynamixel servo fails or pings fail."""
    pass

class JointLimitExceededError(RoboticArmException):
    """Raised when requested joint angle violates configured joint software limits."""
    pass

class SafetyFaultError(RoboticArmException):
    """Raised when safety monitoring detects a fault condition (e.g. over-temp, watchdog)."""
    pass

class EmergencyStopError(RoboticArmException):
    """Raised when an operation is attempted while Emergency Stop is active."""
    pass

class InvalidCommandError(RoboticArmException):
    """Raised when invalid parameters (e.g. unknown joint ID, out of bound speed) are provided."""
    pass
