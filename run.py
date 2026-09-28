"""Entry point script to run the 5-DOF Dynamixel Robotic Arm Control Server."""

import sys
import uvicorn
from backend.config import get_config

def main():
    config = get_config()
    print("=" * 60)
    print(" 5-DOF Dynamixel AX-12A Robotic Arm Control Server")
    print(f" Mode: {'SIMULATION' if config.simulation else 'HARDWARE (U2D2)'}")
    print(f" Serial Port: {config.serial_port}")
    print(f" Baud Rate: {config.baud_rate}")
    print("=" * 60)

    uvicorn.run(
        "backend.main:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level=config.logging_level.lower()
    )

if __name__ == "__main__":
    main()
