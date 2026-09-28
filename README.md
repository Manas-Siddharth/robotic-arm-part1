# 5-DOF Dynamixel AX-12A Robotic Arm Control System

A complete, high-performance software stack for a 5-DOF ROBOTIS Dynamixel AX-12A robotic arm. Includes simulation mode, real U2D2 hardware driver, safety subsystem, FastAPI REST API, live WebSocket telemetry stream, forward/inverse kinematics (FK/IK), teach pose system, and a glassmorphism web control dashboard.

---

## 🏗 System Architecture

```text
Browser (Single Page Web Dashboard)
    │
    │  HTTP REST API & WebSocket Telemetry Stream
    ▼
FastAPI Backend (Uvicorn Server)
    │
    ├───────────────► Kinematics Engine (backend/kinematics.py: FK/IK & XYZ Control)
    │
    ▼
Arm Abstraction Layer (backend/arm.py)
    │
    ├───────────────► Simulation Engine (simulation/simulated_arm.py)
    │
    └───────────────► Real Dynamixel Driver (ROBOTIS U2D2 / SDK)
                         │
                         ▼
                     ROBOTIS U2D2 (/dev/ttyUSB0)
                         │
                         ▼
                     5 × Dynamixel AX-12A Servos
```

> **CRITICAL ARCHITECTURAL RULE:**
> ONLY `backend/arm.py` communicates directly with Dynamixel hardware / SDK. The frontend, API routes, safety system, telemetry system, kinematics, and pose system MUST NOT directly access the SDK. Everything communicates strictly through the `BaseArm` abstraction interface.

---

## 📋 Hardware Specifications

- **Microcontroller/Host**: Raspberry Pi 4B (8GB RAM recommended)
- **USB Serial Adapter**: ROBOTIS U2D2
- **Actuators**: 5 × ROBOTIS Dynamixel AX-12A
- **Protocol**: Dynamixel Protocol 1.0 (Baud Rate: 1,000,000 bps / 1 Mbps)
- **Bus**: TTL half-duplex serial bus with `threading.RLock()` thread safety
- **Servo IDs**: 1 (Base), 2 (Shoulder), 3 (Elbow), 4 (Wrist Pitch), 5 (Wrist Roll)
- **Position Range**: 0 – 1023 (corresponds to approx. 0.0° – 300.0°, ~0.293°/unit)
- **Operating Voltage**: 9.0 – 12.0 V (Recommended: 11.1 – 12.0 V)
- **Linux Serial Device**: `/dev/ttyUSB0`

---

## 📁 Project Structure

```text
robotic arm/
├── backend/
│   ├── __init__.py
│   ├── main.py            # FastAPI REST & WebSocket server
│   ├── arm.py             # BaseArm interface & DynamixelArm implementation (SDK isolated)
│   ├── safety.py          # Watchdog, over-temp protection, E-STOP & fault latch
│   ├── telemetry.py       # Async WebSocket telemetry broadcaster & background safety loop
│   ├── kinematics.py      # Forward/Inverse Kinematics solver & XYZ controller
│   ├── pose_manager.py    # Teach, save, load, and replay poses
│   ├── config.py          # YAML & Pydantic configuration loader
│   ├── exceptions.py      # Custom exception hierarchy
│   └── models.py          # Pydantic request/response schemas
├── simulation/
│   ├── __init__.py
│   └── simulated_arm.py   # Realistic simulated arm physics engine
├── frontend/
│   ├── index.html         # Single Page Dashboard
│   ├── css/
│   │   └── style.css      # Dark mode glassmorphism UI stylesheet
│   └── js/
│       ├── app.js         # Main UI controller & DOM event handling
│       ├── arm.js         # REST API client
│       └── telemetry.js   # WebSocket client with auto-reconnect
├── tests/
│   ├── __init__.py
│   ├── test_simulation.py # Simulation unit tests
│   ├── test_arm.py        # Arm layer abstraction tests
│   ├── test_safety.py     # Watchdog, temp & fault latch tests
│   ├── test_api.py        # REST API integration tests
│   ├── test_kinematics.py # FK/IK kinematics unit tests
│   └── test_telemetry.py # Telemetry manager unit tests
├── data/
│   └── poses/             # JSON stored poses
├── config.yaml            # System configuration parameters
├── requirements.txt       # Python dependencies
├── run.py                 # Application launcher
├── pytest.ini             # Pytest configuration
├── HARDWARE_READINESS.md  # Physical hardware bring-up safety guide
└── README.md              # Project documentation
```

---

## 💻 Windows Development & Simulation Mode

Simulation mode works seamlessly out of the box on Windows without needing U2D2 hardware or serial drivers.

### 1. Setup Environment
```powershell
# Navigate to workspace
cd "G:\Projects\robotic arm"

# Create and activate virtual environment
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt
```

### 2. Verify Config (`config.yaml`)
Ensure `simulation: true` in `config.yaml`:
```yaml
simulation: true
serial_port: "/dev/ttyUSB0"
baud_rate: 1000000
protocol: 1.0
servo_ids: [1, 2, 3, 4, 5]
```

### 3. Run Server
```powershell
python run.py
```

Open your browser and navigate to:
`http://localhost:8000`

---

## 🍓 Raspberry Pi OS / Linux Hardware Mode

### 1. System Dependencies & Serial Permissions
Connect the ROBOTIS U2D2 to a USB port on the Raspberry Pi.

```bash
# Update Pi OS
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-venv git -y

# Grant serial port permissions (dialout group)
sudo usermod -aG dialout $USER

# Log out and log back in, or run:
newgrp dialout
```

### 2. Install Python Environment
```bash
cd ~
git clone <your-repository-url> "robotic arm"
cd "robotic arm"

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 3. Configure Hardware Mode
Edit `config.yaml` to set `simulation: false`:
```yaml
simulation: false
serial_port: "/dev/ttyUSB0"
baud_rate: 1000000
protocol: 1.0
servo_ids: [1, 2, 3, 4, 5]
```

### 4. Run Application & Access LAN Dashboard
```bash
python run.py
```
From any phone, tablet, or PC on the same local network, open:
`http://<RASPBERRY_PI_IP>:8000`

---

## 🛡 Real Hardware Bring-Up Safety Protocol

Before running physical motion on the 5-DOF arm, refer to [HARDWARE_READINESS.md](file:///G:/Projects/robotic%20arm/HARDWARE_READINESS.md) for the complete 6-phase bring-up sequence.

---

## 🧪 Automated Testing Status

Run the test suite:
```bash
python -m pytest -v
```

Output:
```text
======================= 25 passed in 0.80s ========================
```

- **Automated Test Results**: **25 automated tests pass cleanly**.

---

## ⚠️ Real Hardware Validation Status

> **Status**: Real hardware validation pending U2D2 + AX-12A physical connection.
