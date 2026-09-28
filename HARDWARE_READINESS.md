# Hardware Readiness & Physical Bring-Up Checklist

This document details the exact safety protocols, hardware setup instructions, bring-up sequence, and emergency procedures required BEFORE connecting real ROBOTIS Dynamixel AX-12A hardware.

---

## 1. Simulation vs Physical Test Status

### ✅ Tested & Verified in Simulation Mode (25 Unit Tests)
- [x] Coordinated 5-joint movement (`GroupSyncWrite` byte packing and degree/raw conversion).
- [x] Software joint limit validation (per-joint min/max enforcement).
- [x] Torque enable/disable controls.
- [x] Watchdog timeout safety (5.0s countdown trigger and torque cutoff).
- [x] Over-temperature fault protection (70.0°C cutoff & clear_fault prevention).
- [x] Emergency Stop broadcast transmission and movement blocking.
- [x] Fault state latching and safe manual recovery flow.
- [x] REST API endpoints (`/api/state`, `/api/move`, `/api/torque/*`, `/api/estop`, `/api/clear_fault`, `/api/move_xyz`).
- [x] Forward and Inverse Kinematics (FK/IK) calculations and XYZ Cartesian positioning.
- [x] Real-time WebSocket telemetry broadcasting.
- [x] Thread-safe serial bus locking (`threading.RLock()`).

### ⚠️ NOT YET TESTED PHYSICALLY (Pending Hardware Connection)
- [ ] Real U2D2 USB-to-RS485/TTL serial communication on Linux `/dev/ttyUSB0`.
- [ ] Physical Dynamixel AX-12A EEPROM ID writing & baud rate setting.
- [ ] Actual half-duplex bus timing, packet checksum verification, and voltage drop under mechanical load.
- [ ] Physical mechanical joint alignment and cable tension.

---

## 2. Hardware Environment Setup

### Raspberry Pi 4B Setup
- Operating System: Raspberry Pi OS (64-bit Debian 12 / Bookworm).
- Power Supply: Official Raspberry Pi 15W USB-C Power Supply (5.1V / 3.0A).
- User Permissions: User must belong to `dialout` group to access `/dev/ttyUSB0` without `sudo`.
  ```bash
  sudo usermod -aG dialout $USER
  ```

### ROBOTIS U2D2 & Power Wiring
- Connect U2D2 to Raspberry Pi USB 3.0 port.
- Power Supply for AX-12A Servos: External 11.1V – 12.0V DC regulated power supply (capable of supplying at least 5.0A peak stall current for 5 servos).
- **CRITICAL**: Connect ground (GND) of external 12V power supply to U2D2 GND to establish common ground potential!

---

## 3. Mandatory Physical Bring-Up Sequence

Do NOT connect all 5 servos and attempt full-range movement immediately. Follow this 6-phase sequence strictly:

### Phase 1: Single Servo Bring-Up
1. Connect **only Servo 1** to U2D2 via 3-pin TTL ribbon cable.
2. Run single servo ping test:
   ```bash
   python -c "from backend.config import ArmConfig; from backend.arm import DynamixelArm; cfg=ArmConfig(simulation=False); arm=DynamixelArm(cfg); print('Ping ID 1:', arm.ping_servo(1))"
   ```
3. Test LED toggle to confirm bi-directional serial transmission:
   ```bash
   python -c "... arm.packet_handler.write1ByteTxRx(arm.port_handler, 1, 25, 1)" # LED ON
   ```
4. Perform present position reading to verify feedback encoder response.

### Phase 2: Configure Servo IDs (1 to 5)
1. Connect servos one by one to configure EEPROM ID register (Address 3):
   - Set Base servo to **ID 1** (Baud rate: 1 Mbps / value 1).
   - Set Shoulder servo to **ID 2**.
   - Set Elbow servo to **ID 3**.
   - Set Wrist Pitch servo to **ID 4**.
   - Set Wrist Roll servo to **ID 5**.
2. Verify each servo responds independently without ID conflicts on the bus.

### Phase 3: Chain Servos & Telemetry Check
1. Daisy-chain all 5 servos in series using 3-pin TTL cables.
2. Start server in hardware mode (`config.yaml`: `simulation: false`).
3. Open Web Dashboard (`http://<PI_IP>:8000`) and verify all 5 servos display `ONLINE` status.
4. Inspect baseline telemetry:
   - Voltage: Expected **11.1V – 12.0V**.
   - Temperature: Expected **25°C – 40°C** ambient.
   - Load: **0.0%**.

### Phase 4: Torque ON & Conservative Homing
1. Click **TORQUE ON** button.
2. Observe arm holding current posture without sudden jerks.
3. If any servo emits high-pitched buzzing or rapidly rising load (>70%), IMMEDIATELY PRESS E-STOP.

### Phase 5: Small Position Moves (±20°)
1. Set speed slider to **Slow (50 raw)**.
2. Move joint sliders by small increments (±10° to ±20° around center 150°).
3. Click **MOVE JOINTS SYNCHRONIZED** and verify smooth, controlled physical joint motion.

### Phase 6: Gradual Envelope & Speed Expansion
1. Gradually expand joint angle range towards configured software limits.
2. Test pose saving and execution at medium speeds (speed = 200).

---

## 4. Expected Telemetry Baseline & Failure Criteria

### Nominal Operating Values
- **Bus Voltage**: `11.1V` to `12.2V` (If voltage drops below `9.5V`, power supply is under-dimensioned).
- **Idle Temperature**: `28.0°C` to `42.0°C`.
- **Moving Temperature**: `35.0°C` to `55.0°C`.
- **Comm Status**: `OK`.

### 🚨 What Constitutes a Hardware Failure?
1. **Unresponsive Servo**: Communication error on bus (`COMM_ERROR` status).
2. **Over-Temperature**: Temperature exceeding `65.0°C` (Warning) or `70.0°C` (Critical Cutoff).
3. **Stall Condition**: Servo load exceeding `85%` for more than 2 seconds without position progress.
4. **Undervoltage**: Voltage dropping below `9.5V` under motion.
5. **Watchdog Tripped**: No command sent for 5.0s while torque is active.

---

## 5. Emergency Shutdown Procedure

If physical arm behaves erratically, mechanical collision occurs, or power supply short-circuits:

1. **Software Emergency Stop**: Click the large red **EMERGENCY STOP** button on the Web Dashboard (or issue `POST /api/estop`).
2. **Hardware Power Disconnect**: Switch OFF or unplug the main 12V DC external power supply supplying the arm servos.
3. **Serial Disconnect**: Disconnect U2D2 USB cable from Raspberry Pi.
