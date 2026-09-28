/**
 * Main Frontend Application Controller for Dynamixel Robotic Arm Dashboard.
 */

const ArmControl = {
  jointLimits: {
    1: { min: 0, max: 300, label: "J1 Base" },
    2: { min: 15, max: 285, label: "J2 Shoulder" },
    3: { min: 15, max: 285, label: "J3 Elbow" },
    4: { min: 10, max: 290, label: "J4 Wrist Pitch" },
    5: { min: 0, max: 300, label: "J5 Wrist Roll" }
  },

  currentGoalAngles: { 1: 150, 2: 150, 3: 150, 4: 150, 5: 150 },
  telemetryClient: null,
  isUserInteracting: false,

  init() {
    this.renderJointSliders();
    this.initTelemetry();
    this.refreshPoses();
    this.fetchInitialState();

    // Event listener to unlock slider update after manual user dragging
    document.addEventListener('mouseup', () => {
      this.isUserInteracting = false;
    });
  },

  async fetchInitialState() {
    try {
      const state = await ArmAPI.getState();
      if (state) {
        this.onTelemetryData(state);
      }
    } catch (e) {
      console.warn('Initial state fetch failed:', e);
    }
  },

  renderJointSliders() {
    const container = document.getElementById('joint-sliders');
    if (!container) return;

    container.innerHTML = '';
    for (let id = 1; id <= 5; id++) {
      const limit = this.jointLimits[id];
      const initialVal = 150;

      const row = document.createElement('div');
      row.className = 'joint-row';
      row.innerHTML = `
        <div class="joint-row-header">
          <span class="joint-title">${limit.label} (Servo ${id})</span>
          <div class="joint-values">
            Pres: <span id="pos-val-${id}">150.0°</span> | 
            Goal: <span id="goal-val-${id}">150.0°</span>
          </div>
        </div>
        <div class="joint-slider-control">
          <span class="limit-label">${limit.min}°</span>
          <input type="range" id="slider-j${id}" 
                 min="${limit.min}" max="${limit.max}" step="0.5" value="${initialVal}"
                 onmousedown="ArmControl.isUserInteracting = true"
                 oninput="ArmControl.onSliderInput(${id}, this.value)">
          <span class="limit-label">${limit.max}°</span>
          <input type="number" id="num-j${id}" class="joint-num-input"
                 min="${limit.min}" max="${limit.max}" step="0.5" value="${initialVal}"
                 onchange="ArmControl.onNumberInput(${id}, this.value)">
        </div>
      `;
      container.appendChild(row);
    }
  },

  onSliderInput(jointId, value) {
    const val = parseFloat(value);
    this.currentGoalAngles[jointId] = val;
    const numElem = document.getElementById(`num-j${jointId}`);
    if (numElem) numElem.value = val.toFixed(1);
    const goalElem = document.getElementById(`goal-val-${jointId}`);
    if (goalElem) goalElem.innerText = `${val.toFixed(1)}°`;
  },

  onNumberInput(jointId, value) {
    const limit = this.jointLimits[jointId];
    let val = parseFloat(value);
    if (isNaN(val)) val = limit.min;
    val = Math.max(limit.min, Math.min(limit.max, val));

    this.currentGoalAngles[jointId] = val;
    const sliderElem = document.getElementById(`slider-j${jointId}`);
    if (sliderElem) sliderElem.value = val;
    const numElem = document.getElementById(`num-j${jointId}`);
    if (numElem) numElem.value = val.toFixed(1);
    const goalElem = document.getElementById(`goal-val-${jointId}`);
    if (goalElem) goalElem.innerText = `${val.toFixed(1)}°`;
  },

  updateSpeedLabel(val) {
    const display = document.getElementById('speed-val-display');
    if (display) display.innerText = `${val} (Raw)`;
  },

  setSpeedPreset(speed) {
    const slider = document.getElementById('speed-slider');
    if (slider) {
      slider.value = speed;
      this.updateSpeedLabel(speed);
    }
  },

  async applyMove() {
    const speed = document.getElementById('speed-slider').value;
    try {
      const res = await ArmAPI.moveJoints(this.currentGoalAngles, speed);
      this.showToast(res.message, 'success');
    } catch (err) {
      this.showToast(`Move failed: ${err.message}`, 'error');
    }
  },

  async presetHome() {
    const homeAngles = { 1: 150, 2: 150, 3: 150, 4: 150, 5: 150 };
    for (let i = 1; i <= 5; i++) {
      this.onSliderInput(i, homeAngles[i]);
    }
    await this.applyMove();
  },

  async presetSleep() {
    const sleepAngles = { 1: 150, 2: 45, 3: 250, 4: 45, 5: 150 };
    for (let i = 1; i <= 5; i++) {
      this.onSliderInput(i, sleepAngles[i]);
    }
    await this.applyMove();
  },

  async setTorque(enable) {
    try {
      const res = await ArmAPI.setTorque(enable);
      this.showToast(res.message, enable ? 'success' : 'info');
    } catch (err) {
      this.showToast(`Torque error: ${err.message}`, 'error');
    }
  },

  async triggerEstop() {
    try {
      const res = await ArmAPI.triggerEstop();
      this.showToast('EMERGENCY STOP ACTIVATED!', 'error', 5000);
    } catch (err) {
      this.showToast(`E-STOP error: ${err.message}`, 'error');
    }
  },

  async clearFault() {
    try {
      const res = await ArmAPI.clearFault();
      this.showToast(res.message, 'success');
    } catch (err) {
      this.showToast(`Clear fault error: ${err.message}`, 'error');
    }
  },

  async saveCurrentPose() {
    const input = document.getElementById('pose-name-input');
    const name = input ? input.value.trim() : '';
    if (!name) {
      this.showToast('Please enter a pose name', 'warning');
      return;
    }
    const speed = document.getElementById('speed-slider').value;
    try {
      const res = await ArmAPI.saveCurrentPose(name, speed);
      this.showToast(res.message, 'success');
      if (input) input.value = '';
      this.refreshPoses();
    } catch (err) {
      this.showToast(`Save pose error: ${err.message}`, 'error');
    }
  },

  async refreshPoses() {
    try {
      const res = await ArmAPI.listPoses();
      this.renderPoseList(res.data || []);
    } catch (err) {
      console.warn('Failed to refresh poses:', err);
    }
  },

  renderPoseList(poses) {
    const container = document.getElementById('pose-list-container');
    if (!container) return;

    if (poses.length === 0) {
      container.innerHTML = '<p class="empty-list-msg">No saved poses found.</p>';
      return;
    }

    container.innerHTML = '';
    poses.forEach(pose => {
      const item = document.createElement('div');
      item.className = 'pose-item';
      item.innerHTML = `
        <span class="pose-item-name"><i class="fa-solid fa-location-dot"></i> ${pose.name}</span>
        <div class="pose-item-actions">
          <button class="btn btn-xs btn-primary" onclick="ArmControl.executePose('${pose.name}')">
            <i class="fa-solid fa-play"></i> Move
          </button>
          <button class="btn btn-xs btn-outline" onclick="ArmControl.deletePose('${pose.name}')">
            <i class="fa-solid fa-trash"></i>
          </button>
        </div>
      `;
      container.appendChild(item);
    });
  },

  async executePose(name) {
    try {
      const res = await ArmAPI.executePose(name);
      this.showToast(`Executing pose '${name}'`, 'info');
    } catch (err) {
      this.showToast(`Execute pose error: ${err.message}`, 'error');
    }
  },

  async deletePose(name) {
    try {
      await ArmAPI.deletePose(name);
      this.showToast(`Pose '${name}' deleted`, 'info');
      this.refreshPoses();
    } catch (err) {
      this.showToast(`Delete pose error: ${err.message}`, 'error');
    }
  },

  initTelemetry() {
    this.telemetryClient = new TelemetryClient(
      (state) => this.onTelemetryData(state),
      (connected) => this.onConnectionStatus(connected)
    );
    this.telemetryClient.connect();
  },

  onConnectionStatus(connected) {
    const dot = document.getElementById('dot-connection');
    const val = document.getElementById('val-connection');
    if (dot && val) {
      if (connected) {
        dot.className = 'status-dot online';
        val.innerText = 'ONLINE';
      } else {
        dot.className = 'status-dot offline';
        val.innerText = 'OFFLINE';
      }
    }
  },

  onTelemetryData(state) {
    // 1. Update Mode badge
    const valMode = document.getElementById('val-mode');
    if (valMode) valMode.innerText = state.simulation ? 'SIMULATION' : 'HARDWARE';

    // 2. Update System status badge
    const badgeSys = document.getElementById('badge-system');
    const valSys = document.getElementById('val-system');
    if (badgeSys && valSys) {
      valSys.innerText = state.system_status;
      badgeSys.className = `status-badge ${state.system_status.toLowerCase()}`;
    }

    // 3. Update Torque status badge
    const valTorque = document.getElementById('val-torque');
    if (valTorque) valTorque.innerText = state.torque_enabled ? 'ON' : 'OFF';

    // 4. Update Diagnostics
    const watchdogVal = document.getElementById('diag-watchdog');
    if (watchdogVal) {
      watchdogVal.innerText = state.watchdog_tripped ? 'TRIPPED (TIMEOUT)' : 'ACTIVE (5.0s)';
      watchdogVal.className = state.watchdog_tripped ? 'diag-val fault-text' : 'diag-val';
    }

    const lastCmdVal = document.getElementById('diag-last-cmd');
    if (lastCmdVal && state.last_command_timestamp > 0) {
      const elapsed = (Date.now() / 1000 - state.last_command_timestamp).toFixed(1);
      lastCmdVal.innerText = `${elapsed}s ago`;
    }

    const faultVal = document.getElementById('diag-fault-reason');
    if (faultVal) {
      faultVal.innerText = state.fault_reason || 'None';
    }

    // 5. Update Telemetry Table & Joint Present Positions
    const tbody = document.getElementById('telemetry-table-body');
    if (!tbody || !state.servos) return;

    let rowsHtml = '';
    for (let sid = 1; sid <= 5; sid++) {
      let s = null;
      if (state.servos) {
        s = state.servos[sid] || state.servos[String(sid)];
        if (!s && Array.isArray(state.servos)) {
          s = state.servos.find(item => item && item.id === sid);
        }
      }
      if (!s) continue;

      // Update position labels if not actively dragging
      const posLabel = document.getElementById(`pos-val-${sid}`);
      if (posLabel) posLabel.innerText = `${s.present_position.toFixed(1)}°`;

      if (!this.isUserInteracting) {
        const goalLabel = document.getElementById(`goal-val-${sid}`);
        if (goalLabel) goalLabel.innerText = `${s.goal_position.toFixed(1)}°`;
      }

      // Temp badge class
      let tempClass = 'temp-normal';
      if (s.temperature >= 65) tempClass = 'temp-danger';
      else if (s.temperature >= 55) tempClass = 'temp-warn';

      rowsHtml += `
        <tr>
          <td><strong>J${sid} / Servo ${sid}</strong></td>
          <td>${s.present_position.toFixed(1)}°</td>
          <td>${s.goal_position.toFixed(1)}°</td>
          <td>${s.speed}</td>
          <td>${s.load.toFixed(1)}%</td>
          <td>${s.voltage.toFixed(1)} V</td>
          <td><span class="temp-badge ${tempClass}">${s.temperature.toFixed(1)} °C</span></td>
          <td>${s.torque_enabled ? '<span style="color:#10b981">ON</span>' : '<span style="color:#ef4444">OFF</span>'}</td>
          <td>${s.comm_status}</td>
        </tr>
      `;
    }
    tbody.innerHTML = rowsHtml;
  },

  showToast(message, type = 'info', duration = 3000) {
    const container = document.getElementById('toast-container');
    if (!container) return;

    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    let icon = 'fa-info-circle';
    if (type === 'success') icon = 'fa-circle-check';
    if (type === 'warning') icon = 'fa-triangle-exclamation';
    if (type === 'error') icon = 'fa-circle-xmark';

    toast.innerHTML = `<i class="fa-solid ${icon}"></i> <span>${message}</span>`;
    container.appendChild(toast);

    setTimeout(() => {
      toast.style.opacity = '0';
      toast.style.transition = 'opacity 0.3s ease';
      setTimeout(() => toast.remove(), 300);
    }, duration);
  }
};

document.addEventListener('DOMContentLoaded', () => ArmControl.init());
