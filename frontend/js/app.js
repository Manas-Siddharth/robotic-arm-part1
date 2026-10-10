/**
 * Main Frontend Application Controller for Dynamixel AX-12A Robotic Arm Workstation.
 */

const ArmControl = {
  jointLimits: {
    1: { min: 0, max: 300, label: "J1 Base Yaw" },
    2: { min: 15, max: 285, label: "J2 Shoulder Pitch" },
    3: { min: 15, max: 285, label: "J3 Elbow Pitch" },
    4: { min: 10, max: 290, label: "J4 Wrist Pitch" },
    5: { min: 0, max: 300, label: "J5 Wrist Roll" }
  },

  currentGoalAngles: { 1: 150, 2: 150, 3: 150, 4: 150, 5: 150 },
  presentAngles: { 1: 150, 2: 150, 3: 150, 4: 150, 5: 150 },
  
  telemetryClient: null,
  arm3D: null,
  telemetryChart: null,
  chartPaused: false,

  isUserInteracting: false,
  lastTelemetryTimestamp: 0,

  savedPoses: [],
  currentSequenceSteps: [],
  eventLogs: [],

  init() {
    this.renderJointSliders();
    this.renderJointSummary();
    this.init3D();
    this.initTelemetry();
    this.initChart();
    
    this.refreshPoses();
    this.refreshSequences();
    this.initTrajectory();
    // Support opening specific tab via URL hash (e.g. #tab-trajectory)
    const hashTab = window.location.hash ? window.location.hash.replace('#', '') : null;
    if (hashTab && document.getElementById(hashTab)) {
      this.switchTab(hashTab);
    }

    document.addEventListener('mouseup', () => {
      this.isUserInteracting = false;
    });

    this.logEvent('System dashboard initialized in simulation mode.', 'info');
  },

  switchTab(tabId) {
    document.querySelectorAll('.nav-tab').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.tab === tabId);
    });
    document.querySelectorAll('.tab-page').forEach(page => {
      page.classList.toggle('active', page.id === tabId);
    });

    if (tabId === 'tab-overview' && this.arm3D) {
      setTimeout(() => this.arm3D.onWindowResize(), 50);
    }
    if (tabId === 'tab-trajectory' && this.trajectoryPreviewChart) {
      setTimeout(() => this.trajectoryPreviewChart.resize(), 50);
    }
  },

  init3D() {
    try {
      this.arm3D = new Arm3DView('arm-3d-canvas-container');
    } catch (e) {
      console.warn('3D Arm init error:', e);
    }
  },

  toggleTargetGhost() {
    if (this.arm3D) {
      this.arm3D.showTarget = !this.arm3D.showTarget;
      this.arm3D.updateAngles(this.presentAngles, this.currentGoalAngles);
      const btn = document.getElementById('btn-toggle-ghost');
      if (btn) btn.classList.toggle('btn-primary', this.arm3D.showTarget);
    }
  },

  async fetchInitialState() {
    try {
      const state = await ArmAPI.getState();
      if (state) {
        this.onTelemetryData(state);
      }
      this.updateBackendBadge(true);
    } catch (e) {
      console.warn('Initial state fetch failed:', e);
      this.updateBackendBadge(false);
    }
  },

  updateBackendBadge(online) {
    const dot = document.getElementById('dot-backend');
    const val = document.getElementById('val-backend');
    if (dot && val) {
      dot.className = online ? 'status-dot online' : 'status-dot offline';
      val.innerText = online ? 'ONLINE' : 'OFFLINE';
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
          <div class="joint-header-actions">
            <button class="btn btn-xs btn-outline" onclick="ArmControl.presetJointHome(${id})">
              <i class="fa-solid fa-house"></i> Home (150°)
            </button>
            <div class="joint-values">
              Pres: <span id="pos-val-${id}">150.0°</span> | 
              Goal: <span id="goal-val-${id}">150.0°</span>
            </div>
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

  renderJointSummary() {
    const container = document.getElementById('joint-summary-container');
    if (!container) return;
    container.innerHTML = '';

    for (let id = 1; id <= 5; id++) {
      const limit = this.jointLimits[id];
      const row = document.createElement('div');
      row.className = 'summary-row';
      row.innerHTML = `
        <span class="summary-joint-name">J${id} (${limit.label})</span>
        <span class="summary-angles" id="summary-pos-${id}">150.0°</span>
      `;
      container.appendChild(row);
    }
  },

  onSliderInput(jointId, value) {
    const val = parseFloat(value);
    this.currentGoalAngles[jointId] = val;

    const numElem = document.getElementById(`num-j${jointId}`);
    if (numElem) {
      numElem.value = val.toFixed(1);
      numElem.classList.remove('input-error');
    }

    const goalElem = document.getElementById(`goal-val-${jointId}`);
    if (goalElem) goalElem.innerText = `${val.toFixed(1)}°`;

    if (this.arm3D) {
      this.arm3D.updateAngles(this.presentAngles, this.currentGoalAngles);
    }
  },

  onNumberInput(jointId, value) {
    const limit = this.jointLimits[jointId];
    let val = parseFloat(value);
    const numElem = document.getElementById(`num-j${jointId}`);

    if (isNaN(val) || val < limit.min || val > limit.max) {
      if (numElem) numElem.classList.add('input-error');
      this.showToast(`Joint ${jointId} angle must be between ${limit.min}° and ${limit.max}°`, 'warning');
      return;
    }

    if (numElem) numElem.classList.remove('input-error');

    this.currentGoalAngles[jointId] = val;
    const sliderElem = document.getElementById(`slider-j${jointId}`);
    if (sliderElem) sliderElem.value = val;
    const goalElem = document.getElementById(`goal-val-${jointId}`);
    if (goalElem) goalElem.innerText = `${val.toFixed(1)}°`;

    if (this.arm3D) {
      this.arm3D.updateAngles(this.presentAngles, this.currentGoalAngles);
    }
  },

  presetJointHome(jointId) {
    this.onSliderInput(jointId, 150.0);
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
      this.logEvent(`Moved joints: ${JSON.stringify(this.currentGoalAngles)} (Speed: ${speed})`, 'info');
    } catch (err) {
      this.showToast(`Move failed: ${err.message}`, 'error');
      this.logEvent(`Move error: ${err.message}`, 'error');
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
    const sleepAngles = { 1: 150, 2: 45, 3: 255, 4: 45, 5: 150 };
    for (let i = 1; i <= 5; i++) {
      this.onSliderInput(i, sleepAngles[i]);
    }
    await this.applyMove();
  },

  async presetReach() {
    const reachAngles = { 1: 150, 2: 110, 3: 190, 4: 150, 5: 150 };
    for (let i = 1; i <= 5; i++) {
      this.onSliderInput(i, reachAngles[i]);
    }
    await this.applyMove();
  },

  async presetVertical() {
    const vertAngles = { 1: 150, 2: 240, 3: 150, 4: 150, 5: 150 };
    for (let i = 1; i <= 5; i++) {
      this.onSliderInput(i, vertAngles[i]);
    }
    await this.applyMove();
  },

  async setTorque(enable) {
    try {
      const res = await ArmAPI.setTorque(enable);
      this.showToast(res.message, enable ? 'success' : 'info');
      this.logEvent(`Torque ${enable ? 'ENABLED' : 'DISABLED'}`, enable ? 'info' : 'warn');
    } catch (err) {
      this.showToast(`Torque error: ${err.message}`, 'error');
      this.logEvent(`Torque command failed: ${err.message}`, 'error');
    }
  },

  async triggerEstop() {
    try {
      const res = await ArmAPI.triggerEstop();
      this.showToast('EMERGENCY STOP ACTIVATED!', 'error', 5000);
      this.logEvent('EMERGENCY STOP TRIGGERED BY USER!', 'error');
    } catch (err) {
      this.showToast(`E-STOP error: ${err.message}`, 'error');
    }
  },

  async clearFault() {
    try {
      const res = await ArmAPI.clearFault();
      this.showToast(res.message, 'success');
      this.logEvent('Safety fault latched cleared successfully.', 'info');
    } catch (err) {
      this.showToast(`Clear fault error: ${err.message}`, 'error');
      this.logEvent(`Clear fault failed: ${err.message}`, 'error');
    }
  },

  // CARTESIAN WORKSPACE
  setCartesianTarget(x, y, z, pitch) {
    document.getElementById('cart-x').value = x;
    document.getElementById('cart-y').value = y;
    document.getElementById('cart-z').value = z;
    document.getElementById('cart-pitch').value = pitch;
    this.validateCartesianTarget();
  },

  async validateCartesianTarget() {
    const x = parseFloat(document.getElementById('cart-x').value) || 0;
    const y = parseFloat(document.getElementById('cart-y').value) || 0;
    const z = parseFloat(document.getElementById('cart-z').value) || 0;
    const pitch = parseFloat(document.getElementById('cart-pitch').value) || 0;

    const badge = document.getElementById('ik-status-badge');
    const preview = document.getElementById('ik-angles-preview');

    try {
      const res = await ArmAPI.validateXYZ(x, y, z, pitch);
      if (res.success && res.data && res.data.reachable) {
        if (badge) {
          badge.className = 'ik-status-badge reachable';
          badge.innerHTML = '<i class="fa-solid fa-check"></i> TARGET REACHABLE';
        }
        const a = res.data.angles;
        if (preview && a) {
          preview.innerText = `Target Joint Angles: J1: ${a[1]}°, J2: ${a[2]}°, J3: ${a[3]}°, J4: ${a[4]}°, J5: ${a[5]}°`;
        }
        if (this.arm3D) {
          this.arm3D.updateAngles(this.presentAngles, a);
        }
      } else {
        if (badge) {
          badge.className = 'ik-status-badge unreachable';
          badge.innerHTML = `<i class="fa-solid fa-xmark"></i> UNREACHABLE (${res.message || 'Out of reach'})`;
        }
        if (preview) {
          preview.innerText = 'Calculated position exceeds physical arm length or software joint limits.';
        }
      }
    } catch (e) {
      if (badge) {
        badge.className = 'ik-status-badge unreachable';
        badge.innerHTML = `<i class="fa-solid fa-xmark"></i> INVALID TARGET (${e.message})`;
      }
    }
  },

  async executeCartesianMove() {
    const x = parseFloat(document.getElementById('cart-x').value) || 0;
    const y = parseFloat(document.getElementById('cart-y').value) || 0;
    const z = parseFloat(document.getElementById('cart-z').value) || 0;
    const pitch = parseFloat(document.getElementById('cart-pitch').value) || 0;
    const speed = document.getElementById('speed-slider').value;

    try {
      const res = await ArmAPI.moveXYZ(x, y, z, pitch, speed);
      this.showToast(res.message, 'success');
      this.logEvent(`Moved Cartesian XYZ (${x}, ${y}, ${z}) Pitch: ${pitch}°`, 'info');
    } catch (err) {
      this.showToast(`Cartesian move error: ${err.message}`, 'error');
      this.logEvent(`Cartesian move failed: ${err.message}`, 'error');
    }
  },

  // POSE LIBRARY & SEQUENCES
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
      this.logEvent(`Saved pose '${name}'`, 'info');
      if (input) input.value = '';
      this.refreshPoses();
    } catch (err) {
      this.showToast(`Save pose error: ${err.message}`, 'error');
    }
  },

  async refreshPoses() {
    try {
      const res = await ArmAPI.listPoses();
      this.savedPoses = res.data || [];
      this.renderPoseList(this.savedPoses);
      this.updateSequencePoseSelector(this.savedPoses);
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

  updateSequencePoseSelector(poses) {
    const select = document.getElementById('seq-pose-selector');
    if (!select) return;
    select.innerHTML = '';
    poses.forEach(p => {
      const opt = document.createElement('option');
      opt.value = p.name;
      opt.innerText = p.name;
      select.appendChild(opt);
    });
  },

  async executePose(name) {
    try {
      const res = await ArmAPI.executePose(name);
      this.showToast(`Executing pose '${name}'`, 'info');
      this.logEvent(`Executing pose '${name}'`, 'info');
    } catch (err) {
      this.showToast(`Execute pose error: ${err.message}`, 'error');
    }
  },

  async deletePose(name) {
    try {
      await ArmAPI.deletePose(name);
      this.showToast(`Pose '${name}' deleted`, 'info');
      this.logEvent(`Deleted pose '${name}'`, 'info');
      this.refreshPoses();
    } catch (err) {
      this.showToast(`Delete pose error: ${err.message}`, 'error');
    }
  },

  addStepToSequence() {
    const select = document.getElementById('seq-pose-selector');
    if (!select || !select.value) {
      this.showToast('Select a pose to add', 'warning');
      return;
    }
    const poseName = select.value;
    this.currentSequenceSteps.push({
      pose_name: poseName,
      delay_seconds: 1.0,
      speed: 100
    });
    this.renderSequenceSteps();
  },

  renderSequenceSteps() {
    const container = document.getElementById('sequence-steps-container');
    if (!container) return;

    if (this.currentSequenceSteps.length === 0) {
      container.innerHTML = '<p class="empty-list-msg">No steps added to sequence yet.</p>';
      return;
    }

    container.innerHTML = '';
    this.currentSequenceSteps.forEach((step, idx) => {
      const item = document.createElement('div');
      item.className = 'step-item';
      item.innerHTML = `
        <span class="pose-item-name">#${idx + 1}: ${step.pose_name}</span>
        <div class="pose-item-actions">
          <label style="font-size:11px; color:#9ca3af;">Delay(s):</label>
          <input type="number" class="joint-num-input" style="width:55px;" value="${step.delay_seconds}" step="0.5" min="0.1" 
                 onchange="ArmControl.currentSequenceSteps[${idx}].delay_seconds = parseFloat(this.value)">
          <button class="btn btn-xs btn-outline" onclick="ArmControl.removeSequenceStep(${idx})">
            <i class="fa-solid fa-trash"></i>
          </button>
        </div>
      `;
      container.appendChild(item);
    });
  },

  removeSequenceStep(idx) {
    this.currentSequenceSteps.splice(idx, 1);
    this.renderSequenceSteps();
  },

  async saveSequence() {
    const nameInput = document.getElementById('sequence-name-input');
    const name = nameInput ? nameInput.value.trim() : '';
    if (!name) {
      this.showToast('Please enter a sequence name', 'warning');
      return;
    }
    if (this.currentSequenceSteps.length === 0) {
      this.showToast('Add at least one step to the sequence', 'warning');
      return;
    }
    try {
      await ArmAPI.saveSequence(name, this.currentSequenceSteps);
      this.showToast(`Sequence '${name}' saved`, 'success');
      this.logEvent(`Saved sequence '${name}'`, 'info');
      this.refreshSequences();
    } catch (err) {
      this.showToast(`Save sequence error: ${err.message}`, 'error');
    }
  },

  async refreshSequences() {
    try {
      const res = await ArmAPI.listSequences();
      // Sequnces stored if needed
    } catch (e) {
      console.warn('Refresh sequences error:', e);
    }
  },

  async playCurrentSequence() {
    if (this.currentSequenceSteps.length === 0) {
      this.showToast('No steps to execute', 'warning');
      return;
    }
    const speed = document.getElementById('speed-slider').value;
    try {
      const res = await ArmAPI.executeSequence(this.currentSequenceSteps, speed);
      this.showToast(res.message, 'success');
      this.logEvent(`Executing sequence with ${this.currentSequenceSteps.length} steps`, 'info');
    } catch (err) {
      this.showToast(`Sequence error: ${err.message}`, 'error');
    }
  },

  // TRAJECTORY PLANNING (PHASE 1)
  trajectoryWaypoints: [
    { name: "Home Pose", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0 }, duration: 2.0 },
    { name: "Forward Reach", angles: { 1: 150.0, 2: 110.0, 3: 190.0, 4: 140.0, 5: 150.0 }, duration: 2.5 },
    { name: "Return Home", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0 }, duration: 2.0 }
  ],
  trajectoryPreviewChart: null,
  trajectoryPollInterval: null,

  initTrajectory() {
    this.renderTrajectoryTable();
    this.initTrajectoryChart();
    setTimeout(() => this.previewTrajectory(true), 200);
  },

  initTrajectoryChart() {
    const ctx = document.getElementById('trajectoryPreviewChart');
    if (!ctx || typeof Chart === 'undefined') return;

    this.trajectoryPreviewChart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'J1 Base', borderColor: '#3b82f6', data: [], borderWidth: 2, fill: false, tension: 0.1, pointRadius: 0 },
          { label: 'J2 Shoulder', borderColor: '#10b981', data: [], borderWidth: 2, fill: false, tension: 0.1, pointRadius: 0 },
          { label: 'J3 Elbow', borderColor: '#f59e0b', data: [], borderWidth: 2, fill: false, tension: 0.1, pointRadius: 0 },
          { label: 'J4 Wrist P', borderColor: '#ef4444', data: [], borderWidth: 2, fill: false, tension: 0.1, pointRadius: 0 },
          { label: 'J5 Wrist R', borderColor: '#06b6d4', data: [], borderWidth: 2, fill: false, tension: 0.1, pointRadius: 0 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: {
            title: { display: true, text: 'Time (s)', color: '#9ca3af', font: { size: 10 } },
            grid: { color: 'rgba(255,255,255,0.05)' },
            ticks: { color: '#9ca3af', font: { size: 10 } }
          },
          y: {
            title: { display: true, text: 'Angle (°)', color: '#9ca3af', font: { size: 10 } },
            grid: { color: 'rgba(255,255,255,0.05)' },
            ticks: { color: '#9ca3af', font: { size: 10 } }
          }
        },
        plugins: {
          legend: { labels: { color: '#f3f4f6', font: { size: 11 } } },
          tooltip: { mode: 'index', intersect: false }
        }
      }
    });
  },

  renderTrajectoryTable() {
    const tbody = document.getElementById('trajectory-table-body');
    if (!tbody) return;

    tbody.innerHTML = '';
    this.trajectoryWaypoints.forEach((wp, idx) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><strong>#${idx + 1}</strong></td>
        <td><input type="text" class="traj-input text-left" value="${wp.name || 'WP ' + (idx + 1)}" onchange="ArmControl.updateWaypointField(${idx}, 'name', this.value)"></td>
        <td><input type="number" class="traj-input" step="0.5" min="0" max="300" value="${wp.angles[1]}" onchange="ArmControl.updateWaypointField(${idx}, 1, this.value)"></td>
        <td><input type="number" class="traj-input" step="0.5" min="15" max="285" value="${wp.angles[2]}" onchange="ArmControl.updateWaypointField(${idx}, 2, this.value)"></td>
        <td><input type="number" class="traj-input" step="0.5" min="15" max="285" value="${wp.angles[3]}" onchange="ArmControl.updateWaypointField(${idx}, 3, this.value)"></td>
        <td><input type="number" class="traj-input" step="0.5" min="10" max="290" value="${wp.angles[4]}" onchange="ArmControl.updateWaypointField(${idx}, 4, this.value)"></td>
        <td><input type="number" class="traj-input" step="0.5" min="0" max="300" value="${wp.angles[5]}" onchange="ArmControl.updateWaypointField(${idx}, 5, this.value)"></td>
        <td><input type="number" class="traj-input" step="0.1" min="0.1" max="60" value="${wp.duration}" onchange="ArmControl.updateWaypointField(${idx}, 'duration', this.value)"></td>
        <td>
          <button class="btn btn-xs btn-outline" onclick="ArmControl.removeTrajectoryWaypoint(${idx})" title="Delete Waypoint">
            <i class="fa-solid fa-trash-can"></i>
          </button>
        </td>
      `;
      tbody.appendChild(tr);
    });

    const metricWp = document.getElementById('traj-metric-waypoints');
    if (metricWp) metricWp.innerText = this.trajectoryWaypoints.length;
  },

  updateWaypointField(idx, field, val) {
    if (!this.trajectoryWaypoints[idx]) return;
    if (field === 'name') {
      this.trajectoryWaypoints[idx].name = val;
    } else if (field === 'duration') {
      this.trajectoryWaypoints[idx].duration = Math.max(0.05, parseFloat(val) || 1.0);
    } else {
      const sid = parseInt(field);
      this.trajectoryWaypoints[idx].angles[sid] = parseFloat(val) || 150.0;
    }
  },

  addCurrentPoseAsWaypoint() {
    const newWp = {
      name: `Pose @ ${new Date().toLocaleTimeString()}`,
      angles: { ...this.presentAngles },
      duration: 2.0
    };
    this.trajectoryWaypoints.push(newWp);
    this.renderTrajectoryTable();
    this.showToast('Current arm pose added as waypoint', 'info');
  },

  addBlankWaypoint() {
    const lastWp = this.trajectoryWaypoints[this.trajectoryWaypoints.length - 1];
    const newAngles = lastWp ? { ...lastWp.angles } : { 1: 150, 2: 150, 3: 150, 4: 150, 5: 150 };
    this.trajectoryWaypoints.push({
      name: `Waypoint ${this.trajectoryWaypoints.length + 1}`,
      angles: newAngles,
      duration: 2.0
    });
    this.renderTrajectoryTable();
  },

  removeTrajectoryWaypoint(idx) {
    if (this.trajectoryWaypoints.length <= 2) {
      this.showToast('Trajectory must have at least 2 waypoints', 'warning');
      return;
    }
    this.trajectoryWaypoints.splice(idx, 1);
    this.renderTrajectoryTable();
  },

  clearWaypoints() {
    this.trajectoryWaypoints = [
      { name: "Start (Home)", angles: { ...this.presentAngles }, duration: 0.1 },
      { name: "Target Goal", angles: { ...this.currentGoalAngles }, duration: 2.0 }
    ];
    this.renderTrajectoryTable();
    this.showToast('Reset to 2 basic waypoints (Present -> Goal)', 'info');
  },

  loadTrajectoryPreset(preset) {
    if (!preset) return;
    if (preset === 'pick_place') {
      this.trajectoryWaypoints = [
        { name: "Standby Home", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0 }, duration: 1.5 },
        { name: "Above Pick", angles: { 1: 180.0, 2: 130.0, 3: 170.0, 4: 135.0, 5: 150.0 }, duration: 2.0 },
        { name: "Grip Object", angles: { 1: 180.0, 2: 110.0, 3: 195.0, 4: 125.0, 5: 150.0 }, duration: 1.5 },
        { name: "Lift Clearance", angles: { 1: 180.0, 2: 140.0, 3: 160.0, 4: 140.0, 5: 150.0 }, duration: 1.8 },
        { name: "Place Target", angles: { 1: 120.0, 2: 115.0, 3: 190.0, 4: 130.0, 5: 150.0 }, duration: 2.5 },
        { name: "Return Home", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0 }, duration: 2.0 }
      ];
    } else if (preset === 'scan_sweep') {
      this.trajectoryWaypoints = [
        { name: "Scan Center", angles: { 1: 150.0, 2: 120.0, 3: 180.0, 4: 150.0, 5: 150.0 }, duration: 1.5 },
        { name: "Scan Left", angles: { 1: 210.0, 2: 120.0, 3: 180.0, 4: 150.0, 5: 150.0 }, duration: 2.0 },
        { name: "Scan Right", angles: { 1: 90.0, 2: 120.0, 3: 180.0, 4: 150.0, 5: 150.0 }, duration: 3.0 },
        { name: "Center Home", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0 }, duration: 2.0 }
      ];
    } else if (preset === 'wave') {
      this.trajectoryWaypoints = [
        { name: "Vertical Posture", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0 }, duration: 1.5 },
        { name: "Wave Tilt Left", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 180.0 }, duration: 1.0 },
        { name: "Wave Tilt Right", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 120.0 }, duration: 1.2 },
        { name: "Wave Tilt Left 2", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 180.0 }, duration: 1.2 },
        { name: "Wave Neutral", angles: { 1: 150.0, 2: 150.0, 3: 150.0, 4: 150.0, 5: 150.0 }, duration: 1.0 }
      ];
    }
    this.renderTrajectoryTable();
    this.previewTrajectory(true);
    const sel = document.getElementById('traj-preset-select');
    if (sel) sel.value = '';
  },

  onTrajectoryParamChange() {
    this.previewTrajectory(true);
  },

  async previewTrajectory(silent = false) {
    if (this.trajectoryWaypoints.length < 2) {
      if (!silent) this.showToast('Need at least 2 waypoints for trajectory', 'warning');
      return;
    }

    const interp = document.getElementById('traj-interp-select')?.value || 'quintic';
    const rate = parseFloat(document.getElementById('traj-sample-rate')?.value) || 20.0;

    const banner = document.getElementById('traj-validation-banner');
    const icon = document.getElementById('traj-val-icon');
    const title = document.getElementById('traj-val-title');
    const details = document.getElementById('traj-val-details');

    try {
      const res = await ArmAPI.previewTrajectory(this.trajectoryWaypoints, interp, rate);
      if (res && res.data) {
        const d = res.data;
        if (d.is_valid) {
          if (banner) {
            banner.className = 'validation-banner valid';
            if (icon) icon.className = 'fa-solid fa-circle-check';
            if (title) title.innerText = `Validation Passed: Feasible ${d.interpolation.toUpperCase()} Trajectory`;
            if (details) {
              const warns = d.validation_warnings.length ? `<br>Note: ${d.validation_warnings.join('; ')}` : '';
              details.innerHTML = `Total duration: <strong>${d.total_duration}s</strong> across ${d.waypoints.length} waypoints (${d.time_points.length} samples at ${d.sample_rate}Hz). Feasible within all joint, velocity, and acceleration limits.${warns}`;
            }
          }

          // Update chart
          this.updateTrajectoryChart(d);

          // Update metrics
          const durEl = document.getElementById('traj-metric-duration');
          if (durEl) durEl.innerText = `${d.total_duration}s`;

          const maxVelEl = document.getElementById('traj-metric-max-vel');
          const maxPeak = Math.max(...Object.values(d.peak_velocities));
          if (maxVelEl) maxVelEl.innerText = `${maxPeak.toFixed(1)}°/s`;

          for (let sid = 1; sid <= 5; sid++) {
            const chip = document.getElementById(`pv-${sid}`);
            if (chip && d.peak_velocities[sid] !== undefined) {
              chip.innerText = `${d.peak_velocities[sid].toFixed(1)}°/s`;
            }
          }

          if (!silent) this.showToast('Trajectory preview generated successfully', 'success');
        } else {
          if (banner) {
            banner.className = 'validation-banner invalid';
            if (icon) icon.className = 'fa-solid fa-triangle-exclamation';
            if (title) title.innerText = 'Validation Failed: Infeasible Trajectory';
            if (details) details.innerHTML = d.validation_errors.join('<br>');
          }
          if (!silent) this.showToast(`Trajectory invalid: ${d.validation_errors[0]}`, 'error');
        }
      }
    } catch (err) {
      if (banner) {
        banner.className = 'validation-banner invalid';
        if (icon) icon.className = 'fa-solid fa-triangle-exclamation';
        if (title) title.innerText = 'Validation / Preview Error';
        if (details) details.innerText = err.message;
      }
      if (!silent) this.showToast(`Preview error: ${err.message}`, 'error');
    }
  },

  updateTrajectoryChart(planData) {
    if (!this.trajectoryPreviewChart || !planData.time_points) return;

    this.trajectoryPreviewChart.data.labels = planData.time_points.map(t => t.toFixed(2));
    for (let sid = 1; sid <= 5; sid++) {
      if (this.trajectoryPreviewChart.data.datasets[sid - 1]) {
        this.trajectoryPreviewChart.data.datasets[sid - 1].data = planData.positions[sid] || planData.positions[String(sid)] || [];
      }
    }
    this.trajectoryPreviewChart.update();
  },

  async executeTrajectory() {
    if (this.trajectoryWaypoints.length < 2) {
      this.showToast('Need at least 2 waypoints', 'warning');
      return;
    }

    const interp = document.getElementById('traj-interp-select')?.value || 'quintic';
    const rate = parseFloat(document.getElementById('traj-sample-rate')?.value) || 20.0;

    try {
      const res = await ArmAPI.executeTrajectory(this.trajectoryWaypoints, interp, rate);
      this.showToast(res.message, 'success');
      this.logEvent(`Started trajectory execution: ${res.message}`, 'info');

      // Update badge
      const badge = document.getElementById('traj-exec-badge');
      if (badge) {
        badge.className = 'status-badge in-progress';
        badge.innerText = 'RUNNING';
      }

      this.startTrajectoryStatusPolling();
    } catch (err) {
      this.showToast(`Execution error: ${err.message}`, 'error');
      this.logEvent(`Trajectory execution error: ${err.message}`, 'error');
    }
  },

  async cancelTrajectory() {
    try {
      const res = await ArmAPI.cancelTrajectory();
      this.showToast('Trajectory execution cancelled', 'warning');
      this.logEvent('Trajectory execution cancelled by user.', 'warn');
      this.stopTrajectoryStatusPolling();

      const badge = document.getElementById('traj-exec-badge');
      if (badge) {
        badge.className = 'status-badge offline';
        badge.innerText = 'CANCELLED';
      }
    } catch (err) {
      this.showToast(`Cancel error: ${err.message}`, 'error');
    }
  },

  startTrajectoryStatusPolling() {
    this.stopTrajectoryStatusPolling();
    this.trajectoryPollInterval = setInterval(async () => {
      try {
        const res = await ArmAPI.getTrajectoryStatus();
        if (res && res.data) {
          const s = res.data;
          const pct = document.getElementById('traj-progress-pct');
          const bar = document.getElementById('traj-progress-bar');
          const elapsed = document.getElementById('traj-elapsed-time');
          const total = document.getElementById('traj-total-time');
          const activeWp = document.getElementById('traj-metric-active-wp');
          const badge = document.getElementById('traj-exec-badge');

          if (pct) pct.innerText = `${s.progress_percent.toFixed(1)}%`;
          if (bar) bar.style.width = `${s.progress_percent}%`;
          if (elapsed) elapsed.innerText = `${s.current_time.toFixed(2)}s`;
          if (total) total.innerText = `${s.total_duration.toFixed(2)}s`;
          if (activeWp) activeWp.innerText = s.current_waypoint_index > 0 ? `#${s.current_waypoint_index}` : '-';

          if (badge) {
            badge.innerText = s.state;
            if (s.state === 'RUNNING') badge.className = 'status-badge in-progress';
            else if (s.state === 'COMPLETED') badge.className = 'status-badge ready';
            else if (s.state === 'CANCELLED') badge.className = 'status-badge offline';
            else if (s.state === 'FAULT') badge.className = 'status-badge fault';
          }

          if (s.state !== 'RUNNING') {
            this.stopTrajectoryStatusPolling();
            if (s.state === 'COMPLETED') {
              this.showToast('Trajectory completed successfully', 'success');
              this.logEvent('Trajectory execution completed.', 'info');
            }
          }
        }
      } catch (e) {
        // Silent polling error
      }
    }, 150);
  },

  stopTrajectoryStatusPolling() {
    if (this.trajectoryPollInterval) {
      clearInterval(this.trajectoryPollInterval);
      this.trajectoryPollInterval = null;
    }
  },

  // CHARTING & TELEMETRY
  initChart() {
    const ctx = document.getElementById('telemetryChart');
    if (!ctx || typeof Chart === 'undefined') return;

    this.telemetryChart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'J1 Base', borderColor: '#3b82f6', data: [], borderWidth: 2, fill: false, tension: 0.2 },
          { label: 'J2 Shoulder', borderColor: '#10b981', data: [], borderWidth: 2, fill: false, tension: 0.2 },
          { label: 'J3 Elbow', borderColor: '#f59e0b', data: [], borderWidth: 2, fill: false, tension: 0.2 },
          { label: 'J4 Wrist P', borderColor: '#ef4444', data: [], borderWidth: 2, fill: false, tension: 0.2 },
          { label: 'J5 Wrist R', borderColor: '#06b6d4', data: [], borderWidth: 2, fill: false, tension: 0.2 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { grid: { color: 'rgba(255,255,255,0.05)' }, ticks: { color: '#9ca3af', font: { size: 10 } } },
          y: { grid: { color: 'rgba(255,255,255,0.05)' }, ticks: { color: '#9ca3af', font: { size: 10 } } }
        },
        plugins: {
          legend: { labels: { color: '#f3f4f6', font: { size: 11 } } }
        }
      }
    });
  },

  toggleChartPause() {
    this.chartPaused = !this.chartPaused;
    const btn = document.getElementById('btn-pause-chart');
    if (btn) btn.innerHTML = this.chartPaused ? '<i class="fa-solid fa-play"></i> Resume Chart' : '<i class="fa-solid fa-pause"></i> Pause Chart';
  },

  onChartMetricChange(metric) {
    if (!this.telemetryChart) return;
    this.telemetryChart.data.labels = [];
    this.telemetryChart.data.datasets.forEach(ds => ds.data = []);
    this.telemetryChart.update();
  },

  updateChartData(state) {
    if (!this.telemetryChart || this.chartPaused || !state.servos) return;

    const nowStr = new Date().toLocaleTimeString();
    const metric = document.getElementById('chart-metric-select')?.value || 'position';

    if (this.telemetryChart.data.labels.length > 40) {
      this.telemetryChart.data.labels.shift();
      this.telemetryChart.data.datasets.forEach(ds => ds.data.shift());
    }

    this.telemetryChart.data.labels.push(nowStr);

    for (let sid = 1; sid <= 5; sid++) {
      const s = state.servos[sid] || state.servos[String(sid)];
      if (s && this.telemetryChart.data.datasets[sid - 1]) {
        let val = s.present_position;
        if (metric === 'load') val = s.load;
        if (metric === 'temperature') val = s.temperature;
        if (metric === 'voltage') val = s.voltage;
        this.telemetryChart.data.datasets[sid - 1].data.push(val);
      }
    }

    this.telemetryChart.update();
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
      dot.className = connected ? 'status-dot online' : 'status-dot offline';
      val.innerText = connected ? 'STREAMING' : 'DISCONNECTED';
    }
    this.updateBackendBadge(connected);

    if (!connected) {
      this.logEvent('WebSocket connection lost. Reconnecting...', 'warn');
    } else {
      this.logEvent('WebSocket telemetry stream connected.', 'info');
    }
  },

  onTelemetryData(state) {
    const now = Date.now();
    if (this.lastTelemetryTimestamp > 0) {
      const dataAge = now - this.lastTelemetryTimestamp;
      const ageElem = document.getElementById('diag-data-age');
      if (ageElem) ageElem.innerText = `${dataAge} ms`;
    }
    this.lastTelemetryTimestamp = now;

    // 1. Mode badge
    const valMode = document.getElementById('val-mode');
    if (valMode) valMode.innerText = state.simulation ? 'SIMULATION' : 'HARDWARE';

    // 2. System status badge
    const badgeSys = document.getElementById('badge-system');
    const valSys = document.getElementById('val-system');
    if (badgeSys && valSys) {
      valSys.innerText = state.system_status;
      badgeSys.className = `status-badge ${state.system_status.toLowerCase()}`;
    }

    // 3. Torque status badge
    const valTorque = document.getElementById('val-torque');
    if (valTorque) valTorque.innerText = state.torque_enabled ? 'ON' : 'OFF';

    // 4. Diagnostics Watchdog state
    const watchdogVal = document.getElementById('diag-watchdog');
    const overviewWatchdog = document.getElementById('overview-watchdog');
    let wText = 'IDLE';

    if (state.watchdog_tripped) {
      wText = 'TRIPPED (TIMEOUT)';
    } else if (state.active_control) {
      wText = 'ACTIVE (5.0s)';
    }

    if (watchdogVal) {
      watchdogVal.innerText = wText;
      watchdogVal.className = state.watchdog_tripped ? 'diag-val fault-text' : 'diag-val';
    }
    if (overviewWatchdog) overviewWatchdog.innerText = wText;

    // Last command timestamp
    const lastCmdVal = document.getElementById('diag-last-cmd');
    const overviewLastCmd = document.getElementById('overview-last-cmd');
    let cmdText = 'None';
    if (state.last_command_timestamp > 0) {
      const elapsed = ((now / 1000) - state.last_command_timestamp).toFixed(1);
      cmdText = `${elapsed}s ago`;
    }
    if (lastCmdVal) lastCmdVal.innerText = cmdText;
    if (overviewLastCmd) overviewLastCmd.innerText = cmdText;

    const faultVal = document.getElementById('diag-fault-reason');
    const overviewFault = document.getElementById('overview-fault-reason');
    if (faultVal) faultVal.innerText = state.fault_reason || 'None';
    if (overviewFault) overviewFault.innerText = state.fault_reason || 'None';

    // Handle single notification on latched fault state transition
    if (state.fault_latched && state.fault_reason) {
      if (this.lastFaultReason !== state.fault_reason) {
        this.lastFaultReason = state.fault_reason;
        this.showToast(state.fault_reason, 'error', 6000);
        this.logEvent(`FAULT LATCHED: ${state.fault_reason}`, 'error');
      }
    } else {
      this.lastLatchedFaultReason = null;
      this.lastFaultReason = null;
    }

    // 5. Update Servo Present Angles & 3D Model
    if (state.servos) {
      for (let sid = 1; sid <= 5; sid++) {
        const s = state.servos[sid] || state.servos[String(sid)];
        if (s) {
          this.presentAngles[sid] = s.present_position;

          const posLabel = document.getElementById(`pos-val-${sid}`);
          if (posLabel) posLabel.innerText = `${s.present_position.toFixed(1)}°`;

          const sumPos = document.getElementById(`summary-pos-${sid}`);
          if (sumPos) sumPos.innerText = `${s.present_position.toFixed(1)}°`;

          if (!this.isUserInteracting) {
            const goalLabel = document.getElementById(`goal-val-${sid}`);
            if (goalLabel) goalLabel.innerText = `${s.goal_position.toFixed(1)}°`;
          }
        }
      }

      if (this.arm3D) {
        this.arm3D.updateAngles(this.presentAngles, this.currentGoalAngles);
      }
    }

    // 6. Update FK Overlay Readout
    this.updateFKReadout();

    // 7. Update Telemetry Table & Chart
    this.updateTelemetryTable(state);
    this.updateChartData(state);
  },

  async updateFKReadout() {
    try {
      const res = await ArmAPI.getFK();
      if (res && res.data) {
        const { x, y, z, pitch } = res.data;
        const overlay = document.getElementById('overlay-xyz');
        if (overlay) overlay.innerText = `X: ${x}, Y: ${y}, Z: ${z} mm | Pitch: ${pitch}°`;

        const fkX = document.getElementById('fk-val-x');
        const fkY = document.getElementById('fk-val-y');
        const fkZ = document.getElementById('fk-val-z');
        const fkP = document.getElementById('fk-val-pitch');

        if (fkX) fkX.innerText = `${x} mm`;
        if (fkY) fkY.innerText = `${y} mm`;
        if (fkZ) fkZ.innerText = `${z} mm`;
        if (fkP) fkP.innerText = `${pitch}°`;
      }
    } catch (e) {
      // FK update silent fallback
    }
  },

  updateTelemetryTable(state) {
    const tbody = document.getElementById('telemetry-table-body');
    if (!tbody || !state.servos) return;

    let rowsHtml = '';
    for (let sid = 1; sid <= 5; sid++) {
      const s = state.servos[sid] || state.servos[String(sid)];
      if (!s) continue;

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
          <td>${s.torque_enabled ? '<span style="color:#10b981;font-weight:bold">ON</span>' : '<span style="color:#ef4444;font-weight:bold">OFF</span>'}</td>
          <td>${s.comm_status}</td>
        </tr>
      `;
    }
    tbody.innerHTML = rowsHtml;
  },

  logEvent(message, type = 'info') {
    const timeStr = new Date().toLocaleTimeString();
    const entry = { time: timeStr, msg: message, type: type };
    this.eventLogs.unshift(entry);
    if (this.eventLogs.length > 100) this.eventLogs.pop();

    const container = document.getElementById('event-log-container');
    if (!container) return;

    const row = document.createElement('div');
    row.className = `log-item log-${type}`;
    row.innerHTML = `<span class="log-time">[${timeStr}]</span> <span>${message}</span>`;
    container.insertBefore(row, container.firstChild);
  },

  clearEventLog() {
    this.eventLogs = [];
    const container = document.getElementById('event-log-container');
    if (container) container.innerHTML = '';
  },

  showToast(message, type = 'info', duration = 3000) {
    const container = document.getElementById('toast-container');
    if (!container || !message) return;

    const trimmedMsg = String(message).trim();

    // Deduplicate against active toast elements in container
    const existingToasts = container.querySelectorAll('.toast span');
    for (let i = 0; i < existingToasts.length; i++) {
      if (existingToasts[i].innerText.trim() === trimmedMsg) {
        return; // Skip duplicate active notification
      }
    }

    // Deduplicate against very recent identical error/warning messages (within 3 seconds)
    if (!this._recentToasts) this._recentToasts = new Map();
    const now = Date.now();
    const lastShown = this._recentToasts.get(trimmedMsg);
    if (lastShown && (now - lastShown < 3000)) {
      return;
    }
    this._recentToasts.set(trimmedMsg, now);

    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    let icon = 'fa-info-circle';
    if (type === 'success') icon = 'fa-circle-check';
    if (type === 'warning') icon = 'fa-triangle-exclamation';
    if (type === 'error') icon = 'fa-circle-xmark';

    toast.innerHTML = `<i class="fa-solid ${icon}"></i> <span>${trimmedMsg}</span>`;
    container.appendChild(toast);

    setTimeout(() => {
      toast.style.opacity = '0';
      toast.style.transition = 'opacity 0.3s ease';
      setTimeout(() => toast.remove(), 300);
    }, duration);
  }

};

document.addEventListener('DOMContentLoaded', () => ArmControl.init());
