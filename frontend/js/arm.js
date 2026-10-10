/**
 * REST API client for 5-DOF Dynamixel Robotic Arm Control.
 */

const ArmAPI = {
  async _request(url, method = 'GET', data = null) {
    const opts = {
      method: method,
      headers: { 'Content-Type': 'application/json' }
    };
    if (data) {
      opts.body = JSON.stringify(data);
    }
    try {
      const resp = await fetch(url, opts);
      const json = await resp.json();
      if (!resp.ok || (json && json.success === false)) {
        throw new Error(json.message || `HTTP Error ${resp.status}`);
      }
      return json;
    } catch (err) {
      console.error(`API Request Error [${method} ${url}]:`, err);
      throw err;
    }
  },

  async getState() {
    return await fetch('/api/state').then(r => r.json());
  },

  async getConfig() {
    return await this._request('/api/config');
  },

  async moveJoints(angles, speed = 100) {
    return await this._request('/api/move', 'POST', {
      angles: angles,
      speed: parseInt(speed)
    });
  },

  async setTorque(enable, jointId = null) {
    return await this._request('/api/torque', 'POST', {
      enable: enable,
      joint_id: jointId
    });
  },

  async triggerEstop() {
    return await this._request('/api/estop', 'POST');
  },

  async clearFault() {
    return await this._request('/api/clear_fault', 'POST');
  },

  async listPoses() {
    return await this._request('/api/poses');
  },

  async saveCurrentPose(name, speed = 100) {
    const url = `/api/poses/save_current?name=${encodeURIComponent(name)}&speed=${speed}`;
    return await this._request(url, 'POST');
  },

  async deletePose(name) {
    return await this._request(`/api/poses/${encodeURIComponent(name)}`, 'DELETE');
  },

  async executePose(name) {
    return await this._request(`/api/poses/${encodeURIComponent(name)}/execute`, 'POST');
  },

  // Cartesian Endpoints
  async getFK() {
    return await this._request('/api/kinematics/fk');
  },

  async validateXYZ(x, y, z, pitch = 0.0) {
    return await this._request('/api/kinematics/validate', 'POST', {
      x: parseFloat(x),
      y: parseFloat(y),
      z: parseFloat(z),
      pitch: parseFloat(pitch)
    });
  },

  async moveXYZ(x, y, z, pitch = 0.0, speed = 100) {
    return await this._request('/api/move_xyz', 'POST', {
      x: parseFloat(x),
      y: parseFloat(y),
      z: parseFloat(z),
      pitch: parseFloat(pitch),
      speed: parseInt(speed)
    });
  },

  // Sequence Endpoints
  async listSequences() {
    return await this._request('/api/sequences');
  },

  async saveSequence(name, steps) {
    return await this._request('/api/sequences', 'POST', {
      name: name,
      steps: steps
    });
  },

  async deleteSequence(name) {
    return await this._request(`/api/sequences/${encodeURIComponent(name)}`, 'DELETE');
  },

  async executeSequence(steps, speed = 100, delaySeconds = 1.0) {
    return await this._request('/api/poses/sequence', 'POST', {
      steps: steps,
      speed: parseInt(speed),
      delay_seconds: parseFloat(delaySeconds)
    });
  },

  async executeSavedSequence(name) {
    return await this._request(`/api/sequences/${encodeURIComponent(name)}/execute`, 'POST');
  },

  // Trajectory Planning Endpoints (Phase 1)
  async validateTrajectory(waypoints, interpolation = 'quintic', sampleRate = 20.0, velocityLimits = null, accelerationLimits = null) {
    return await this._request('/api/trajectory/validate', 'POST', {
      waypoints: waypoints,
      interpolation: interpolation,
      sample_rate: parseFloat(sampleRate),
      velocity_limits: velocityLimits,
      acceleration_limits: accelerationLimits
    });
  },

  async previewTrajectory(waypoints, interpolation = 'quintic', sampleRate = 20.0, velocityLimits = null, accelerationLimits = null) {
    return await this._request('/api/trajectory/preview', 'POST', {
      waypoints: waypoints,
      interpolation: interpolation,
      sample_rate: parseFloat(sampleRate),
      velocity_limits: velocityLimits,
      acceleration_limits: accelerationLimits
    });
  },

  async executeTrajectory(waypoints, interpolation = 'quintic', sampleRate = 20.0, velocityLimits = null, accelerationLimits = null) {
    return await this._request('/api/trajectory/execute', 'POST', {
      waypoints: waypoints,
      interpolation: interpolation,
      sample_rate: parseFloat(sampleRate),
      velocity_limits: velocityLimits,
      acceleration_limits: accelerationLimits
    });
  },

  async cancelTrajectory() {
    return await this._request('/api/trajectory/cancel', 'POST');
  },

  async getTrajectoryStatus() {
    return await this._request('/api/trajectory/status');
  }
};

