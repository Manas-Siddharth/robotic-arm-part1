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
      if (!resp.ok) {
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
  }
};
