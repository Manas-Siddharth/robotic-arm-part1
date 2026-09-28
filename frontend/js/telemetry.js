/**
 * Telemetry WebSocket Client for 5-DOF Dynamixel Robotic Arm.
 * Handles auto-reconnect, status notifications, and state callbacks.
 */

class TelemetryClient {
  constructor(onStateUpdate, onStatusChange) {
    const self = this;
    this.onStateUpdate = onStateUpdate;
    this.onStatusChange = onStatusChange;
    this.socket = null;
    this.connected = false;
    this.reconnectTimer = null;
    this.reconnectInterval = 2000;
  }

  connect() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host;
    const wsUrl = `${protocol}//${host}/ws/telemetry`;

    console.log(`[TelemetryClient] Connecting to ${wsUrl}...`);

    try {
      this.socket = new WebSocket(wsUrl);

      this.socket.onopen = () => {
        console.log('[TelemetryClient] WebSocket connected.');
        this.connected = true;
        if (this.onStatusChange) this.onStatusChange(true);
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = null;
        }
      };

      this.socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (this.onStateUpdate) {
            this.onStateUpdate(data);
          }
        } catch (err) {
          console.error('[TelemetryClient] Parse error:', err);
        }
      };

      this.socket.onclose = () => {
        console.warn('[TelemetryClient] WebSocket closed. Reconnecting...');
        this.connected = false;
        if (this.onStatusChange) this.onStatusChange(false);
        this.scheduleReconnect();
      };

      this.socket.onerror = (err) => {
        console.error('[TelemetryClient] WebSocket error:', err);
        this.socket.close();
      };
    } catch (e) {
      console.error('[TelemetryClient] Failed to establish connection:', e);
      this.scheduleReconnect();
    }
  }

  scheduleReconnect() {
    if (!this.reconnectTimer) {
      this.reconnectTimer = setTimeout(() => {
        this.reconnectTimer = null;
        this.connect();
      }, this.reconnectInterval);
    }
  }
}
