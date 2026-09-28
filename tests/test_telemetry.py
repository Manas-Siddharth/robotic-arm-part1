"""Tests for Telemetry system state generation and safety integration."""

import pytest
from backend.main import app, telemetry_manager, safety_monitor

def test_telemetry_manager_state():
    state = telemetry_manager.get_current_system_state()
    assert state.connected is True
    assert state.simulation is True
    assert len(state.servos) == 5

def test_telemetry_broadcast_cycle():
    # Verify telemetry manager state generation under different system states
    state = telemetry_manager.get_current_system_state()
    assert state.system_status in ["READY", "MOVING", "ESTOP", "FAULT"]
    assert state.watchdog_tripped is False

def test_telemetry_estop_state_reflection():
    safety_monitor.trigger_estop()
    state = telemetry_manager.get_current_system_state()
    assert state.fault_latched is True
    assert state.system_status == "ESTOP"

    # Clean up safety monitor state after test
    safety_monitor.clear_fault()
    cleared_state = telemetry_manager.get_current_system_state()
    assert cleared_state.fault_latched is False
    assert cleared_state.system_status in ["READY", "MOVING"]
