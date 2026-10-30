"""
Telemetry thread: the ONLY reader of the MAVLink connection.
Everything it receives goes into the shared `state` object (including COMMAND_ACKs,
which quad.py waits on), and it sends the 1 Hz companion heartbeat.
"""

import threading
import time

from pymavlink import mavutil
from utils.logger import logger
from quad import connection
from quad.state import get_state
from mission import mission_manager

state = get_state()
master = connection.get_master()


class TelemetryThread(threading.Thread):
    def __init__(self):
        super().__init__(name="Telemetry Thread", daemon=True)
        self.running = True
        self._last_hb = 0.0

    def stop(self):
        self.running = False

    def _send_heartbeat(self):
        if time.time() - self._last_hb < 1.0:
            return
        self._last_hb = time.time()
        from quad.quad import send_lock   # imported lazily: quad.quad imports connection too
        with send_lock:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def run(self):
        while self.running:
            self._send_heartbeat()
            msg = master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue

            if msg_type == "GLOBAL_POSITION_INT":
                state.lat = msg.lat / 1e7
                state.lon = msg.lon / 1e7
                state.alt = msg.relative_alt / 1000
                state.heading = msg.hdg / 100
                state.gps_ready = state.gps_fix >= 3

                # Update home position if not set and we have a real GPS fix
                if state.gps_ready and mission_manager.home_position is None:
                    mission_manager.set_home_position(state.lat, state.lon, state.alt)
                    logger.info(f"Home position set: {state.lat:.6f}, {state.lon:.6f}, {state.alt:.1f}m")

            elif msg_type == "GPS_RAW_INT":
                state.gps_fix = msg.fix_type
                state.sats = msg.satellites_visible

            elif msg_type == "SYS_STATUS":
                state.battery_v = msg.voltage_battery / 1000
                state.battery_pct = msg.battery_remaining

            elif msg_type == "HEARTBEAT":
                # Only the autopilot (other GCS clients on the bridge send heartbeats too)
                if msg.get_srcSystem() != master.target_system or msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue
                state.heartbeat = True
                state.mode = master.flightmode
                state.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                mission_manager.update_mavlink_heartbeat()
                if (state.allowed_modes and state.armed and state.mode not in state.allowed_modes
                        and not state.pilot_override):
                    state.pilot_override = True
                    logger.warning(f"PILOT OVERRIDE: mode is {state.mode} - all autonomous commands stopped")

            elif msg_type == "RC_CHANNELS":
                state.rc_chancount = msg.chancount
                state.rc_channels = [getattr(msg, f"chan{i}_raw") for i in range(1, 9)]

            elif msg_type == "VFR_HUD":
                state.speed = msg.groundspeed

            elif msg_type == "COMMAND_ACK":
                state.acks[msg.command] = msg.result

            elif msg_type == "STATUSTEXT":
                logger.info(f"FC: {msg.text}")
