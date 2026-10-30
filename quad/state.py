"""
the object "state" created for the DroneState is created for sharing
informations of Drone state simultaneously over other classes
"""
from typing import List, Set
from collections import defaultdict

class DroneState:
    def __init__(self):
        self.lat = None
        self.lon = None
        self.alt = None
        self.heading = None
        self.speed = None
        self.armed = None
        self.mode = None
        self.connected = False
        self.heartbeat = False
        self.is_op_completed = False
        self.addc_mission = True
        self.decoded_data = None
        self.gps_ready = False
        self.gps_fix = 0             # GPS_RAW_INT.fix_type (3 = 3D fix)
        self.sats = 0
        self.battery_v = None
        self.battery_pct = None
        self.acks = {}               # MAV_CMD id -> MAV_RESULT, filled by the telemetry thread
        # Pilot override: main.py sets allowed_modes once the vehicle is armed in GUIDED. If the flight mode is
        # ever outside it while armed (pilot flipped the RC mode switch), pilot_override latches True and
        # nothing in this program sends a mode change or a velocity command any more.
        self.rc_chancount = None      # RC_CHANNELS.chancount (0 = no receiver data)
        self.rc_channels = None       # [chan1..chan8] raw PWM
        self.allowed_modes = None
        self.pilot_override = False

state = DroneState()

def get_state():
    return state
