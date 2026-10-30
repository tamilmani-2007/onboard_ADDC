"""
Mission State Machine for ADDC project.
Implements the finite state machine for the autonomous mission.
"""

from enum import Enum, auto
from utils.logger import logger
from utils.config import (
    MISSION_TIMEOUT, HOVER_TIME,
    DRY_RUN
)
import time


class MissionState(Enum):
    """Mission states for the ADDC autonomous mission"""
    INITIALIZING = auto()
    MAVLINK_CONNECTED = auto()
    READY = auto()
    TAKEOFF = auto()
    APRILTAG_SEARCH = auto()
    HUMAN_TRACKING = auto()
    QR_SEARCH = auto()
    QR_DETECTED = auto()
    HOVER = auto()
    DYNAMIC_RTL = auto()
    LAND = auto()
    COMPLETE = auto()
    EMERGENCY = auto()


class MissionManager:
    def __init__(self):
        self.current_state = MissionState.INITIALIZING
        self.previous_state = None
        self.state_start_time = time.time()
        self.mission_start_time = None
        self.last_apriltag_detection_time = None
        self.last_video_frame_time = None
        self.last_mavlink_heartbeat_time = None
        self.qr_result = None
        self.home_position = None  # Will be set when GPS is ready

    def update_state(self, new_state):
        """Transition to a new state"""
        if new_state != self.current_state:
            logger.info(f"MISSION STATE TRANSITION: {self.current_state.name} -> {new_state.name}")
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()

            # Record mission start time
            if new_state == MissionState.TAKEOFF and self.mission_start_time is None:
                self.mission_start_time = time.time()

    def get_time_in_state(self):
        """Get time spent in current state"""
        return time.time() - self.state_start_time

    def get_mission_time(self):
        """Get total mission time"""
        if self.mission_start_time is None:
            return 0
        return time.time() - self.mission_start_time

    def is_mission_timeout(self):
        """Check if mission has exceeded time limit"""
        return self.get_mission_time() > MISSION_TIMEOUT

    def is_apriltag_lost(self):
        """Check if AprilTag has been lost for too long"""
        if self.last_apriltag_detection_time is None:
            return False
        return (time.time() - self.last_apriltag_detection_time) > 5.0  # 5 second timeout

    def is_video_lost(self):
        """Check if video has been lost for too long"""
        if self.last_video_frame_time is None:
            return False
        return (time.time() - self.last_video_frame_time) > 2.0  # 2 second timeout

    def is_mavlink_lost(self):
        """Check if MAVLink connection has been lost"""
        if self.last_mavlink_heartbeat_time is None:
            return False
        return (time.time() - self.last_mavlink_heartbeat_time) > 3.0  # 3 second timeout

    def update_apriltag_detection(self):
        """Update last AprilTag detection time"""
        self.last_apriltag_detection_time = time.time()

    def update_video_frame(self):
        """Update last video frame time"""
        self.last_video_frame_time = time.time()

    def update_mavlink_heartbeat(self):
        """Update last MAVLink heartbeat time"""
        self.last_mavlink_heartbeat_time = time.time()

    def set_home_position(self, lat, lon, alt):
        """Set home position for RTL"""
        self.home_position = (lat, lon, alt)

    def get_state_info(self):
        """Get current state information for logging"""
        return {
            'state': self.current_state.name,
            'time_in_state': self.get_time_in_state(),
            'mission_time': self.get_mission_time(),
            'apriltag_lost': self.is_apriltag_lost(),
            'video_lost': self.is_video_lost(),
            'mavlink_lost': self.is_mavlink_lost()
        }


# Global mission manager instance
mission_manager = MissionManager()