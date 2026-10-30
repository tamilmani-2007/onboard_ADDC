"""
The Pi camera can be opened by ONE process only. fpv-video-ap.service / fpv-video.service (rpicam-vid) hold it
at boot, so while the onboard vision runs they are stopped (vision streams the H.264 video itself) and the
ones that were running are started again afterwards.
"""
import subprocess

from utils.logger import logger
from utils.config import MANAGE_VIDEO_SERVICES, VIDEO_SERVICES

_stopped = []


def _active(unit):
    return subprocess.run(["systemctl", "is-active", "--quiet", unit]).returncode == 0


def release_camera():
    if not MANAGE_VIDEO_SERVICES:
        return
    for unit in VIDEO_SERVICES:
        if _active(unit):
            logger.info(f"Stopping {unit} (it holds the camera)")
            subprocess.run(["sudo", "-n", "systemctl", "stop", unit], check=False)
            _stopped.append(unit)


def restore_camera():
    while _stopped:
        unit = _stopped.pop()
        logger.info(f"Restarting {unit}")
        subprocess.run(["sudo", "-n", "systemctl", "start", unit], check=False)
