import threading
import time
from pymavlink import mavutil
from quad import connection
from quad.state import state
from utils.logger import logger

master = connection.get_master()

# One lock for every write to the link (main thread commands + telemetry-thread heartbeat).
send_lock = threading.Lock()


def _command(cmd, *params, timeout=3.0, retries=2):
    """Send a COMMAND_LONG and wait for its ACK (filled in by the telemetry thread).
    Returns the MAV_RESULT, or None if nothing answered."""
    p = list(params) + [0] * (7 - len(params))
    for _ in range(retries):
        state.acks.pop(cmd, None)
        with send_lock:
            master.mav.command_long_send(master.target_system, master.target_component, cmd, 0, *p)
        end = time.time() + timeout
        while time.time() < end:
            res = state.acks.get(cmd)
            if res is not None:
                return res
            time.sleep(0.05)
    return None


def isFlying():
    return state.alt is not None and state.alt > 1.0


def arm(timeout=10.0) -> bool:
    res = _command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if res not in (None, mavutil.mavlink.MAV_RESULT_ACCEPTED):
        logger.error(f"Arm rejected (result {res}) - see the FC STATUSTEXT lines above")
        return False
    end = time.time() + timeout
    while time.time() < end:
        if state.armed:
            return True
        time.sleep(0.1)
    logger.error("Arm command not confirmed by heartbeat")
    return False


def disarm():
    if state.pilot_override:
        return None
    return _command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)


def takeoff(alt: float) -> bool:
    res = _command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
    if res not in (None, mavutil.mavlink.MAV_RESULT_ACCEPTED):
        logger.error(f"Takeoff rejected (result {res})")
        return False
    return True


def setmode(mode: str, timeout=5.0) -> bool:
    """Request a flight mode and wait until the heartbeat confirms it."""
    mode_id = master.mode_mapping()[mode]
    end = time.time() + timeout
    while time.time() < end:
        if state.pilot_override:
            logger.warning(f"Pilot has control ({state.mode}) - not requesting {mode}")
            return False
        if state.mode == mode:
            return True
        with send_lock:
            master.mav.set_mode_send(master.target_system,
                                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                     mode_id)
        time.sleep(0.5)
    logger.error(f"Mode {mode} not confirmed (still {state.mode})")
    return False


def send_velocity(vx, vy, vz=0):
    """Body-frame velocity set-point (forward, right, down) in m/s. Must be repeated at >= 2 Hz in GUIDED."""
    if state.pilot_override:
        return
    with send_lock:
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )


def send_statustext(text: str, severity=mavutil.mavlink.MAV_SEVERITY_INFO, prefix="QR"):
    """Show `text` on the GCS (Mission Planner message line / HUD). STATUSTEXT carries 50 bytes, so longer
    text goes out as numbered parts 'QR 1/2: ...'."""
    text = text.replace("\n", " ").strip()
    room = 50 - len(prefix) - 2
    parts = [text[i:i + room - 6] for i in range(0, len(text), room - 6)] if len(text) > room else [text]
    for n, part in enumerate(parts, 1):
        label = f"{prefix} {n}/{len(parts)}: " if len(parts) > 1 else f"{prefix}: "
        with send_lock:
            master.mav.statustext_send(severity, (label + part)[:50].encode("ascii", "replace"))
    logger.info(f"Sent to GCS: {text}")


def land():
    return setmode("LAND")
