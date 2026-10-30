import sys
import time
from pymavlink import mavutil
from utils.logger import logger
from utils.config import MAVLINK_HEARTBEAT_WAIT
from quad.const import connection_type, tcp_address, device, baud, port

master = None

# GLOBAL_POSITION_INT, GPS_RAW_INT, SYS_STATUS, VFR_HUD at 5 Hz (ArduPilot only sends what is requested)
_INTERVALS_US = {33: 200000, 24: 200000, 1: 500000, 74: 200000, 65: 500000}   # 65 = RC_CHANNELS


def _open():
    if connection_type == "tcp":
        # Through fc-tcp-bridge.service: it is the single owner of /dev/ttyS0.
        logger.info(f"Connecting to flight controller via bridge tcp:{tcp_address}")
        return mavutil.mavlink_connection(f"tcp:{tcp_address}", source_system=255, source_component=191)
    if connection_type == "serial":
        logger.info(f"Connecting to flight controller over serial: {device} @ {baud}")
        return mavutil.mavlink_connection(device, baud=baud, source_system=255, source_component=191)
    conn = f"udp:127.0.0.1:{port}"
    logger.info(f"Connecting to flight controller over UDP: {conn}")
    return mavutil.mavlink_connection(conn, source_system=255, source_component=191)


def _wait_autopilot(m, timeout):
    """Wait for a heartbeat from the autopilot itself (ignore other GCS clients on the bridge)."""
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb is not None and hb.type != mavutil.mavlink.MAV_TYPE_GCS and hb.get_srcSystem() != 255:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return True
        logger.warning("Waiting for autopilot heartbeat...")
    return False


def get_master():
    global master

    if master is None:
        try:
            m = _open()
        except Exception as e:
            hint = ""
            if connection_type == "tcp":
                hint = " (is fc-tcp-bridge.service running?  sudo systemctl start fc-tcp-bridge)"
            elif connection_type == "serial":
                hint = (" (the UART is probably held by fc-tcp-bridge / fc-bridge - "
                        "use the default --connection tcp instead)")
            logger.error(f"Cannot open the MAVLink link: {e}{hint}")
            sys.exit(1)

        if not _wait_autopilot(m, MAVLINK_HEARTBEAT_WAIT):
            logger.error("No autopilot heartbeat - check the FC power / UART wiring / baud rate")
            sys.exit(1)
        logger.info(f"Autopilot found: system {m.target_system} component {m.target_component}")

        for msg_id, interval in _INTERVALS_US.items():
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                    msg_id, interval, 0, 0, 0, 0, 0)
        master = m

    return master
