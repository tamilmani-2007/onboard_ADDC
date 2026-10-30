from typing import List
import argparse

from utils.config import (
    MAVLINK_CONNECTION, MAVLINK_TCP, MAVLINK_SERIAL_DEVICE, MAVLINK_BAUDRATE, MAVLINK_UDP_PORT,
    TAKEOFF_ALTITUDE, STREAM_ENABLE, MAX_ALTITUDE,
)

parser = argparse.ArgumentParser()

parser.add_argument("--connection",
                     type=str,
                     default=MAVLINK_CONNECTION,
                     choices=["tcp", "serial", "udp"],
                     help="MAVLink transport to the CUAV V5+: 'tcp' (via fc-tcp-bridge, default), "
                          "'serial' (open the UART directly) or 'udp' (bench/SITL)",
                     )
parser.add_argument("--tcp",
                     type=str,
                     default=MAVLINK_TCP,
                     help="host:port of the fc-tcp-bridge",
                     )
parser.add_argument("--device",
                     type=str,
                     default=MAVLINK_SERIAL_DEVICE,
                     help="Serial device for the flight controller UART (serial mode only)",
                     )
parser.add_argument("--baud",
                     type=int,
                     default=MAVLINK_BAUDRATE,
                     help="Baud rate for the serial MAVLink connection",
                     )
parser.add_argument("--port",
                     type=int,
                     default=MAVLINK_UDP_PORT,
                     help="UDP port for the MAVLink connection (bench/SITL only)",
                     )
parser.add_argument("--alt",
                     type=float,
                     default=TAKEOFF_ALTITUDE,
                     help="Takeoff altitude in metres",
                     )
parser.add_argument("--auto-arm",
                     action="store_true",
                     help="Arm the vehicle from the script. Without it the pilot arms (RC) and the script waits.",
                     )
parser.add_argument("--no-stream",
                     action="store_true",
                     help="Do not stream H.264 video to the GCS (detection only)",
                     )
# parse_known_args: never crash on flags meant for other scripts
args, _unknown = parser.parse_known_args()
if not 0.5 <= args.alt <= MAX_ALTITUDE:
    parser.error(f"--alt must be between 0.5 and {MAX_ALTITUDE} m (safety ceiling, utils/config.py MAX_ALTITUDE)")

connection_type = args.connection
tcp_address = args.tcp
device = args.device
baud = args.baud
port = args.port
takeoff_alt = args.alt
auto_arm = args.auto_arm
stream_enabled = STREAM_ENABLE and not args.no_stream
tag = (-35.3616961, 149.1636683)
