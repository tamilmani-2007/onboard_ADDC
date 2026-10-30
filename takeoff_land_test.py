#!/usr/bin/env python3
"""Takeoff to --alt (default 1 m), hover briefly, land. Modes used: GUIDED (required by ArduCopter for
NAV_TAKEOFF) and LAND only.

Connects through the fc-tcp-bridge (tcp:127.0.0.1:5760) instead of opening /dev/ttyS0, so it
never fights another process for the serial port ("port already in use").

    ~/mav-venv/bin/python ~/onboard_ADDC/takeoff_land_test.py --dry-run   # checks only, never arms
    ~/mav-venv/bin/python ~/onboard_ADDC/takeoff_land_test.py             # real flight (asks to confirm)

Ctrl-C or any error while airborne -> LAND.

PILOT OVERRIDE: in GUIDED the sticks do nothing except yaw. To take over, flip the RC mode switch to a manual
mode (STABILIZE / ALT_HOLD / LOITER) - if the switch already sits in that position, move it to another
position and back first. The script watches the flight mode: as soon as it is not GUIDED/LAND it stops
commanding, never changes the mode again, and exits, so it can not pull the aircraft back from you.
"""
import argparse, sys, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
ap.add_argument("--alt", type=float, default=1.0)
ap.add_argument("--hover", type=float, default=3.0)
ap.add_argument("--dry-run", action="store_true")
ap.add_argument("--yes", action="store_true")
args = ap.parse_args()
if not 0.5 <= args.alt <= 2.0:
    sys.exit("--alt must be 0.5-2.0 m")


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


log(f"connecting {args.conn}")
m = mavutil.mavlink_connection(args.conn, source_system=255, source_component=190)
hb = None
t_end = time.time() + 15
while time.time() < t_end:  # ignore heartbeats from other GCS clients
    h = m.wait_heartbeat(timeout=2)
    if h and h.type != mavutil.mavlink.MAV_TYPE_GCS and m.target_system != 255:
        hb = h
        break
if hb is None:
    sys.exit("no autopilot heartbeat (is fc-tcp-bridge.service running?)")
log(f"autopilot sys {m.target_system} comp {m.target_component}")

S = {"alt": None, "fix": 0, "sats": 0, "acks": {}}
expect = None          # set of flight modes the script itself expects while airborne


class PilotOverride(Exception):
    pass


def pump(timeout=0.1):
    while True:
        msg = m.recv_match(blocking=True, timeout=timeout)
        timeout = 0
        if msg is None:
            return
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            S["alt"] = msg.relative_alt / 1000.0
        elif t == "GPS_RAW_INT":
            S["fix"], S["sats"] = msg.fix_type, msg.satellites_visible
        elif t == "COMMAND_ACK":
            S["acks"][msg.command] = msg.result
        elif t == "STATUSTEXT":
            log("FC: " + msg.text)


def wait(cond, timeout, what):
    end = time.time() + timeout
    while time.time() < end:
        pump()
        if expect and m.flightmode not in expect:
            raise PilotOverride(f"mode is now {m.flightmode}")
        if cond():
            return True
    log("timeout: " + what)
    return False


def cmd(c, *p, timeout=4):
    p = list(p) + [0] * (7 - len(p))
    S["acks"].pop(c, None)
    m.mav.command_long_send(m.target_system, m.target_component, c, 0, *p)
    wait(lambda: c in S["acks"], timeout, f"ack {c}")
    return S["acks"].get(c)


def set_mode(name):
    m.set_mode(m.mode_mapping()[name])
    return wait(lambda: m.flightmode == name, 5, f"mode {name}")


for mid in (33, 24, 1):  # GLOBAL_POSITION_INT, GPS_RAW_INT, SYS_STATUS
    cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 200000)
wait(lambda: S["alt"] is not None and S["fix"] > 0, 8, "telemetry")
log(f"mode {m.flightmode}, armed {bool(m.motors_armed())}, GPS fix {S['fix']} ({S['sats']} sats), alt {S['alt']}")

if m.motors_armed():
    sys.exit("already armed - refusing")
if S["fix"] < 3:
    sys.exit("no 3D GPS fix - refusing (GUIDED takeoff needs it)")
if abs(S["alt"]) > 2.0:
    sys.exit(f"altitude reads {S['alt']:.1f} m on the ground (barometer drift) - refusing: the 'altitude reached' "
             "check would pass immediately. Let the FC settle or reboot it.")
if args.dry_run:
    sys.exit("dry run OK")
if not args.yes and input(f"Take off to {args.alt} m and land? Props on, area clear. Type FLY: ") != "FLY":
    sys.exit("aborted")

airborne = False


def land():
    global expect
    if m.flightmode not in ("GUIDED", "LAND"):
        log(f"pilot has control ({m.flightmode}) - not touching the mode")
        return
    log("LAND")
    expect = {"GUIDED", "LAND"}      # tolerate the GUIDED->LAND transition itself
    set_mode("LAND")
    expect = {"LAND"}


try:
    if not set_mode("GUIDED"):
        raise RuntimeError("GUIDED rejected")
    if cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
        raise RuntimeError("arm rejected")
    airborne = True
    expect = {"GUIDED"}
    if not wait(lambda: m.motors_armed(), 5, "armed"):
        raise RuntimeError("not armed")
    if cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.alt) != 0:
        raise RuntimeError("takeoff rejected")
    log(f"taking off to {args.alt} m")
    if not wait(lambda: S["alt"] >= args.alt * 0.9, 20, "altitude"):
        raise RuntimeError("altitude not reached")
    log(f"reached {S['alt']:.2f} m, hover {args.hover}s")
    end = time.time() + args.hover
    wait(lambda: time.time() >= end, args.hover + 1, "hover")
    land()
    if wait(lambda: not m.motors_armed(), 60, "disarm after landing"):
        log("landed, disarmed")
    airborne = False
except PilotOverride as e:
    airborne = False
    log(f"PILOT OVERRIDE ({e}): script stops commanding and exits. The aircraft is yours.")
except KeyboardInterrupt:
    log("interrupted")
    if airborne:
        land()
except Exception as e:
    log(f"ERROR: {e}")
    if airborne:
        land()
finally:
    m.close()
