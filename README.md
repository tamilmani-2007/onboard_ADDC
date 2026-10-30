# ADDC Autonomous Human Tracking System (Onboard)

Autonomous vision-guided UAV control system developed for the
SAEINDIA Southern Section Autonomous Drone Development Challenge (ADDC).

This is the **onboard** variant of the [ADDC](../ADDC) project. The system
implements the same mission, but the full perception, tracking, control,
QR processing, and mission-management stack runs **on the air vehicle's
Raspberry Pi 4** instead of on a ground-side laptop. There is no OpenHD
video/telemetry bridge in the loop: the camera is attached directly to the
Pi's CSI port and MAVLink runs over a direct UART link to the CUAV V5+.

The `ADDC` directory is left untouched — this is a parallel implementation
for onboard (air-side) processing.

---

## System Status

|       Subsystem         | Implementation              |
|-------------------------|------------------------------|
| Flight Controller       | CUAV V5+                     |
| Autopilot                | ArduPilot                    |
| Autonomous Computer      | Raspberry Pi 4 (onboard, air unit) |
| Camera                  | Global-shutter camera (CSI)   |
| Camera Interface        | libcamera / picamera2         |
| Visual Target            | AprilTag                     |
| Target Detection         | pupil_apriltags               |
| Image Processing         | OpenCV                       |
| Control                 | PID                           |
| Flight Communication     | MAVLink over UART (`/dev/ttyS0`) |
| Python Interface         | pymavlink                    |
| QR Detection             | OpenCV QRCodeDetector         |
| Mission Architecture     | Finite State Machine          |
| Processing Architecture | **Onboard**                   |
| Mission Constraint       | 5-minute mission              |

---

# 1. Overview

The mission objective is unchanged from the offboard project: the UAV
visually follows a human operative using an AprilTag as a dynamic
reference, searches for and decodes a QR code once the human phase is
complete, hovers to confirm the decode, and executes a dynamic
return-to-launch that accounts for remaining mission time before the
5-minute mission window expires.

What changes onboard is *where* the control loop executes. Instead of
streaming video and telemetry to a ground laptop over OpenHD and sending
velocity commands back down the link, every stage of the loop — frame
capture, AprilTag detection, image-space error calculation, PID control,
QR detection, mission state machine, and MAVLink commands — runs locally
on the Raspberry Pi 4 mounted on the airframe:

    CSI Camera (global shutter)
        |
        v
    Raspberry Pi 4 (onboard)
        |
        +--> AprilTag Detection (pupil_apriltags)
        |
        +--> Target Localization
        |
        +--> Image Error
        |
        +--> PID Controller
        |
        +--> Velocity Command
        |
        v
    MAVLink (UART, /dev/ttyS0)
        |
        v
    CUAV V5+ (ArduPilot)
        |
        v
    UAV Motion
        |
        +--------------------------+
                                   |
                                   v
                              New Camera Frame

This removes the ground-laptop link and the OpenHD video/MAVLink bridge
from the control loop entirely — the vision-to-velocity loop now closes
without leaving the vehicle. A ground control station may still observe
telemetry and the decoded QR result over a separate MAVLink/telemetry
radio link, but it is no longer in the control path.

---

# 2. Design Philosophy

Same separation of responsibilities as the offboard project:

    Perception
        |
        v
    State Estimation
        |
        v
    Decision / Mission Logic
        |
        v
    Control
        |
        v
    Vehicle Interface
        |
        v
    Flight Controller

ArduPilot remains the low-level flight-control authority; this software is
the high-level decision and velocity-command layer. The only architectural
change from the offboard project is the transport for camera frames
(direct CSI/libcamera instead of a network video stream) and for MAVLink
(direct UART instead of a UDP link relayed through a ground OpenHD bridge).

---

# 3. Onboard System Architecture

```text
                                  AIR VEHICLE
    +------------------------------------------------------------------+
    |                                                                  |
    |                     Global Shutter Camera (CSI)                 |
    |                              |                                   |
    |                              v                                   |
    |                    +-------------------+                         |
    |                    |   Raspberry Pi 4  |                         |
    |                    |  ONBOARD COMPUTER |                         |
    |                    |                   |                         |
    |                    | libcamera/picamera2|                        |
    |                    | AprilTag Detection |                        |
    |                    | QR Detection       |                        |
    |                    | PID Controller     |                        |
    |                    | Mission State      |                        |
    |                    | Machine            |                        |
    |                    | pymavlink          |                        |
    |                    +---------+---------+                         |
    |                              |                                   |
    |                              | UART (/dev/ttyS0, MAVLink)        |
    |                              v                                   |
    |                    +-------------------+                         |
    |                    |      CUAV V5+     |                         |
    |                    |    Flight Control |                         |
    |                    |                   |                         |
    |                    |     ArduPilot     |                         |
    |                    +---------+---------+                         |
    |                              |                                   |
    |                              v                                   |
    |                          ESC / Motors                            |
    |                                                                  |
    +------------------------------+-----------------------------------+
                                   |
                                   | Telemetry radio (monitoring only,
                                   | not in the control loop)
                                   v
                             Ground Control Station
```

---

# 4. Hardware / Environment Target

This directory is built and run against the actual onboard computer:

- **Board**: Raspberry Pi 4 (aarch64), 4 cores, Ubuntu 24.04 LTS (Noble)
- **Python**: 3.12 (system `python3`)
- **Camera**: CSI global-shutter camera on `unicam` (requires the
  `libcamera`/`bcm2835-isp` pipeline — plain V4L2 reads of `/dev/video0`
  return raw, undemosaiced sensor data, so frame capture goes through
  `picamera2`, not `cv2.VideoCapture`)
- **Flight controller link**: direct UART at `/dev/ttyS0`, 921600 baud

---

# 5. Environment Setup (uv)

Same tooling as the offboard `ADDC` project — [uv](https://docs.astral.sh/uv/)
manages the Python version and virtual environment, pinned via
`.python-version` and `pyproject.toml`.

Because `picamera2` depends on the system `libcamera` Python bindings
(installed via `apt`, matched to the kernel/ISP ABI — there is no portable
PyPI wheel for them), the venv is created with `--system-site-packages` so
it can see the apt-installed `libcamera` module alongside the uv-managed
dependencies.

```bash
# One-time system packages (apt) for the camera stack:
sudo apt-get install -y python3-libcamera python3-prctl

# Create the uv-managed venv with access to the system libcamera bindings:
uv venv --system-site-packages
uv sync

# Run the mission:
uv run main.py --connection serial --device /dev/ttyS0 --baud 921600
```

For bench-testing against SITL (no flight controller attached) from this
same Pi or another machine:

```bash
uv run main.py --connection udp --port 14551
```

---

# 6. Key Differences from `ADDC` (Offboard)

| Aspect              | Offboard (`ADDC`)                     | Onboard (`onboard_ADDC`)                  |
|---------------------|----------------------------------------|---------------------------------------------|
| Processing location | Ground laptop                          | Raspberry Pi 4 (air unit)                    |
| Camera transport    | OpenHD video stream over UDP           | Direct CSI capture via `picamera2`           |
| MAVLink transport    | UDP, relayed through OpenHD/ground Pi  | Direct UART (`/dev/ttyS0`) to CUAV V5+       |
| Ground Pi role       | Video/telemetry bridge                 | Not required for the control loop            |
| Config defaults      | `VIDEO_SOURCE = "udp"`                 | `VIDEO_SOURCE = "picamera2"`                 |
| Config defaults      | MAVLink over `udp:127.0.0.1:<port>`    | MAVLink over `serial:/dev/ttyS0@921600`      |

Mission logic, state machine, PID control, and QR/AprilTag detection code
are otherwise the same as the offboard project — only the I/O boundaries
(camera and MAVLink transport) changed.

---

# Running onboard (bridge + shared camera)

* **MAVLink** goes through `fc-tcp-bridge.service` (`tcp:127.0.0.1:5760`), the only process that opens
  `/dev/ttyS0`. Never open the UART directly while the bridge runs ("port already in use").
  `--connection serial` exists only for when no bridge is running.
* **Camera** (Pi Camera V2): one process owns it. `cam/vision.py` captures a grayscale frame (Y plane of a
  YUV420 stream), runs AprilTag (full-frame decimate 2, then ROI tracking) or QR detection, and streams H.264
  to the GCS from the same capture. `fpv-video-ap.service` is stopped for the run and restarted afterwards.
* Tunables: `utils/config.py` (detector speed/strictness, stream host, FOV, controller gains).

```
cd ~/onboard_ADDC
.venv/bin/python vision_test.py --bench                  # detector speed, no camera
.venv/bin/python vision_test.py --mode apriltag          # live camera (also --mode qr, --stream)
.venv/bin/python takeoff_land_test.py --dry-run          # 1 m takeoff/land checks, never arms
.venv/bin/python main.py --alt 7                         # mission; the pilot arms on the RC
.venv/bin/python main.py --alt 7 --auto-arm              # mission; the script arms (props on, area clear!)
```
Bench test without a flight controller: `~/mock_fc.py` fakes an autopilot on tcp 5761 →
`main.py --tcp 127.0.0.1:5761 --auto-arm --alt 2 --no-stream`.
