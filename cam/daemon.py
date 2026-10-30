"""
Camera daemon (cam-daemon.service): the ONLY process that opens the Pi camera.

  camera -> H.264 encoder -> gst-launch -> RTP/UDP to the GCS (WFB-ng 127.0.0.1:5602, or the hotspot laptop)
         -> gray detection frames -> /dev/shm/drone_cam  (read by the mission's vision thread, cam/shm.py)

Video therefore never pauses when a mission starts, stops or crashes. The stream target is chosen at start from
the current link mode (utils.config.stream_target); drone-mode restarts this service when the mode changes.
Without a camera it just retries every 5 s.
"""
import os
import signal
import threading
import time

from utils.logger import logger
from utils import config as cfg
from cam.capture import Picamera2Capture
from cam.shm import ShmPublisher, PATH

_stop = False
_stop_at = 0.0
_running = False     # camera open and publishing
_last_pub = time.time()
FRAME_STALL_S = 5.0  # no frame for this long while running -> exit (systemd restarts the daemon)
STOP_GRACE_S = 3.0   # after SIGTERM, force-exit if the camera shutdown hangs


def _handle(_sig, _frm):
    global _stop, _stop_at
    _stop, _stop_at = True, time.time()


def _watchdog():
    """The camera pipeline can wedge (capture_array / stop_recording block forever). Do not rely on the main
    loop: kill the process so systemd restarts it with a fresh camera. Also restarts if the shm file vanishes."""
    while True:
        time.sleep(1)
        if _stop and time.time() - _stop_at > STOP_GRACE_S:
            logger.error("Shutdown hung - forcing exit")
            os._exit(0)
        if _running and time.time() - _last_pub > FRAME_STALL_S:
            logger.error(f"No camera frame for {FRAME_STALL_S:.0f} s - restarting daemon")
            os._exit(1)
        if not os.path.exists(PATH):
            logger.error("Shared-memory frame file disappeared - restarting daemon")
            os._exit(1)


def main():
    signal.signal(signal.SIGTERM, _handle)
    signal.signal(signal.SIGINT, _handle)
    global _running, _last_pub
    w, h = cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT
    stream = None
    if cfg.STREAM_ENABLE:
        host, port = cfg.stream_target()
        stream = dict(host=host, port=port, size=cfg.STREAM_SIZE, bitrate=cfg.STREAM_BITRATE)
    pub = ShmPublisher(w, h)
    threading.Thread(target=_watchdog, name="Camera Watchdog", daemon=True).start()
    warned = False
    while not _stop:
        try:
            cap = Picamera2Capture(size=(w, h), fps=cfg.CAMERA_FPS, stream=stream)
        except Exception as e:  # noqa: BLE001
            if not warned:
                logger.error(f"Camera not available ({e}) - retrying every 5 s")
                warned = True
            for _ in range(50):
                if _stop:
                    break
                time.sleep(0.1)
            continue
        warned = False
        logger.info(f"Camera daemon running, stream -> {stream and (stream['host'], stream['port'])}")
        fails = 0
        _last_pub = time.time()
        _running = True
        try:
            while not _stop:
                try:
                    ok, gray = cap.read_gray()
                except Exception as e:  # noqa: BLE001
                    logger.error(f"Camera read failed: {e}")
                    break
                if not ok:
                    fails += 1
                    if fails > 100:
                        break
                    time.sleep(0.01)
                    continue
                fails = 0
                pub.publish(gray)
                _last_pub = time.time()
        finally:
            _running = False
            cap.release()
            logger.info("Camera released")
    pub.close()


if __name__ == "__main__":
    main()
