"""
Owns the single VisionThread. Kept free of any MAVLink import so the camera can be tested on its own.
"""
from utils.logger import logger
from cam.vision import VisionThread
from cam import video_service

_vision = None


def get_vision():
    return _vision


def start_vision(stream=True) -> bool:
    """Free the camera from the rpicam-vid services, open it and start detection (+ optional GCS video)."""
    global _vision
    if _vision is not None:
        return True
    video_service.release_camera()
    vt = VisionThread(stream=stream)
    vt.start()
    vt.ready.wait(timeout=15)
    if vt.failed or not vt.ready.is_set():
        logger.error("Vision failed to start")
        video_service.restore_camera()
        return False
    _vision = vt
    logger.info("Vision started")
    return True


def stop_vision():
    global _vision
    if _vision is not None:
        _vision.stop()
        _vision.join(timeout=5)
        _vision = None
    video_service.restore_camera()
