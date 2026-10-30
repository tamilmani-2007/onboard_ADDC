import math

from utils.config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_HORIZONTAL_FOV, CAMERA_VERTICAL_FOV,
    FOCAL_LENGTH, SENSOR_WIDTH, SENSOR_HEIGHT,
    SIDE_OVERLAP, FRONT_OVERLAP, FLIGHT_ALTITUDE
)

from quad.state import get_state
state = get_state()


def _altitude(alt=None):
    if alt is None:
        alt = state.alt if state.alt is not None else FLIGHT_ALTITUDE
    return max(alt, 0.5)


def ground_width_height(alt=None):
    """Ground footprint (m) of the camera frame (down-facing) at the given altitude (default: live altitude)."""
    a = _altitude(alt)
    return (2 * a * math.tan(math.radians(CAMERA_HORIZONTAL_FOV) / 2),
            2 * a * math.tan(math.radians(CAMERA_VERTICAL_FOV) / 2))


def get_pixel_length(alt=None):
    """Metres on the ground per image pixel (x, y)."""
    gw, gh = ground_width_height(alt)
    return gw / CAMERA_WIDTH, gh / CAMERA_HEIGHT


def calculate_offset(x_pixels, y_pixels, alt=None):
    px, py = get_pixel_length(alt)
    return x_pixels * px, y_pixels * py
