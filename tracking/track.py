"""
Tracking module for ADDC project (compatibility layer).

AprilTag/QR detection now lives in cam.vision (a single thread that owns the camera) and the tracking
state handlers in quad.operation. These names are kept so old imports keep working.
"""
from cam.runtime import start_vision as initialize_tracking, stop_vision as cleanup_tracking  # noqa: F401
from quad.operation import (  # noqa: F401
    handle_apriltag_tracking_state as process_apriltag_tracking,
     handle_qr_search_state as detect_and_decode_qr
 )
