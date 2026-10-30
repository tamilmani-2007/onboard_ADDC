#!/usr/bin/env python3
"""Show where the AprilTag is relative to the drone (camera down, image-up = nose). No MAVLink, no flight."""
import sys, time
from cam.runtime import start_vision, stop_vision, get_vision
from utils.config import CAMERA_WIDTH as W, CAMERA_HEIGHT as H, CAMERA_OFFSET_FORWARD_M
from utils.utils import get_pixel_length
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60
start_vision(stream=False); v = get_vision(); v.set_mode("apriltag")
t0 = time.time(); last = None
try:
    while time.time() - t0 < secs:
        time.sleep(0.4)
        tag = v.get_tag(0.4)
        if tag is None:
            if last != "none": print(f"{time.time()-t0:5.1f}s  no tag", flush=True); last = "none"
            continue
        cx, cy = tag["center"]; mx, my = get_pixel_length(1.0)
        right = (cx - W/2) * mx; fwd = -((cy - H/2) * my - CAMERA_OFFSET_FORWARD_M*0)   # raw, at 1 m height, no offset
        h = "RIGHT" if cx > W/2 + 20 else "LEFT" if cx < W/2 - 20 else "centre"
        f = "FORWARD(nose)" if cy < H/2 - 20 else "BACK(tail)" if cy > H/2 + 20 else "centre"
        last = "tag"
        print(f"{time.time()-t0:5.1f}s  id {tag['id']:2d}  px ({cx:3.0f},{cy:3.0f})  -> drone should move: {h:6s} / {f}   [{v.fps:.0f} fps]", flush=True)
finally:
    stop_vision()
