#!/usr/bin/env python3
"""Camera + AprilTag/QR test. Never touches MAVLink or the motors.

    .venv/bin/python vision_test.py --bench          # detector speed on synthetic frames, no camera needed
    .venv/bin/python vision_test.py --mode apriltag  # live camera, prints detections + FPS (Ctrl-C to stop)
    .venv/bin/python vision_test.py --mode qr
    .venv/bin/python vision_test.py --mode apriltag --stream   # also send the H.264 video to the GCS

Live mode stops fpv-video-ap.service (it owns the camera) and starts it again on exit.
"""
import argparse
import time

import cv2 as cv
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--bench", action="store_true")
ap.add_argument("--mode", choices=["apriltag", "qr"], default="apriltag")
ap.add_argument("--stream", action="store_true")
ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds (0 = until Ctrl-C)")
args = ap.parse_args()


def synthetic_frame(kind, size=90, pos=(300, 200)):
    img = np.full((480, 640), 150, np.uint8)
    if kind == "apriltag":
        from utils import config as _cfg
        _dict = {"tag25h9": cv.aruco.DICT_APRILTAG_25h9, "tag36h11": cv.aruco.DICT_APRILTAG_36h11,
                 "tag16h5": cv.aruco.DICT_APRILTAG_16h5}[_cfg.APRILTAG_FAMILY]
        tag = cv.aruco.generateImageMarker(cv.aruco.getPredefinedDictionary(_dict), 3, size)
        tag = cv.copyMakeBorder(tag, 12, 12, 12, 12, cv.BORDER_CONSTANT, value=255)
    else:
        tag = cv.QRCodeEncoder.create().encode("ADDC-TEST-123")
        tag = cv.resize(tag, (size + 40, size + 40), interpolation=cv.INTER_NEAREST)
        tag = cv.copyMakeBorder(tag, 12, 12, 12, 12, cv.BORDER_CONSTANT, value=255)
    h, w = tag.shape
    img[pos[1]:pos[1] + h, pos[0]:pos[0] + w] = tag
    return cv.GaussianBlur(img, (3, 3), 0)


if args.bench:
    from cam.vision import AprilTagFinder
    from cam.qr_decoder import QRReader
    f, q = AprilTagFinder(), QRReader()
    img = synthetic_frame("apriltag")
    for label, reset in (("AprilTag full frame (decimate 2)", True), ("AprilTag ROI tracking", False)):
        f.reset()
        r = f.find(img)
        t = time.perf_counter(); n = 60
        for _ in range(n):
            if reset:
                f.reset()
            r = f.find(img)
        dt = (time.perf_counter() - t) / n
        print(f"{label:34s} {dt*1000:6.1f} ms  ({1/dt:5.0f} fps)  found={r is not None and r['id']}  "
              f"center={None if r is None else tuple(round(c) for c in r['center'])}")
    img = synthetic_frame("qr")
    t = time.perf_counter(); n = 30
    for _ in range(n):
        text, pts = q.read(img)
    dt = (time.perf_counter() - t) / n
    print(f"{'QR detect+decode':34s} {dt*1000:6.1f} ms  ({1/dt:5.0f} fps)  text={text!r}")
    raise SystemExit

from cam.runtime import start_vision, stop_vision, get_vision  # noqa: E402  (no MAVLink involved)

if not start_vision(stream=args.stream):
    raise SystemExit("camera did not start")
try:
    get_vision().set_mode(args.mode)
    t0, last = time.time(), None
    while args.seconds == 0 or time.time() - t0 < args.seconds:
        time.sleep(0.5)
        v = get_vision()
        res = v.get_tag(0.5) if args.mode == "apriltag" else v.get_qr(1.0)
        desc = "none" if res is None else (f"id {res['id']} center ({res['center'][0]:.0f},{res['center'][1]:.0f})"
                                           if args.mode == "apriltag" else f"text {res['text']!r}")
        print(f"{time.strftime('%H:%M:%S')}  {v.fps:5.1f} fps  frame age {v.frame_age()*1000:4.0f} ms  {args.mode}: {desc}",
              flush=True)
except KeyboardInterrupt:
    pass
finally:
    stop_vision()
