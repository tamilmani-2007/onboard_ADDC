#!/usr/bin/env python3
"""Bench simulation of the whole mission: mock FC (tcp 5761, ~/mock_fc.py) + scripted fake vision (no camera).
A QR becomes visible after a few seconds of tracking; it decodes only once the drone is low enough.
    python simtest.py --tcp 127.0.0.1:5761 --auto-arm --alt 3 --no-stream
"""
import sys, time
from utils import config
config.HUMAN_TRACKING_TIME = 4
config.HOVER_TIME = 4
config.QR_DECODE_TIMEOUT = 15
config.QR_REPORT_PERIOD = 1.0
config.QR_RESULT_FILE = "/tmp/qr_result_sim.txt"
config.MISSION_TIMEOUT = float(__import__('os').environ.get('SIM_TIMEOUT', '120'))
import cam.runtime as runtime
from quad.state import state


class FakeVision:
    fps = 25.0
    def __init__(self): self.t0 = time.time(); self.mode = "idle"; self.decodes = []
    def set_mode(self, m): self.mode = m
    def frame_age(self):
        d = float(__import__('os').environ.get('SIM_VISION_DEAD', '0'))   # camera dies this many s after start
        return 99.0 if d and time.time() - self.t0 > d else 0.01
    def get_tag(self, max_age=0.5): return {"id": 7, "center": (330.0, 250.0), "corners": None}
    def _visible(self): return time.time() - self.t0 > 5
    def qr_located(self, min_hits=3, window=1.5): return self._visible()
    def get_qr(self, max_age=1.0):
        if not self._visible(): return None
        alt = state.alt or 0
        if alt <= 1.2: self.decodes.append(time.time())
        return {"text": "ADDC-SIM-QR-0042-HELLO-GCS-THIS-IS-A-LONG-MESSAGE" if alt <= 1.2 else "", "points": None,
                "center": (330.0, 240.0 + 0.1 / (2 * max(alt, .5) * 0.4538 / 480))}  # camera 10 cm ahead of FC
    def qr_confirmed(self, min_count=None, window=4.0):
        n = sum(1 for t in self.decodes if time.time() - t <= window)
        return "ADDC-SIM-QR-0042-HELLO-GCS-THIS-IS-A-LONG-MESSAGE" if n >= 3 else None


fake = FakeVision()
runtime.start_vision = lambda stream=True: setattr(runtime, "_vision", fake) or True
runtime.stop_vision = lambda: None
runtime._vision = fake
import main  # noqa: E402  (after patches)
import os
if os.environ.get('SIM_CEILING'):
    main.cfg.MAX_ALTITUDE = float(os.environ['SIM_CEILING'])   # test: pretend the ceiling is lower
main.main()
