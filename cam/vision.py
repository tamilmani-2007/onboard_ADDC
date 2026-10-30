"""
Onboard vision: one thread owns the camera and runs AprilTag / QR detection on the newest frame.

The flight loop never waits on the camera or the detector - it just asks for the latest result:

    vision.set_mode("apriltag")      # or "qr" / "idle"  (only the active detector costs CPU)
    tag = vision.get_tag(max_age=0.5)
    qr  = vision.get_qr(max_age=1.0)
"""
import collections
import threading
import time

import numpy as np

from utils.logger import logger
from utils import config as cfg
from cam.capture import open_capture
from cam.qr_decoder import QRReader


class AprilTagFinder:
    """pupil_apriltags with two detectors:
       * full frame at quad_decimate=2 (cheap search),
       * once a tag is known: a small ROI around it at full resolution (cheaper still, more accurate)."""

    def __init__(self):
        from pupil_apriltags import Detector
        common = dict(families=cfg.APRILTAG_FAMILY, nthreads=cfg.APRILTAG_NTHREADS,
                      quad_sigma=cfg.APRILTAG_QUAD_SIGMA, refine_edges=cfg.APRILTAG_REFINE_EDGES,
                      decode_sharpening=cfg.APRILTAG_DECODE_SHARPENING, debug=cfg.APRILTAG_DEBUG)
        self._full = Detector(quad_decimate=cfg.APRILTAG_QUAD_DECIMATE, **common)
        self._roi_det = Detector(quad_decimate=cfg.APRILTAG_ROI_QUAD_DECIMATE, **common)
        self._roi = None  # (x0, y0, x1, y1)

    @staticmethod
    def _best(results):
        """tag25h9 has a high false-positive rate on clutter: keep only confident detections."""
        ok = [d for d in results
              if d.decision_margin >= cfg.APRILTAG_MIN_MARGIN and d.hamming <= cfg.APRILTAG_MAX_HAMMING
              and (not cfg.APRILTAG_IDS or d.tag_id in cfg.APRILTAG_IDS)]
        return max(ok, key=lambda d: d.decision_margin) if ok else None

    def reset(self):
        self._roi = None

    def find(self, gray):
        """Return dict(id, center=(x, y), corners=4x2) of the best tag, or None."""
        h, w = gray.shape[:2]
        det, ox, oy = None, 0, 0
        if self._roi is not None:
            x0, y0, x1, y1 = self._roi
            res = self._roi_det.detect(np.ascontiguousarray(gray[y0:y1, x0:x1]))
            det, ox, oy = self._best(res), x0, y0
        if det is None:
            det, ox, oy = self._best(self._full.detect(gray)), 0, 0
        if det is None:
            self._roi = None
            return None

        corners = det.corners + np.array([ox, oy])
        center = (float(det.center[0] + ox), float(det.center[1] + oy))
        side = max(np.ptp(corners[:, 0]), np.ptp(corners[:, 1]))
        pad = side * cfg.APRILTAG_ROI_PAD
        self._roi = (int(max(corners[:, 0].min() - pad, 0)), int(max(corners[:, 1].min() - pad, 0)),
                     int(min(corners[:, 0].max() + pad, w)), int(min(corners[:, 1].max() + pad, h)))
        return {"id": int(det.tag_id), "center": center, "corners": corners}


class VisionThread(threading.Thread):
    def __init__(self, stream=False):
        super().__init__(name="Vision Thread", daemon=True)
        self.running = False
        self.mode = "idle"
        self._stream = stream
        self._lock = threading.Lock()
        self._tag = None
        self._tag_time = 0.0
        self._qr = None
        self._qr_time = 0.0
        self._qr_seen = collections.deque(maxlen=20)      # (time, cx, cy) of plausible locates
        self._qr_decodes = collections.deque(maxlen=20)   # (time, text) of successful decodes
        self.last_frame_time = 0.0
        self.fps = 0.0
        self.ready = threading.Event()
        self.failed = False
        self._cap = None
        self._frame = None
        self._frame_no = 0
        self._frame_cv = threading.Condition()

    # ---- public API -------------------------------------------------------------------------------
    def set_mode(self, mode):
        if mode not in ("idle", "apriltag", "qr", "both"):
            raise ValueError(mode)
        self.mode = mode

    def get_tag(self, max_age=0.5):
        with self._lock:
            if self._tag is not None and time.time() - self._tag_time <= max_age:
                return self._tag
        return None

    def get_qr(self, max_age=1.0):
        with self._lock:
            if self._qr is not None and time.time() - self._qr_time <= max_age:
                return self._qr
        return None

    @staticmethod
    def _plausible_qr(pts, frame_shape):
        """Reject clutter the locator mistakes for a QR: must be a roughly square quad of sane size."""
        if pts is None or len(pts) != 4:
            return False
        p = np.asarray(pts, dtype=np.float32).reshape(4, 2)
        sides = [float(np.linalg.norm(p[i] - p[(i + 1) % 4])) for i in range(4)]
        if min(sides) < cfg.QR_MIN_SIDE_PX or max(sides) > 0.9 * min(frame_shape[:2]):
            return False
        if min(sides) / max(sides) < 0.5:
            return False
        x, y = p[:, 0], p[:, 1]      # shoelace area vs. bounding box: rejects bow-ties / slivers
        area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
        return area >= 0.4 * max(sides) * min(sides)

    def qr_located(self, min_hits=3, window=1.5):
        """True once a plausible QR has been LOCATED at (nearly) the same place in at least min_hits frames
        within the last `window` s. Does not need a successful decode."""
        now = time.time()
        with self._lock:
            hits = [(cx, cy) for t, cx, cy in self._qr_seen if now - t <= window]
        if len(hits) < min_hits:
            return False
        xs, ys = [h[0] for h in hits], [h[1] for h in hits]
        return (max(xs) - min(xs)) <= cfg.QR_STABLE_PX and (max(ys) - min(ys)) <= cfg.QR_STABLE_PX

    def qr_confirmed(self, min_count=None, window=4.0):
        """Decoded text seen at least min_count times (identical) within `window` s, else None.
        Guards against a single misread."""
        min_count = min_count or cfg.QR_CONFIRM_COUNT
        now = time.time()
        with self._lock:
            texts = [t for ts, t in self._qr_decodes if now - ts <= window]
        if not texts:
            return None
        text, count = collections.Counter(texts).most_common(1)[0]
        return text if count >= min_count else None

    def frame_age(self):
        return time.time() - self.last_frame_time if self.last_frame_time else float("inf")

    def stop(self):
        self.running = False

    # ---- threads ----------------------------------------------------------------------------------
    # Capture thread (this one): owns the camera, publishes the newest gray frame, and (through the capture
    # object) feeds the H.264 video forwarder. AprilTag and QR run in their OWN worker threads and each always
    # works on the newest frame, so a slow QR decode can never delay tag tracking or the video.
    def _publish(self, gray, now):
        with self._frame_cv:
            self._frame_no += 1
            self._frame = (self._frame_no, gray, now)
            self._frame_cv.notify_all()

    def _next_frame(self, last_no):
        """Block (<=0.2 s) for a frame newer than last_no; None if there is none (or we are stopping)."""
        with self._frame_cv:
            if self.running and (self._frame is None or self._frame[0] == last_no):
                self._frame_cv.wait(timeout=0.2)
            if self._frame is None or self._frame[0] == last_no:
                return None
            return self._frame

    def _tag_worker(self):
        tags, prev_mode, last_no = AprilTagFinder(), "idle", 0
        while self.running:
            f = self._next_frame(last_no)
            if f is None:
                continue
            last_no, gray, ts = f
            mode = self.mode
            if mode != prev_mode:
                tags.reset()
                prev_mode = mode
            if mode not in ("apriltag", "both"):
                continue
            try:
                res = tags.find(gray)
            except Exception as e:  # noqa: BLE001
                logger.error(f"AprilTag error: {e}")
                continue
            if res:
                with self._lock:
                    self._tag, self._tag_time = res, ts

    def _qr_worker(self):
        qr, last_no, last_done = QRReader(), 0, 0
        while self.running:
            f = self._next_frame(last_no)
            if f is None:
                continue
            last_no, gray, ts = f
            mode = self.mode
            if mode not in ("qr", "both"):
                continue
            # "both": QR located on at most every 3rd frame (CPU budget shared with AprilTag); "qr": every frame
            if mode == "both" and last_no - last_done < 3:
                continue
            last_done = last_no
            try:
                text, pts = qr.read(gray)
                if pts is not None and (text or self._plausible_qr(pts, gray.shape)):
                    c = pts.mean(axis=0)
                    with self._lock:
                        self._qr = {"text": text or "", "points": pts, "center": (float(c[0]), float(c[1]))}
                        self._qr_time = ts
                        self._qr_seen.append((ts, float(c[0]), float(c[1])))
                        if text:
                            self._qr_decodes.append((ts, text))
            except Exception as e:  # noqa: BLE001
                logger.error(f"QR error: {e}")

    def run(self):
        stream = None
        if self._stream:
            host, port = cfg.stream_target()
            stream = dict(host=host, port=port, size=cfg.STREAM_SIZE, bitrate=cfg.STREAM_BITRATE)
        self._cap = open_capture(cfg.VIDEO_SOURCE, cfg.VIDEO_SOURCE_PATH, cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT,
                                 fps=cfg.CAMERA_FPS, stream=stream)
        if self._cap is None:
            self.failed = True
            self.ready.set()
            return

        self.running = True
        workers = [threading.Thread(target=self._tag_worker, name="AprilTag Thread", daemon=True),
                   threading.Thread(target=self._qr_worker, name="QR Thread", daemon=True)]
        for w in workers:
            w.start()
        self.ready.set()
        n, t_fps = 0, time.time()
        try:
            while self.running:
                ok, gray = self._cap.read_gray()
                if not ok:
                    time.sleep(0.01)
                    continue
                now = time.time()
                self.last_frame_time = now
                n += 1
                if now - t_fps >= 2.0:
                    self.fps, n, t_fps = n / (now - t_fps), 0, now
                self._publish(gray, now)
        finally:
            self.running = False
            with self._frame_cv:
                self._frame_cv.notify_all()
            for w in workers:
                w.join(timeout=2)
            self._cap.release()
            logger.info("Camera released")
