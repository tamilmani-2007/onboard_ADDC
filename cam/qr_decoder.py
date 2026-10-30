import cv2 as cv
import numpy as np

from utils.config import QR_TRY_UPSCALE


class QRReader:
    """QR detection on a grayscale frame.

    decode is only attempted once a code has been located, and if that fails the located area is retried
    on a 2x upscaled crop (small codes seen from altitude). ArUco-based detector as a fallback.
    """

    def __init__(self):
        self._aruco = cv.QRCodeDetectorAruco() if hasattr(cv, "QRCodeDetectorAruco") else None
        self._classic = cv.QRCodeDetector()

    def read(self, gray):
        """Return (text, 4x2 corner points). text == "" means a code was LOCATED but not decoded;
        (None, None) means nothing found."""
        # Classic first: it decoded the test codes reliably; the ArUco-based detector (cheaper on frames
        # without a code, ~4 ms) is the fallback for blurred / strongly tilted codes.
        text, points = self._try_classic(gray)
        if text:
            return text, points
        located = points
        if self._aruco is not None:
            text, points = self._try_aruco(gray)
            if text:
                return text, points
            located = located if located is not None else points
        return ("", located) if located is not None else (None, None)

    def _try_aruco(self, gray):
        try:
            text, points, _ = self._aruco.detectAndDecode(gray)
        except cv.error:
            return None, None
        if points is not None and len(points):
            return text or "", np.asarray(points).reshape(-1, 2)
        return None, None

    def _try_classic(self, gray):
        try:
            found, points = self._classic.detect(gray)
        except cv.error:
            return None, None
        if not found or points is None:
            return None, None
        pts = np.asarray(points).reshape(-1, 2)
        text, _ = self._classic.decode(gray, points)
        if text:
            return text, pts
        if QR_TRY_UPSCALE:
            h, w = gray.shape[:2]
            pad = 0.3 * max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1]))
            x0, y0 = int(max(pts[:, 0].min() - pad, 0)), int(max(pts[:, 1].min() - pad, 0))
            x1, y1 = int(min(pts[:, 0].max() + pad, w)), int(min(pts[:, 1].max() + pad, h))
            crop = cv.resize(gray[y0:y1, x0:x1], None, fx=2, fy=2, interpolation=cv.INTER_CUBIC)
            try:
                text, p2, _ = self._classic.detectAndDecode(crop)
            except cv.error:
                return "", pts
            if text and p2 is not None:
                return text, np.asarray(p2).reshape(-1, 2) / 2.0 + np.array([x0, y0])
        return "", pts


_reader = None


def decode_QR(gray):
    """Backward-compatible helper: decoded text ('' if none)."""
    global _reader
    if _reader is None:
        _reader = QRReader()
    text, _ = _reader.read(gray)
    return text or ""
