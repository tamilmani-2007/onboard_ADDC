"""
Frame hand-off between the camera daemon (the ONLY process that opens the camera) and the mission's vision
thread, through a memory-mapped file in /dev/shm. A seqlock keeps readers from ever seeing a torn frame:
the writer makes `seq` odd while it writes and even when done; a reader accepts a frame only if `seq` was
even and unchanged before and after its copy. Plain mmap on purpose (multiprocessing.shared_memory unlinks
the segment when a reader exits on Python < 3.13).

Layout: [0:8] seq (u64) | [8:16] timestamp (f64) | [16:20] width (u32) | [20:24] height (u32) | [32:] gray bytes
"""
import mmap
import os
import struct
import time

import numpy as np

PATH = "/dev/shm/drone_cam"
HEADER = 32
_SEQ = struct.Struct("<Q")
_META = struct.Struct("<dII")


class ShmPublisher:
    def __init__(self, width, height, path=PATH):
        self.w, self.h = width, height
        size = HEADER + width * height
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
        os.fchmod(fd, 0o666)
        os.ftruncate(fd, size)
        self._mm = mmap.mmap(fd, size)
        os.close(fd)
        self._seq = _SEQ.unpack_from(self._mm, 0)[0]
        if self._seq % 2:
            self._seq += 1

    def publish(self, gray):
        if gray.shape != (self.h, self.w):
            return
        self._seq += 1                                   # odd: writing
        _SEQ.pack_into(self._mm, 0, self._seq)
        _META.pack_into(self._mm, 8, time.time(), self.w, self.h)
        self._mm[HEADER:HEADER + self.w * self.h] = np.ascontiguousarray(gray).tobytes()
        self._seq += 1                                   # even: done
        _SEQ.pack_into(self._mm, 0, self._seq)

    def close(self):
        self._mm.close()


class ShmCapture:
    """cv2.VideoCapture-like reader (same interface as the other sources in cam/capture.py)."""

    def __init__(self, path=PATH, first_frame_timeout=3.0, max_age=1.0):
        self._max_age = max_age
        self._last = 0
        fd = os.open(path, os.O_RDONLY)
        size = os.fstat(fd).st_size
        self._mm = mmap.mmap(fd, size, prot=mmap.PROT_READ)
        os.close(fd)
        self._opened = True
        ok, _ = self.read_gray(timeout=first_frame_timeout)
        if not ok:
            self.release()
            raise RuntimeError("camera daemon is not delivering frames")

    def isOpened(self):
        return self._opened

    def read_gray(self, timeout=0.2):
        """(ok, HxW uint8). Waits up to `timeout` s for a frame newer than the last one returned."""
        if not self._opened:
            return False, None
        end = time.time() + timeout
        while True:
            s1 = _SEQ.unpack_from(self._mm, 0)[0]
            if s1 % 2 == 0 and s1 != self._last and s1 > 0:
                ts, w, h = _META.unpack_from(self._mm, 8)
                if w and h and HEADER + w * h <= len(self._mm):
                    data = np.frombuffer(self._mm, np.uint8, count=w * h, offset=HEADER).reshape(h, w).copy()
                    if _SEQ.unpack_from(self._mm, 0)[0] == s1:
                        if time.time() - ts > self._max_age:
                            return False, None           # stale: daemon stopped publishing
                        self._last = s1
                        return True, data
            if time.time() >= end:
                return False, None
            time.sleep(0.002)

    def read(self):
        import cv2 as cv
        ok, g = self.read_gray()
        return (True, cv.cvtColor(g, cv.COLOR_GRAY2BGR)) if ok else (False, None)

    def release(self):
        if self._opened:
            self._opened = False
            self._mm.close()
