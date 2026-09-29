import time

import cv2

from core.base_module import BaseModule


class CameraManager(BaseModule):
    def __init__(self, config=None):
        super().__init__("Camera")
        self.config = config or {}
        self.capture = None
        self.device = self.config.get("device", 0)
        self.width = self.config.get("width", 1280)
        self.height = self.config.get("height", 720)
        self.fps = self.config.get("fps", 30)
        self.reconnect_interval = float(self.config.get("reconnect_interval", 2.0))
        self.max_read_failures = int(self.config.get("max_read_failures", 15))
        self._last_attempt = 0.0
        self._read_failures = 0
        self._warned = False

    @property
    def available(self):
        return self.capture is not None and self.capture.isOpened()

    def _apply_format(self):
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)

    def _open(self):
        self._last_attempt = time.monotonic()
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            cap.release()
            return False
        self.capture = cap
        self._apply_format()
        self._read_failures = 0
        return True

    def _mark_lost(self, reason):
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if not self._warned:
            self.logger.error("Camera stream lost: %s", reason)
            self._warned = True
        self._last_attempt = time.monotonic()

    def initialize(self):
        super().initialize()
        if self._open():
            self.logger.info("Camera opened (device %s)", self.device)
        else:
            self.logger.error("Failed to open camera device")
            self._warned = True

    def read_frame(self):
        if self.capture is None:
            if not self._warned or (
                time.monotonic() - self._last_attempt >= self.reconnect_interval
            ):
                if self._open():
                    self.logger.info("Camera connected (device %s)", self.device)
                    self._warned = False
                else:
                    self._last_attempt = time.monotonic()
            return None

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self._read_failures += 1
            if self._read_failures >= self.max_read_failures:
                self._mark_lost("no frames from device %s" % self.device)
            return None

        if self._read_failures:
            self._read_failures = 0
        if self._warned:
            self.logger.info("Camera stream recovered (device %s)", self.device)
            self._warned = False
        return frame

    def stop(self):
        super().stop()
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self.logger.info("Camera released")
