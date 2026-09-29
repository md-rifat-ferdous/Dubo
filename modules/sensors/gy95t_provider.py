"""I2C driver for the WitMotion GY-95T AHRS/IMU.

Measured behaviour of the module (verified with i2cdetect + smbus2 on the Pi, bus 1,
slave 0x52, PS tied to GND for I2C mode):

* The module answers on I2C address 0x52.  0x50 is the WitMotion factory default, so
  0x52 means the address was already changed via the ``IICADDR`` register (0x1A).
* It does **not** expose a conventional register map.  The byte written before a read
  is a *start offset* into an internal 45-byte (0x2D) transmit buffer, so reading
  ``0x00`` and reading ``0x01`` return overlapping windows of the same live stream.
* Offsets 0x00..0x2C are readable; 0x2D and beyond NACK and hang the bus.
* Bytes 0x00..0x07 are a **constant preamble** (``A4 06 01 00 00 00 7A FF``).  It is
  identical on every read regardless of orientation, so it is metadata, not sensor
  data.  ``Gy95tLayout.data_offset`` therefore defaults to 8 to skip it.  Setting it
  to 0 makes ``unpack_from`` reinterpret those fixed bytes as int16 and every angle
  freezes at a constant.
* Individual transfers are unreliable under back-to-back hammering and intermittently
  fail with ``OSError(EIO)`` / ``TimeoutError``; reads are retried and a failure never
  becomes published attitude.
* ``smbus2.read_i2c_block_data`` cannot read more than 32 bytes (SMBus block limit),
  so the 45-byte buffer is stitched from two offset windows.

SCALE NOTE.  Two conventions are supported because the module does not document its
packed I2C buffer:

* ``0.01``   - divide by 100.  Yields 0.00-180.00 over a +/-18000 raw range.
* ``0.0054931640625`` - WitMotion's documented angle encoding, signed int16 mapped
  over +/-180 deg (180.0 / 32768).  This is the vendor-correct scale for Euler angles
  and is the default.

ANGLE OFFSETS ARE STILL UNVERIFIED.  No tilt test has been run against this module, so
which payload bytes carry roll/pitch/yaw is a working hypothesis, not a fact.  A value
that looks plausible while the module sits flat is not evidence of a correct offset.
Run ``scripts/gy95t_calibrate.py`` (static byte map) and ``scripts/tilt_watch.py``
(spans per axis under physical tilt) before trusting the attitude display.
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:  # pragma: no cover
    from modules.sensors.sensor_manager import HudState

try:  # pragma: no cover - import guard exercised on the Pi only
    import smbus2

    SMBUS2_AVAILABLE = True
except ImportError:  # pragma: no cover
    smbus2 = None  # type: ignore[assignment]
    SMBUS2_AVAILABLE = False

#: False when smbus2 is missing; the manager uses it to skip hardware entirely.
GY95T_OK = SMBUS2_AVAILABLE

LOGGER = logging.getLogger("Sensors.GY95T")

DEFAULT_BUS = 1
DEFAULT_ADDRESS = 0x52

#: Start offset of the payload window; the module ignores it as a register selector.
DATA_REGISTER = 0x00

#: Readable window of the internal buffer, measured at 0x2D.
STREAM_LENGTH = 0x2D

#: Constant 8-byte preamble at the head of every transfer; metadata, not data.
STREAM_PREAMBLE = bytes((0xA4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x7A, 0xFF))

#: First payload byte after the preamble.
PAYLOAD_OFFSET = len(STREAM_PREAMBLE)

#: Widest transfer the module reliably accepts (SMBus block-read cap is 32).
MAX_CHUNK = 32

#: Divide-by-100 convention.
SCALE_DIV_100 = 0.01

#: WitMotion documented Euler-angle encoding: signed int16 over +/-180 deg.
SCALE_WIT_ANGLE = 180.0 / 32768.0

DEFAULT_SCALE = SCALE_WIT_ANGLE

#: Values outside this range are a decode/layout error, not attitude.
MAX_PLAUSIBLE_ANGLE_DEG = 180.0

#: struct format characters for the two byte orders, four int16 per unpack.
_BYTEORDER_FORMATS = {"little": "<hhhh", "big": ">hhhh"}


@dataclass(frozen=True)
class Gy95tLayout:
    """How to turn the packed payload into angles.

    ``data_offset`` is where the int16 block starts, ``count`` how many int16 to
    unpack, and ``byteorder`` selects the ``struct`` format with an automatic
    fallback to the other order when the result is implausible.
    """

    data_offset: int = PAYLOAD_OFFSET
    count: int = 4
    byteorder: str = "little"
    scale: float = DEFAULT_SCALE

    @classmethod
    def from_config(cls, config: Optional[dict[str, Any]]) -> "Gy95tLayout":
        layout_cfg = (config or {}).get("layout") or {}
        defaults = cls()
        if not isinstance(layout_cfg, dict):
            return defaults
        byteorder = str(layout_cfg.get("byteorder", defaults.byteorder)).lower()
        if byteorder not in _BYTEORDER_FORMATS:
            byteorder = defaults.byteorder
        count = int(layout_cfg.get("count", defaults.count))
        if count not in (1, 2, 3, 4):
            count = defaults.count
        return cls(
            data_offset=int(layout_cfg.get("data_offset", defaults.data_offset)),
            count=count,
            byteorder=byteorder,
            scale=float(layout_cfg.get("scale", defaults.scale)),
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "data_offset": self.data_offset,
            "count": self.count,
            "byteorder": self.byteorder,
            "scale": self.scale,
        }


@dataclass
class ImuSample:
    """One decoded attitude fix."""

    roll: float
    pitch: float
    yaw: float
    valid: bool
    raw: bytes = b""
    error: Optional[str] = None
    byteorder: str = "little"

    def as_dict(self) -> dict[str, Any]:
        return {
            "roll": round(self.roll, 2),
            "pitch": round(self.pitch, 2),
            "yaw": round(self.yaw),
            "heading": round(self.yaw),
            "valid": self.valid,
            "error": self.error,
            "byteorder": self.byteorder,
        }


class GY95TProvider:
    """Reads attitude from a GY-95T over I2C.

    Mirrors the provider contract used by the rest of the codebase so
    :class:`~modules.sensors.sensor_manager.SensorManager` can swap it against
    ``SimulatedSensorProvider`` without any UI change.
    """

    #: The packed I2C payload has not been mapped to a barometric altitude, so this
    #: provider must not advertise a depth source to the UI.
    provides_depth = False

    def __init__(self, config: Optional[dict[str, Any]] = None) -> None:
        self.config = config or {}
        self.logger = LOGGER
        self.bus_number = int(self.config.get("i2c_bus", DEFAULT_BUS))
        self.address = int(self.config.get("i2c_address", DEFAULT_ADDRESS))
        self.read_retries = max(1, int(self.config.get("read_retries", 3)))
        self.reconnect_interval_s = max(
            0.1, float(self.config.get("reconnect_interval_s", 2.0))
        )
        self.layout = Gy95tLayout.from_config(self.config)
        self._bus: Any = None
        self._open_error: Optional[str] = None
        self._consecutive_failures = 0
        self._total_reads = 0
        self._total_errors = 0
        self._last_attempt = 0.0
        self._last_sample: Optional[ImuSample] = None

    # ------------------------------------------------------------------ lifecycle

    @property
    def ok(self) -> bool:
        """True when the bus is open and the module answered recently."""
        return self._bus is not None and self._consecutive_failures < self.read_retries

    def connect(self) -> bool:
        """Open the I2C bus and confirm the module is answering.

        Returns True on success.  Failures are recorded in ``_open_error`` instead of
        raising, so the manager can fall back to simulation.
        """
        if not SMBUS2_AVAILABLE or smbus2 is None:
            self._open_error = "smbus2 not installed (pip install smbus2)"
            self.logger.error("GY-95T unavailable: %s", self._open_error)
            return False
        self.close()
        try:
            bus = smbus2.SMBus(self.bus_number)
        except Exception as exc:
            self._open_error = f"cannot open /dev/i2c-{self.bus_number}: {exc}"
            self.logger.error("GY-95T %s", self._open_error)
            return False
        # Probe before declaring success: an open bus says nothing about the module.
        if self._read_window(bus, 0, min(STREAM_LENGTH, MAX_CHUNK)) is None:
            try:
                bus.close()
            except Exception:
                pass
            self._open_error = f"no response at 0x{self.address:02X} on bus {self.bus_number}"
            self.logger.error(
                "GY-95T %s; check wiring, PS->GND for I2C mode and i2cdetect -y %d",
                self._open_error,
                self.bus_number,
            )
            return False
        self._bus = bus
        self._open_error = None
        self._consecutive_failures = 0
        self._last_attempt = 0.0
        self.logger.info(
            "GY-95T connected on bus %d at 0x%02X (%d-byte window)",
            self.bus_number,
            self.address,
            STREAM_LENGTH,
        )
        return True

    def close(self) -> None:
        """Release the bus handle.  Safe to call when already closed."""
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                self.logger.debug("GY-95T close failed", exc_info=True)
            self._bus = None
        self._consecutive_failures = 0

    # ------------------------------------------------------------------ reading

    def read_data(self) -> ImuSample:
        """Read one attitude fix, reconnecting as needed.

        Never raises: on persistent failure the last good fix is repeated with
        ``valid=False`` so a dropped sensor degrades to a flagged reading instead of
        an exception in the 100 ms service tick.
        """
        if not self.ok:
            if not self._should_retry():
                return self._degraded("sensor not connected")
            if not self.connect():
                return self._degraded(self._open_error or "connect failed")

        buffer = self._read_full_buffer()
        if buffer is None:
            self._consecutive_failures += 1
            self._total_errors += 1
            if self._consecutive_failures >= self.read_retries:
                self.logger.error(
                    "GY-95T read failed %d times in a row; dropping connection",
                    self._consecutive_failures,
                )
                self.close()
            return self._degraded("i2c read failed")

        self._consecutive_failures = 0
        self._total_reads += 1
        sample = self._decode(buffer)
        self._last_sample = sample
        if not sample.valid:
            self._total_errors += 1
            self.logger.warning("GY-95T decode rejected: %s", sample.error)
        return sample

    def read_values(self) -> Optional[dict[str, Any]]:
        """Payload expected by the UI layer.

        Returns ``{"roll", "pitch", "yaw", "heading", "depth"}`` with yaw and heading
        normalized to [0.0, 360.0), or ``None`` when no valid fix is available.
        """
        sample = self.read_data()
        if not sample.valid:
            return None
        return {
            "roll": float(sample.roll),
            "pitch": float(sample.pitch),
            "yaw": float(sample.yaw),
            "heading": float(sample.yaw),
            "depth": 0.0,
        }

    def get_state(self) -> "HudState":
        """Provider-interface adapter used by :class:`SensorManager`.

        Imported locally because ``sensor_manager`` loads this module.
        """
        from modules.sensors.sensor_manager import HudState

        sample = self.read_data()
        return HudState(
            yaw_deg=sample.yaw,
            roll=sample.roll,
            pitch=sample.pitch,
            imu_valid=sample.valid,
            depth_valid=False,
        )

    def apply_motion(self, *args: Any, **kwargs: Any) -> None:
        """No-op: attitude is measured, not commanded.  Kept for interface parity."""

    def telemetry(self) -> dict[str, Any]:
        sample = self._last_sample
        return {
            "provider": "gy95t",
            "ok": self.ok,
            "i2c_bus": self.bus_number,
            "i2c_address": self.address,
            "open_error": self._open_error,
            "reads": self._total_reads,
            "errors": self._total_errors,
            "consecutive_failures": self._consecutive_failures,
            "layout": self.layout.as_dict(),
            "last_sample": sample.as_dict() if sample else None,
        }

    # ------------------------------------------------------------------ internals

    def _should_retry(self) -> bool:
        """Rate-limit reconnection attempts to the configured interval."""
        now = time.monotonic()
        if now - self._last_attempt < self.reconnect_interval_s:
            return False
        self._last_attempt = now
        return True

    def _degraded(self, error: str) -> ImuSample:
        """Repeat the previous attitude but mark it unusable."""
        last = self._last_sample
        if last is not None:
            return ImuSample(
                roll=last.roll,
                pitch=last.pitch,
                yaw=last.yaw,
                valid=False,
                raw=last.raw,
                error=error,
                byteorder=last.byteorder,
            )
        return ImuSample(roll=0.0, pitch=0.0, yaw=0.0, valid=False, error=error)

    def _read_window(self, bus: Any, offset: int, length: int) -> Optional[bytes]:
        """Single transfer, no retries.  Returns None on any bus failure."""
        try:
            return bytes(bus.read_i2c_block_data(self.address, offset, length))
        except Exception as exc:
            self.logger.debug(
                "GY-95T transfer failed at offset 0x%02X len %d: %s",
                offset,
                length,
                exc,
            )
            return None

    def _read_attempt(self, bus: Any) -> Optional[bytes]:
        """Read the full 45-byte buffer by stitching two <=32-byte windows."""
        head_len = min(STREAM_LENGTH, MAX_CHUNK)
        head = self._read_window(bus, 0, head_len)
        if head is None:
            return None
        if STREAM_LENGTH <= MAX_CHUNK:
            return head
        tail_len = STREAM_LENGTH - MAX_CHUNK
        tail = self._read_window(bus, MAX_CHUNK, tail_len)
        if tail is None:
            return None
        return head + tail

    def _read_full_buffer(self) -> Optional[bytes]:
        """Retry the stitched read; individual transfers are flaky by nature."""
        if self._bus is None:
            return None
        for attempt in range(1, self.read_retries + 1):
            buffer = self._read_attempt(self._bus)
            if buffer is None:
                self.logger.debug(
                    "GY-95T read attempt %d/%d failed", attempt, self.read_retries
                )
                continue
            if len(buffer) < STREAM_LENGTH:
                self.logger.debug(
                    "GY-95T short read: %d of %d bytes", len(buffer), STREAM_LENGTH
                )
                continue
            if buffer[:PAYLOAD_OFFSET] != STREAM_PREAMBLE:
                # Not fatal: the preamble is informational, the payload still parses.
                self.logger.debug(
                    "GY-95T preamble changed: %s", buffer[:PAYLOAD_OFFSET].hex(" ")
                )
            return buffer
        return None

    def _unpack_angles(self, buffer: bytes, order: str) -> Optional[list[float]]:
        """Unpack ``count`` int16 from the layout offset and scale to degrees.

        Returns None if the buffer is too short for the requested window.
        """
        layout = self.layout
        size = 2 * layout.count
        if layout.data_offset < 0 or layout.data_offset + size > len(buffer):
            return None
        try:
            values = struct.unpack_from(
                _BYTEORDER_FORMATS[order], buffer, layout.data_offset
            )
        except struct.error as exc:
            self.logger.debug("GY-95T unpack failed: %s", exc)
            return None
        return [value * layout.scale for value in values]

    @staticmethod
    def _plausible(values: list[float]) -> bool:
        """Roll/pitch must be real angles; a +/-180 tilt is legitimate."""
        for angle in values[:2]:
            if abs(angle) > MAX_PLAUSIBLE_ANGLE_DEG:
                return False
        return True

    def _decode(self, buffer: bytes) -> ImuSample:
        """Turn the raw buffer into an attitude fix, or explain the rejection."""
        raw = bytes(buffer)
        preferred = self.layout.byteorder
        orders = [preferred] + [o for o in _BYTEORDER_FORMATS if o != preferred]

        chosen: Optional[str] = None
        angles: Optional[list[float]] = None
        for order in orders:
            candidate = self._unpack_angles(raw, order)
            if candidate is None:
                continue
            if self._plausible(candidate):
                chosen, angles = order, candidate
                break
            if angles is None:
                # Keep the first decodable result so a rejection has real numbers.
                chosen, angles = order, candidate
        if angles is None or len(angles) < 2:
            return ImuSample(
                0.0,
                0.0,
                0.0,
                False,
                raw,
                f"payload too short for {self.layout.count} int16 at "
                f"offset 0x{self.layout.data_offset:02X}",
            )
        if not self._plausible(angles):
            return ImuSample(
                angles[0],
                angles[1] if len(angles) > 1 else 0.0,
                normalize_yaw(angles[2] if len(angles) > 2 else 0.0),
                False,
                raw,
                f"implausible angles {angles[:2]}; check layout data_offset/scale",
                chosen or preferred,
            )
        if chosen != preferred:
            self.logger.info(
                "GY-95T byte order resolved to '%s' (preferred '%s' was implausible)",
                chosen,
                preferred,
            )
        return ImuSample(
            roll=angles[0],
            pitch=angles[1],
            yaw=normalize_yaw(angles[2] if len(angles) > 2 else 0.0),
            valid=True,
            raw=raw,
            byteorder=chosen or preferred,
        )


def normalize_yaw(value: float) -> float:
    """Wrap an angle into [0.0, 360.0)."""
    return float(value) % 360.0
