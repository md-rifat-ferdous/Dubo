import glob
import os
import time
from typing import Any

from modules.thrusters import dubo_protocol as proto
from modules.thrusters.mixer import (
    MOTOR_NAMES,
    mix_motion,
    normalize_mixing,
    normalize_motor_values,
    normalize_polarity,
)

serial: Any = None
try:
    import serial as serial_module

    serial = serial_module
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

list_ports: Any = None
try:
    from serial.tools import list_ports as list_ports_module

    list_ports = list_ports_module
except ImportError:
    list_ports = None

SERIAL_OK = SERIAL_AVAILABLE

# How long the ESP32 may stay silent before the link counts as unresponsive.
RX_TIMEOUT_S = 2.5

BY_ID_DIR = "/dev/serial/by-id"

# USB-serial bridges an ESP32 board is normally wired with. Used only to pick the
# right node when the configured port is missing, never to reject a working one.
USB_SERIAL_IDS = (
    (0x1A86, 0x7523),  # QinHeng CH340 / CH341
    (0x1A86, 0x7522),  # CH340K
    (0x1A86, 0x55D4),  # CH9102
    (0x10C4, 0xEA60),  # Silicon Labs CP210x
    (0x0403, 0x6001),  # FTDI FT232
    (0x0403, 0x6015),  # FTDI FT231X
    (0x303A, None),    # Espressif native USB CDC
)


def _real(path):
    try:
        return os.path.realpath(str(path))
    except OSError:
        return str(path)


def _by_id_links():
    try:
        names = sorted(os.listdir(BY_ID_DIR))
    except OSError:
        return []
    return [os.path.join(BY_ID_DIR, name) for name in names if "-port" in name]


def _stable_link(device):
    """Return the /dev/serial/by-id name for a device, which survives replugging."""
    target = _real(device)
    for link in _by_id_links():
        if _real(link) == target:
            return link
    return None


def find_serial_port(preferred=None):
    """Resolve the serial port to use.

    An existing configured port always wins. When it is missing the ESP32 is
    looked up by USB VID/PID, then by any stable by-id name, then by device
    glob, so a board plugged in after startup is still picked up.
    """
    if preferred and os.path.exists(str(preferred)):
        return str(preferred)
    if list_ports is not None:
        try:
            infos = list(list_ports.comports())
        except Exception:
            infos = []
        for info in infos:
            if (info.vid, info.pid) in USB_SERIAL_IDS:
                if info.device and os.path.exists(info.device):
                    return _stable_link(info.device) or str(info.device)
    for link in _by_id_links():
        if os.path.exists(link):
            return link
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        candidates = sorted(glob.glob(pattern))
        if candidates:
            return _stable_link(candidates[0]) or candidates[0]
    return str(preferred) if preferred else None


def apply_deadband(value, pos=0, neg=0):
    value = int(value)
    if value == 0:
        return 0
    if pos > 0 and 0 < value < pos:
        return 0
    if neg < 0 and neg < value < 0:
        return 0
    return value


class UartThrusterProvider:
    def __init__(self, config=None, sensors=None):
        self.config = config or {}
        self.sensors = sensors
        self.port = str(self.config.get("port", "/dev/ttyUSB0"))
        self.baud = int(self.config.get("baud", 115200))
        self.ramp = max(0.0, min(1.0, float(self.config.get("ramp", 0.1))))
        self.boost_scale = max(0.0, min(1.0, float(self.config.get("boost_scale", 1.0))))
        deadband = self.config.get("deadband") or {}
        self.deadband_pos = max(0, int(deadband.get("positive", 20)))
        self.deadband_neg = -abs(int(deadband.get("negative", 176)))
        self.mixing = normalize_mixing(self.config.get("mixing"))
        self.polarity = normalize_polarity(self.config.get("polarity"))
        self._ser = None
        self._open_error = None
        self._sequence = 1
        self._applied = {name: 0.0 for name in MOTOR_NAMES}
        self._targets = {name: 0.0 for name in MOTOR_NAMES}
        self._direct_values = None
        self._estop = False
        self._estop_observed = False
        self._rx = bytearray()
        self.last_status = None
        self.last_ack = None
        self._last_rx_time = 0.0
        self._last_write = 0.0
        self._opened_at = 0.0

    def resolve_port(self):
        self.port = find_serial_port(self.config.get("port", "/dev/ttyUSB0")) or self.port
        return self.port

    def open(self):
        if not SERIAL_AVAILABLE:
            self._open_error = "pyserial not installed"
            return False
        self.resolve_port()
        try:
            self._ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=1,
            )
            self._open_error = None
            self._sequence = 1
            self._rx.clear()
            self.last_status = None
            self.last_ack = None
            self._last_rx_time = 0.0
            self._last_write = 0.0
            self._applied = {name: 0.0 for name in MOTOR_NAMES}
            self._targets = {name: 0.0 for name in MOTOR_NAMES}
            self._direct_values = None
            self._estop = False
            self._estop_observed = False
            time.sleep(0.05)
            self._ser.reset_input_buffer()
            self._opened_at = time.monotonic()
            return True
        except Exception as exc:
            self._open_error = str(exc)
            self._ser = None
            return False

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._opened_at = 0.0

    def shutdown(self):
        if self._ser is not None and not self._estop:
            self._targets = {name: 0.0 for name in MOTOR_NAMES}
            self._applied = {name: 0.0 for name in MOTOR_NAMES}
            self._direct_values = None
            for _ in range(5):
                self._sync_sequence()
                if not self._write_frame(proto.build_motor_command(self._sequence, 0, 0, 0, 0, 0)):
                    break
                self._sequence = self._next_sequence()
                self._drain()
                time.sleep(0.02)
        self.close()

    @property
    def ok(self):
        return self._ser is not None

    @property
    def responding(self):
        return bool(
            self._last_rx_time
            and time.monotonic() - self._last_rx_time <= RX_TIMEOUT_S
        )

    @property
    def settled(self):
        """True once the port has been open long enough to have heard from the ESP32.

        Suppresses a false 'no response' report on the first frames after a
        connect or reconnect, before the board has had time to answer.
        """
        return bool(
            self._opened_at
            and time.monotonic() - self._opened_at >= RX_TIMEOUT_S
        )

    @property
    def ready(self):
        status = self.last_status
        return bool(
            self.ok
            and self.responding
            and status is not None
            and not status.get("estop")
            and not self._estop
        )

    def can_arm(self):
        return self.ready

    def setpoints(self, motion):
        return mix_motion(
            motion,
            mixing=self.mixing,
            polarity=self.polarity,
            boost_scale=self.boost_scale,
        )

    def set_direct_values(self, values):
        values = normalize_motor_values(values)
        self._direct_values = {
            name: values[name] * self.polarity.get(name, 1)
            for name in MOTOR_NAMES
        }

    def clear_direct_values(self):
        self._direct_values = None

    def zero(self):
        self._targets = {name: 0.0 for name in MOTOR_NAMES}
        self._direct_values = None

    def emergency_stop(self):
        self._estop = True
        self._applied = {name: 0.0 for name in MOTOR_NAMES}
        self._targets = {name: 0.0 for name in MOTOR_NAMES}
        self._direct_values = None
        if self._ser is not None:
            self._sync_sequence()
            self._write_frame(proto.build_estop(self._sequence))
            self._sequence = self._next_sequence()
            self._drain()

    def recover(self):
        if self._estop or self.last_status is None or self.last_status.get("estop"):
            return False
        return True

    def apply(self, motion, dt):
        if self._ser is None:
            return
        self._drain()
        if self._estop:
            return
        if self._direct_values is None:
            self._targets = self.setpoints(motion)
        else:
            self._targets = dict(self._direct_values)
        step = self.ramp
        for name in MOTOR_NAMES:
            target = self._targets[name]
            current = self._applied[name]
            if target > current:
                self._applied[name] = min(target, current + step)
            else:
                self._applied[name] = max(target, current - step)
        values = [
            apply_deadband(
                int(round(max(-1.0, min(1.0, self._applied[name])) * 1000)),
                self.deadband_pos,
                self.deadband_neg,
            )
            for name in MOTOR_NAMES
        ]
        self._sync_sequence()
        self._write_frame(proto.build_motor_command(self._sequence, *values))
        self._sequence = self._next_sequence()
        self._drain()

    def _next_sequence(self):
        sequence = (self._sequence + 1) & 0xFFFF
        return sequence or 1

    def _sync_sequence(self):
        if self.last_status is None:
            return
        esp_sequence = int(self.last_status.get("sequence", 0))
        difference = (self._sequence - esp_sequence) & 0xFFFF
        if difference == 0 or difference >= 0x8000:
            self._sequence = (esp_sequence + 1) & 0xFFFF or 1

    def _write_frame(self, frame):
        if self._ser is None:
            return False
        try:
            self._ser.write(frame)
            self._last_write = time.monotonic()
            return True
        except Exception as exc:
            self._open_error = str(exc)
            self.close()
            return False

    def _drain(self):
        if self._ser is None:
            return
        try:
            data = self._ser.read(self._ser.in_waiting or 0)
        except Exception as exc:
            self._open_error = str(exc)
            self.close()
            return
        if data:
            self._rx.extend(data)
        while self._rx:
            index = self._rx.find(bytes([proto.HEADER_1, proto.HEADER_2]))
            if index < 0:
                self._rx.clear()
                break
            if index:
                del self._rx[:index]
                continue
            if len(self._rx) < 5:
                break
            payload_length = self._rx[4]
            if payload_length > proto.MAX_PAYLOAD_LENGTH:
                del self._rx[:1]
                continue
            frame_length = 5 + 2 + payload_length + 2
            if len(self._rx) < frame_length:
                break
            frame = bytes(self._rx[:frame_length])
            del self._rx[:frame_length]
            self._handle_frame(frame)

    def _handle_frame(self, frame):
        parsed = proto.parse_frame(frame)
        if parsed is None:
            return
        message_type, _, payload = parsed
        if message_type in (proto.MSG_ACK, proto.MSG_STATUS):
            self._last_rx_time = time.monotonic()
        if message_type == proto.MSG_ACK:
            ack = proto.parse_ack(payload)
            if ack is not None:
                code, sequence = ack
                self.last_ack = {
                    "code": code,
                    "name": proto.ACK_NAMES.get(code, "?"),
                    "sequence": sequence,
                }
                if code == proto.ACK_ESTOP_ACTIVE:
                    self._estop = True
                    self._estop_observed = True
        elif message_type == proto.MSG_STATUS:
            status = proto.parse_status(payload)
            if status is not None:
                self.last_status = status
                if status.get("estop"):
                    self._estop = True
                    self._estop_observed = True
                elif self._estop_observed:
                    self._estop = False
                    self._estop_observed = False

    def telemetry(self):
        now = time.monotonic()
        status = self.last_status or {}
        return {
            "provider": "uart",
            "ok": self.ok,
            "responding": self.responding,
            "ready": self.ready,
            "port": self.port,
            "baud": self.baud,
            "open_error": self._open_error,
            "estop": self._estop or bool(status.get("estop")),
            "failsafe": bool(status.get("failsafe")),
            "sequence": status.get("sequence", self._sequence - 1),
            "last_ack": self.last_ack,
            "status": self.last_status,
            "rx_age_s": round(now - self._last_rx_time, 2) if self._last_rx_time else None,
            "write_age_s": round(now - self._last_write, 2) if self._last_write else None,
            "targets": {name: round(value, 3) for name, value in self._targets.items()},
            "applied": {name: round(value, 3) for name, value in self._applied.items()},
            "deadband": [self.deadband_pos, self.deadband_neg],
            "mixing": self.mixing,
            "polarity": self.polarity,
        }
