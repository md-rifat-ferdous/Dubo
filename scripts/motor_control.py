#!/usr/bin/env python3
"""Interactive DUBO thruster / motor control console for the Raspberry Pi.

Streams motor commands to the ESP32 over USB/serial (auto-detects
/dev/ttyUSB* or /dev/ttyACM*), applies the same M1..M5 mixing used by the
app (configs/thrusters.yaml), keeps the ESP32 250 ms watchdog fed with a
smooth 10% ramp, and prints ACK/STATUS telemetry.

Usage:
    .venv/bin/python scripts/motor_control.py [port] [baud]

Commands:
    fwd <v> | back <v>     surge  -> M2/M3
    up <v>  | down <v>     heave  -> M1/M4/M5
    left <v>| right <v>    yaw    -> M2/M3 differential
    pitchup <v> | pitchdown <v>   -> M1 vs M4/M5
    rollleft <v> | rollright <v>  -> M4 vs M5
    m1 <v> m2 <v> ... m5 <v>      any motor directly (-1000..+1000)
    zero | stop            all motors to 0
    estop                  latched E-STOP (needs ESP32 power-cycle to clear)
    status | ack           show last ESP32 telemetry
    help | quit

Values are -1000..+1000 (1000 = full speed).
"""
import glob
import select
import sys
import time

sys.path.insert(0, ".")

from modules.thrusters import dubo_protocol as proto
from modules.thrusters.mixer import (
    DEFAULT_MIXING,
    DEFAULT_POLARITY,
    load_thruster_config,
)
from modules.thrusters.uart_provider import apply_deadband

try:
    import serial
except ImportError:
    print("pyserial not installed: .venv/bin/pip install pyserial")
    sys.exit(1)

MOTORS = ["M1", "M2", "M3", "M4", "M5"]

AXIS_LABELS = {
    "fwd": ("surge", 1.0),
    "back": ("surge", -1.0),
    "up": ("heave", 1.0),
    "down": ("heave", -1.0),
    "left": ("yaw", -1.0),
    "right": ("yaw", 1.0),
    "pitchup": ("pitch", 1.0),
    "pitchdown": ("pitch", -1.0),
    "rollleft": ("roll", -1.0),
    "rollright": ("roll", 1.0),
}


def find_port():
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if candidates:
        return candidates[0]
    return load_thruster_config().get("port", "/dev/serial0")


def motion_to_motors(axes, mixing=None, polarity=None):
    """Turn {axis: -1..1} into {M1..M5: -1000..1000} via the app mixing."""
    mixing = mixing or DEFAULT_MIXING
    polarity = polarity or DEFAULT_POLARITY
    out = {}
    for name, weights in mixing.items():
        value = sum(axes.get(axis, 0.0) * weight for axis, weight in weights.items())
        out[name] = int(round(max(-1.0, min(1.0, value * polarity.get(name, 1))) * 1000))
    return out


def spark(value):
    n = abs(value) * 20 // 1000
    return ("+" * n) if value >= 0 else ("-" * n)


class Console:
    def __init__(self, ser):
        self.ser = ser
        self.rx = bytearray()
        self.seq = 1
        self.config = load_thruster_config()
        self.mixing = self.config["mixing"]
        self.polarity = self.config["polarity"]
        deadband = self.config["deadband"]
        self.deadband_pos = deadband["positive"]
        self.deadband_neg = -abs(deadband["negative"])
        self.targets = {name: 0 for name in MOTORS}
        self.applied = {name: 0 for name in MOTORS}
        self.last_ack = None
        self.last_status = None
        self.last_status_print = 0.0
        self.estop_latched = False
        self.estop_observed = False

    # ------------------------------------------------------------- serial

    def _write(self, frame):
        self.ser.write(frame)

    def _send_estop(self):
        self._sync_seq()
        self._write(proto.build_estop(self.seq))
        self.seq = (self.seq + 1) & 0xFFFF or 1

    def drain(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except Exception:
            data = b""
        self.rx.extend(data)
        while True:
            idx = self.rx.find(bytes([proto.HEADER_1, proto.HEADER_2]))
            if idx > 0:
                del self.rx[:idx]
                continue
            if idx == -1:
                self.rx.clear()
                break
            if len(self.rx) < 5:
                break
            frame_len = 5 + 2 + self.rx[4] + 2
            if len(self.rx) < frame_len:
                break
            parsed = proto.parse_frame(bytes(self.rx[:frame_len]))
            del self.rx[:frame_len]
            if parsed is None:
                continue
            mtype, seq, payload = parsed
            if mtype == proto.MSG_ACK:
                ack = proto.parse_ack(payload)
                if ack is None:
                    continue
                code, echo = ack
                name = proto.ACK_NAMES.get(code, "?")
                self.last_ack = (code, name, echo)
                if code == proto.ACK_ESTOP_ACTIVE:
                    self.estop_latched = True
                    self.estop_observed = True
                    print("\n  ! ESP32 E-STOP LATCHED - power-cycle the ESP32 to clear")
                elif code != proto.ACK_OK:
                    print(f"  [ACK] seq={echo} status={name}")
            elif mtype == proto.MSG_STATUS:
                self.last_status = proto.parse_status(payload)
                if self.last_status and self.last_status["estop"]:
                    self.estop_latched = True
                    self.estop_observed = True
                elif self.estop_observed:
                    self.estop_latched = False
                    self.estop_observed = False
                    print("\n  ESP32 E-STOP cleared after power-cycle; explicit re-command required")

    def _print_status(self, force=False):
        st = self.last_status
        now = time.monotonic()
        if st is None or (not force and now - self.last_status_print < 1.0):
            return
        self.last_status_print = now
        fs = "FAILSAFE" if st["failsafe"] else "armed"
        es = "E-STOP" if st["estop"] else "ok"
        motors = "  ".join(
            f"{name}:{st['motors'][name]:+5d} {spark(st['motors'][name])}"
            for name in MOTORS
        )
        print(f"\n[STATUS] {fs} | {es}\n  {motors}\n")

    # ------------------------------------------------------------- motion

    def set_motion(self, axis, direction, value):
        value = max(-1000, min(1000, value))
        motor_values = motion_to_motors(
            {axis: direction * value / 1000.0},
            mixing=self.mixing,
            polarity=self.polarity,
        )
        for name in MOTORS:
            self.targets[name] = apply_deadband(
                motor_values[name], self.deadband_pos, self.deadband_neg)
        print(f"  -> {axis} {direction * value:+d}")
        self._show_targets()

    def set_direct(self, value_map):
        for name, val in value_map.items():
            clipped = int(round(int(val) * self.polarity.get(name, 1)))
            clipped = max(-1000, min(1000, clipped))
            self.targets[name] = apply_deadband(
                clipped, self.deadband_pos, self.deadband_neg)
        self._show_targets()

    def _show_targets(self):
        line = "  targets: " + "  ".join(
            f"{name}:{self.targets[name]:+5d}" for name in MOTORS
        )
        print(line)

    # ------------------------------------------------------------- commands

    def handle(self, line):
        parts = line.split()
        name = parts[0].lower()
        arg = lambda: int(parts[1]) if len(parts) > 1 else 300

        if name in ("quit", "exit"):
            return True
        if name in ("help",):
            print(__doc__)
        elif name in ("zero", "stop"):
            self.set_direct({m: 0 for m in MOTORS})
        elif name in AXIS_LABELS:
            axis, direction = AXIS_LABELS[name]
            self.set_motion(axis, direction, arg())
        elif name.startswith("m") and name[1:] in ("1", "2", "3", "4", "5"):
            vm = {}
            for i in range(1, len(parts), 2):
                key = parts[i].upper()
                if key in MOTORS and i + 1 < len(parts):
                    vm[key] = int(parts[i + 1])
            self.set_direct(vm)
        elif name == "estop":
            self._send_estop()
            self.estop_latched = True
            print("  sent E-STOP (latched until ESP32 power-cycle)")
        elif name == "status":
            self._print_status(force=True)
        elif name == "ack":
            print("  last_ack:", self.last_ack)
        else:
            print("  unknown command (try: help)")
        return False

    # ------------------------------------------------------------- loop

    def run(self):
        print("Type 'help' for commands. 'quit'/'Ctrl-C' to exit, motors stop.")
        print(f"ESC deadband: +{self.deadband_pos} / {self.deadband_neg}\n")
        while True:
            self.drain()
            self._print_status()
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            except KeyboardInterrupt:
                break
            if ready:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip()
                if line and self.handle(line):
                    break
            self._ramp_and_stream()
        self.stop()

    def _ramp_and_stream(self):
        if self.estop_latched:
            return
        self._sync_seq()
        step = max(1, int(round(float(self.config.get("ramp", 0.1)) * 1000)))
        for name in MOTORS:
            cur, tgt = self.applied[name], self.targets[name]
            if tgt > cur:
                self.applied[name] = min(tgt, cur + step)
            else:
                self.applied[name] = max(tgt, cur - step)
        values = [
            apply_deadband(self.applied[m], self.deadband_pos, self.deadband_neg)
            for m in MOTORS
        ]
        self._write(proto.build_motor_command(self.seq, *values))
        self.seq = (self.seq + 1) & 0xFFFF or 1

    def _sync_seq(self):
        """Keep our frame sequence strictly newer than the ESP32's last one.

        When this tool starts (or reconnects) against an ESP32 that already
        accepted higher sequence numbers, starting at seq=1 would be seen as
        OLD_SEQUENCE and the ESP32 would stay in FAILSAFE. Sync from STATUS.
        """
        st = self.last_status
        if st is None:
            return
        esp = st.get("sequence", 0)
        diff = (self.seq - esp) & 0xFFFF
        if diff == 0 or diff >= 0x8000:
            self.seq = (esp + 1) & 0xFFFF or 1

    def stop(self):
        self.targets = {m: 0 for m in MOTORS}
        self.applied = {m: 0 for m in MOTORS}
        for _ in range(12):
            self._ramp_and_stream()
            time.sleep(0.05)
        print("motors stopped")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else int(load_thruster_config().get("baud", 115200))
    ser = serial.Serial(port, baud, timeout=0, write_timeout=1)
    print(f"Open {port} @ {baud}")
    time.sleep(0.2)
    ser.reset_input_buffer()
    Console(ser).run()
    ser.close()


if __name__ == "__main__":
    main()
