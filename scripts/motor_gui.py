#!/usr/bin/env python3
"""Interactive DUBO motor/ESTOP console - GUI (PySide6).

Connects the Raspberry Pi to the ESP32 over USB serial, streams DUBO motor
frames (keeps the ESP32 250 ms watchdog fed, 10% ramp), and shows the ESP32's
real ACK/STATUS feedback live: failsafe/E-STOP pills, per-motor bars, last
accepted sequence.

Run:
    .venv/bin/python scripts/motor_gui.py
"""
import glob
import sys
import time

sys.path.insert(0, ".")

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from modules.thrusters import dubo_protocol as proto
from modules.thrusters.mixer import (
    DEFAULT_MIXING,
    DEFAULT_POLARITY,
    load_thruster_config,
)
from modules.thrusters.uart_provider import apply_deadband
from ui.glass import GlassPanel, StatusPill, caps_label
from ui.theme import C, FONT_DATA, apply_theme

try:
    import serial
except ImportError:
    print("pyserial not installed: .venv/bin/pip install pyserial")
    sys.exit(1)

MOTORS = ["M1", "M2", "M3", "M4", "M5"]

AXIS_LABELS = {
    "FWD": ("surge", 1.0),
    "BACK": ("surge", -1.0),
    "UP": ("heave", 1.0),
    "DOWN": ("heave", -1.0),
    "YAW L": ("yaw", -1.0),
    "YAW R": ("yaw", 1.0),
    "PITCH UP": ("pitch", 1.0),
    "PITCH DN": ("pitch", -1.0),
    "ROLL L": ("roll", -1.0),
    "ROLL R": ("roll", 1.0),
}

COL_PRIMARY = QColor(C["primary"])
COL_AMBER = QColor("#ffd364")
COL_DIM = QColor(C["on_surface_variant"])
COL_SURF = QColor(C["surface"])


def motion_to_motors(axes, mixing=None, polarity=None):
    mixing = mixing or DEFAULT_MIXING
    polarity = polarity or DEFAULT_POLARITY
    out = {}
    for name, weights in mixing.items():
        value = sum(axes.get(axis, 0.0) * weight for axis, weight in weights.items())
        out[name] = int(round(max(-1.0, min(1.0, value * polarity.get(name, 1))) * 1000))
    return out


def list_ports():
    config = load_thruster_config()
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")) or [config.get("port", "/dev/serial0")]


class MotorRow(QWidget):
    """Name + bidirectional bar + value for one motor."""

    def __init__(self, name, parent=None):
        super().__init__(parent)
        self.name = name
        self.value = 0
        self.setFixedHeight(30)

    def set_value(self, value):
        self.value = int(value)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        label_w = 40
        value_w = 52
        x0 = label_w
        bw = w - label_w - value_w
        cx = x0 + bw // 2
        p.fillRect(0, 0, w, h, QColor(13, 28, 50, 120))
        p.setPen(QPen(COL_DIM, 1))
        p.drawText(6, -1, label_w - 6, h + 2, Qt.AlignVCenter, self.name)
        p.setPen(QPen(QColor(C["outline_variant"]), 1))
        p.drawLine(int(cx), int(h * 0.35), int(cx), int(h * 0.65))
        mid = h / 2
        half = bw / 2 - 4
        frac = max(-1.0, min(1.0, self.value / 1000.0))
        span = abs(frac) * half
        if span > 2:
            color = COL_PRIMARY if self.value >= 0 else COL_AMBER
            if self.value >= 0:
                p.fillRect(cx + 1, int(mid - 3), int(span), 6, color)
            else:
                p.fillRect(cx - int(span), int(mid - 3), int(span), 6, color)
        p.setPen(QPen(COL_DIM if not self.value else COL_PRIMARY, 1))
        p.drawText(x0 + bw + 4, -1, value_w - 4, h + 2, Qt.AlignVCenter,
                   f"{self.value:+4d}")
        p.end()


class MotorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DUBO MOTOR CONSOLE - ESP32")
        self.resize(860, 640)

        self.ser = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.seq = 1
        self.config = load_thruster_config()
        self.mixing = self.config["mixing"]
        self.polarity = self.config["polarity"]
        deadband = self.config["deadband"]
        self.deadband_pos = deadband["positive"]
        self.deadband_neg = -abs(deadband["negative"])
        self.targets = {m: 0 for m in MOTORS}
        self.applied = {m: 0 for m in MOTORS}
        self.last_status = None
        self.last_ack = None
        self.estop_latched = False
        self.estop_observed = False
        self._held = set()
        self._last_log = 0.0

        self._build_ui()
        self._refresh_ports()

    # ------------------------------------------------------------ UI

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        root.addWidget(self._build_connbar())
        body = QHBoxLayout()
        body.setSpacing(10)
        body.addWidget(self._build_feedback(), 1)
        body.addWidget(self._build_control(), 1)
        root.addLayout(body, 1)
        root.addWidget(self._build_log(), 2)
        self.setCentralWidget(central)

    def _build_connbar(self):
        panel = GlassPanel()
        row = QHBoxLayout()
        row.setContentsMargins(10, 6, 10, 6)
        row.setSpacing(8)

        row.addWidget(caps_label("PORT"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(140)
        row.addWidget(self.port_combo)
        refresh = QPushButton("REFRESH")
        refresh.clicked.connect(self._refresh_ports)
        row.addWidget(refresh)

        self.baud_combo = QComboBox()
        for b in ("115200", "57600", "9600"):
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText(str(self.config.get("baud", 115200)))
        row.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("CONNECT")
        self.connect_btn.setObjectName("Primary")
        self.connect_btn.clicked.connect(self._toggle_connect)
        row.addWidget(self.connect_btn)

        row.addSpacing(16)
        zero_btn = QPushButton("ZERO")
        zero_btn.clicked.connect(self._zero)
        row.addWidget(zero_btn)

        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setObjectName("Danger")
        self.estop_btn.clicked.connect(self._estop)
        row.addWidget(self.estop_btn)

        row.addStretch(1)
        panel.body().addLayout(row)
        return panel

    @staticmethod
    def _wrap(layout):
        w = QWidget()
        w.setLayout(layout)
        return w

    def _build_feedback(self):
        panel = GlassPanel("ESP32 FEEDBACK")
        panel.setMinimumWidth(330)

        pills = QHBoxLayout()
        pills.setSpacing(8)
        self.fs_pill = StatusPill("FAILSAFE", "info")
        self.es_pill = StatusPill("E-STOP OFF", "ok")
        pills.addWidget(self.fs_pill)
        pills.addWidget(self.es_pill)
        pills.addStretch(1)
        panel.add_row(self._wrap(pills))

        self.rows = {}
        for m in MOTORS:
            row = MotorRow(m)
            self.rows[m] = row
            panel.add_row(row)

        info = QHBoxLayout()
        info.setSpacing(8)
        self.seq_pill = StatusPill("SEQ --", "info")
        self.ack_pill = StatusPill("ACK --", "info")
        info.addWidget(self.seq_pill)
        info.addWidget(self.ack_pill)
        info.addStretch(1)
        panel.add_row(self._wrap(info))
        return panel

    def _build_control(self):
        panel = GlassPanel("CONTROL")
        panel.setMinimumWidth(330)

        bar = QHBoxLayout()
        self.speed_lbl = QLabel("SPEED")
        self.speed_lbl.setStyleSheet(
            f"color: {C['primary']}; font-family: {FONT_DATA}; font-size: 16px;"
        )
        bar.addWidget(caps_label("SPEED"))
        bar.addStretch(1)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(0, 1000)
        self.speed_slider.setValue(250)
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_lbl.setText(f"{v:4d}"))
        bar.addWidget(self.speed_slider)
        bar.addWidget(self.speed_lbl)
        panel.add_row(self._wrap(bar))

        db_label = QLabel(
            f"ESC DEADBAND  +{self.deadband_pos} / {self.deadband_neg}  "
            "(below this the motor stays idle)")
        db_label.setStyleSheet(
            f"color: {C['on_surface_variant']}; font-size: 11px;")
        panel.add_row(db_label)

        grid = QGridLayout()
        grid.setSpacing(6)
        for i, (label, (axis, direction)) in enumerate(AXIS_LABELS.items()):
            btn = QPushButton(label)
            btn.pressed.connect(
                lambda a=axis, d=direction: self._motion(a, d, True))
            btn.released.connect(
                lambda a=axis, d=direction: self._motion(a, d, False))
            grid.addWidget(btn, i // 3, i % 3)
        grid.setRowStretch(2, 1)
        panel.add_row(self._wrap(grid))

        direct = QHBoxLayout()
        self.spins = {}
        for m in MOTORS:
            col = QVBoxLayout()
            col.addWidget(caps_label(m), 0, Qt.AlignHCenter)
            spin = QSpinBox()
            spin.setRange(-1000, 1000)
            spin.setMinimumWidth(56)
            spin.valueChanged.connect(
                lambda v, name=m: self._direct(name, v))
            self.spins[m] = spin
            col.addWidget(spin)
            direct.addLayout(col)
        direct.addStretch(1)
        panel.add_row(self._wrap(direct))
        return panel

    def _build_log(self):
        panel = GlassPanel("LOG")
        panel.setMinimumHeight(180)
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        panel.add_row(self.log_box, 1)
        return panel

    # ------------------------------------------------------------ state

    def _log(self, text):
        stamp = time.strftime("%H:%M:%S")
        self.log_box.appendPlainText(f"[{stamp}] {text}")

    def _refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItems(list_ports())

    def _toggle_connect(self):
        if self.ser is not None:
            self._disconnect("disconnected")
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=1)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
        except Exception as exc:
            self.ser = None
            self.fs_pill.set_status("bad")
            self.fs_pill.setText("OPEN FAIL")
            self._log(f"Cannot open {port}: {exc}")
            return
        self.seq = 1
        self.targets = {m: 0 for m in MOTORS}
        self.applied = {m: 0 for m in MOTORS}
        self.last_status = None
        self.last_ack = None
        self.estop_latched = False
        self.estop_observed = False
        self._held.clear()
        self.connect_btn.setText("DISCONNECT")
        self._log(f"Connected to {port} @ {baud} - feeding zeros (motors neutral)")
        self.timer.start(50)

    def _disconnect(self, why="disconnected"):
        self.timer.stop()
        self._held.clear()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.connect_btn.setText("CONNECT")
        self.estop_btn.setEnabled(True)
        self._log(f"Serial {why} - motors will failsafe-stop within 250 ms")

    # ------------------------------------------------------------ commands

    def _motion(self, axis, direction, pressed):
        key = (axis, direction)
        if pressed:
            self._held.add(key)
        else:
            self._held.discard(key)
        if self.ser is None:
            self._log("not connected")
            return
        if not self._held:
            self.targets = {m: 0 for m in MOTORS}
            self._sync_spins()
            self._log("hold released -> all motors 0")
            return
        axes = {}
        v = self.speed_slider.value()
        for held_axis, held_direction in self._held:
            axes[held_axis] = axes.get(held_axis, 0.0) + held_direction * v / 1000.0
        mv = motion_to_motors(
            axes,
            mixing=self.mixing,
            polarity=self.polarity,
        )
        for m in MOTORS:
            self.targets[m] = apply_deadband(
                mv[m], self.deadband_pos, self.deadband_neg)
        self._sync_spins()
        desc = "  ".join(f"{m}:{self.targets[m]:+4d}" for m in MOTORS)
        self._log(f"hold {axis}{' +' if direction > 0 else ' -'} -> {desc}")

    def _direct(self, name, value):
        if self.ser is None:
            return
        self.targets[name] = apply_deadband(
            int(round(value * self.polarity.get(name, 1))),
            self.deadband_pos,
            self.deadband_neg,
        )
        self._log(f"cmd {name} -> {self.targets[name]:+5d}")

    def _zero(self):
        if self.ser is None:
            return
        self._held.clear()
        self.targets = {m: 0 for m in MOTORS}
        self._sync_spins()
        self._log("cmd zero -> all motors 0")

    def _estop(self):
        if self.ser is None:
            return
        if self.estop_latched:
            self._log("E-STOP already latched - power-cycle the ESP32 to clear")
            return
        ans = QMessageBox.question(
            self,
            "E-STOP",
            "Send E-STOP?\n\nLatches on the ESP32 until power-cycle.\n"
            "All motor commands will be rejected until then.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if ans != QMessageBox.StandardButton.Yes:
            self._log("E-STOP aborted")
            return
        self._held.clear()
        try:
            self._sync_seq()
            self.ser.write(proto.build_estop(self.seq))
            self.seq = (self.seq + 1) & 0xFFFF or 1
        except Exception as exc:
            self._log(f"E-STOP write failed: {exc}")
            self._disconnect("write failed after E-STOP")
            return
        self.estop_latched = True
        self.es_pill.set_status("bad")
        self.es_pill.setText("E-STOP LATCHED")
        self._log("E-STOP SENT - latched on ESP32 until power-cycle")

    def _sync_spins(self):
        for m in MOTORS:
            self.spins[m].blockSignals(True)
            self.spins[m].setValue(self.targets[m])
            self.spins[m].blockSignals(False)

    # ------------------------------------------------------------ serial loop

    def _tick(self):
        if self.ser is None:
            return
        try:
            self._drain()
            if self.estop_latched:
                return
            self._sync_seq()
            step = max(1, int(round(float(self.config.get("ramp", 0.1)) * 1000)))
            for m in MOTORS:
                cur, tgt = self.applied[m], self.targets[m]
                if tgt > cur:
                    self.applied[m] = min(tgt, cur + step)
                else:
                    self.applied[m] = max(tgt, cur - step)
            values = [self.applied[m] for m in MOTORS]
            values = [
                apply_deadband(v, self.deadband_pos, self.deadband_neg)
                for v in values
            ]
            self.ser.write(proto.build_motor_command(self.seq, *values))
            self.seq = (self.seq + 1) & 0xFFFF or 1
        except Exception as exc:
            self._log(f"SERIAL ERROR: {exc}")
            self._disconnect("serial error")

    def _decode_seq(self, seq_a, seq_b):
        diff = (seq_a - seq_b) & 0xFFFF
        return diff != 0 and diff < 0x8000

    def _sync_seq(self):
        """Keep our frame sequence newer than the ESP32's last accepted one.

        The ESP32 firmware rejects frames whose sequence is not strictly
        newer than its last accepted sequence. After a reconnect (or a fresh
        tool start against an already-running ESP32) our counter starts at 1,
        which is older, so sync from the first STATUS frame instead.
        """
        st = self.last_status
        if st is None:
            return
        esp = st.get("sequence", 0)
        if not self._decode_seq(self.seq, esp):
            self.seq = (esp + 1) & 0xFFFF or 1

    def _drain(self):
        data = self.ser.read(self.ser.in_waiting or 0)
        if not data:
            return
        buf = bytearray(data)
        while True:
            idx = buf.find(bytes([proto.HEADER_1, proto.HEADER_2]))
            if idx > 0:
                del buf[:idx]
                continue
            if idx == -1:
                break
            if len(buf) < 5:
                break
            frame_len = 5 + 2 + buf[4] + 2
            if len(buf) < frame_len:
                break
            parsed = proto.parse_frame(bytes(buf[:frame_len]))
            del buf[:frame_len]
            if parsed is None:
                continue
            mtype, sq, payload = parsed
            if mtype == proto.MSG_ACK:
                ack = proto.parse_ack(payload)
                if ack is None:
                    continue
                code, echo = ack
                name = proto.ACK_NAMES.get(code, "?")
                self.last_ack = (code, name, echo)
                if code == proto.ACK_ESTOP_ACTIVE and not self.estop_latched:
                    self.estop_latched = True
                    self.estop_observed = True
                    self.es_pill.set_status("bad")
                    self.es_pill.setText("E-STOP LATCHED")
                    self.fs_pill.set_status("bad")
                    self.fs_pill.setText("FAILSAFE")
                    self._log("! E-STOP LATCHED on ESP32 - power-cycle to clear")
                elif code != proto.ACK_OK:
                    self.ack_pill.set_status("bad")
                    self.ack_pill.setText(f"ACK {name}")
                    self._log(f"[ACK] seq={echo} status={name}")
                else:
                    self.ack_pill.set_status("ok")
                    self.ack_pill.setText("ACK OK")
            elif mtype == proto.MSG_STATUS:
                self.last_status = proto.parse_status(payload)

        st = self.last_status
        if st is not None:
            if st["estop"]:
                self.estop_latched = True
                self.estop_observed = True
            elif self.estop_observed:
                self.estop_latched = False
                self.estop_observed = False
                self._log("ESP32 E-STOP cleared after power-cycle; explicit re-command required")
            self.fs_pill.set_status("bad" if st["failsafe"] else "ok")
            self.fs_pill.setText("FAILSAFE" if st["failsafe"] else "ARMED")
            self.es_pill.set_status(
                "bad" if st["estop"] or self.estop_latched else "ok")
            self.es_pill.setText(
                "E-STOP LATCHED" if (st["estop"] or self.estop_latched) else "E-STOP OFF")
            self.seq_pill.setText(f"SEQ {st['sequence']}")
            for m in MOTORS:
                self.rows[m].set_value(st["motors"][m])
            now = time.monotonic()
            if now - self._last_log >= 1.0:
                self._last_log = now
                self._log(
                    "STATUS " + "  ".join(
                        f"{m}:{st['motors'][m]:+4d}" for m in MOTORS)
                )


def main():
    app = QApplication(sys.argv)
    apply_theme(app)
    win = MotorGUI()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
