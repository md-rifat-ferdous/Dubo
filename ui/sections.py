"""Workspace pages for the GCS nav rail sections."""

from collections import deque
from copy import deepcopy
import os

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from modules.controller.controller import MotionState
from modules.thrusters.mixer import MOTOR_NAMES
from ui.glass import GlassPanel, SectionPage, StatusPill, ValueRow, caps_label


AXIS_LABELS = (
    ("SURGE -", "surge", -1.0),
    ("SURGE +", "surge", 1.0),
    ("SWAY -", "sway", -1.0),
    ("SWAY +", "sway", 1.0),
    ("HEAVE -", "heave", -1.0),
    ("HEAVE +", "heave", 1.0),
    ("YAW -", "yaw", -1.0),
    ("YAW +", "yaw", 1.0),
    ("PITCH -", "pitch", -1.0),
    ("PITCH +", "pitch", 1.0),
    ("ROLL -", "roll", -1.0),
    ("ROLL +", "roll", 1.0),
)


def _wrap(layout):
    widget = QWidget()
    widget.setLayout(layout)
    return widget


def _motor_values(status):
    source = status.get("status", {}).get("motors", {}) if isinstance(status, dict) else {}
    if not source:
        source = status.get("applied", {}) if isinstance(status, dict) else {}
    result = {}
    for name in MOTOR_NAMES:
        value = source.get(name, 0)
        if isinstance(value, float) and abs(value) <= 1.0:
            value = int(round(value * 1000))
        result[name] = int(value or 0)
    return result


class SensorsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        nav_panel = GlassPanel("CORE NAVIGATION")
        self.depth = ValueRow("DEPTH", "--", highlight=True)
        self.heading = ValueRow("HEADING", "--")
        self.pos_x = ValueRow("POSITION X", "--")
        self.pos_y = ValueRow("POSITION Y", "--")
        for row in (self.depth, self.heading, self.pos_x, self.pos_y):
            nav_panel.add_row(row)

        att_panel = GlassPanel("ATTITUDE")
        self.pitch = ValueRow("PITCH", "--")
        self.roll = ValueRow("ROLL", "--")
        self.boost = ValueRow("BOOST", "OFF")
        for row in (self.pitch, self.roll, self.boost):
            att_panel.add_row(row)

        power = GlassPanel("POWER")
        self.bus_v = ValueRow("BUS VOLTAGE", "--")
        self.bus_i = ValueRow("TOTAL CURRENT", "--")
        self.temp = ValueRow("INT TEMP", "--")
        self.leak = ValueRow("LEAK SENSOR", "SAFE")
        for row in (self.bus_v, self.bus_i, self.temp, self.leak):
            power.add_row(row)

        self.health = GlassPanel("SENSOR HEALTH")
        health_layout = QHBoxLayout()
        health_layout.setSpacing(8)
        self._health_pills = {}
        for name in ("IMU", "DEPTH", "CAMERA", "LINK"):
            pill = StatusPill(name, "info")
            self._health_pills[name] = pill
            health_layout.addWidget(pill)
        health_layout.addStretch(1)
        self.health.add_row(_wrap(health_layout))

        page.add(nav_panel)
        page.add(att_panel)
        page.add(power)
        page.add(self.health)

    def update(self):
        st = self.ctx["sensors"]
        if st is None:
            return
        data = st.get_sensor_data()
        valid = data["imu_valid"]
        # Depth only exists when the active provider supplies it; the IMU alone
        # must not render a convincing constant 0.00 m.
        depth_text = f"{data['depth']:5.2f} m" if data["depth_valid"] else "--"
        self.depth.set_value(depth_text)
        self.heading.set_value(f"{data['yaw']:03.0f} deg")
        self.pos_x.set_value(f"{data['x']:5.2f} m")
        self.pos_y.set_value(f"{data['y']:5.2f} m")
        # Sensed attitude from the IMU, not the commanded stick input.
        self.pitch.set_value(f"{data['pitch']:+05.1f} deg" if valid else "--")
        self.roll.set_value(f"{data['roll']:+05.1f} deg" if valid else "--")
        ctrl = self.ctx["controller"]
        if ctrl is not None:
            self.boost.set_value("ON" if ctrl.last_motion.boost else "OFF")
        self._health_pills["IMU"].set_status("ok" if valid else "bad")
        self._health_pills["DEPTH"].set_status(
            "ok" if data["depth_valid"] else "warn"
        )
        for name in ("CAMERA", "LINK"):
            self._health_pills[name].set_status("ok")


class DiagnosticsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        self.health_panel = GlassPanel("SUBSYSTEM HEALTH")
        self.health_grid = QGridLayout()
        self.health_grid.setSpacing(8)
        self.health_panel.body().addLayout(self.health_grid)
        self._health_pills = {}

        perf = GlassPanel("COMPUTE CORE LOAD")
        self.fps = ValueRow("FPS", "--")
        self.cpu = ValueRow("CPU LOAD", "--")
        self.ram = ValueRow("MEMORY ALLOC", "--")
        self.uptime = ValueRow("UPTIME", "--")
        for row in (self.fps, self.cpu, self.ram, self.uptime):
            perf.add_row(row)

        umb = GlassPanel("UMBILICAL")
        self.latency = ValueRow("LATENCY", "--")
        self.link = ValueRow("LINK STATE", "--")
        for row in (self.latency, self.link):
            umb.add_row(row)

        self.motor_panel = GlassPanel("MOTOR LINK")
        self.motor_provider = ValueRow("PROVIDER", "--")
        self.motor_port = ValueRow("SERIAL PORT", "--")
        self.motor_ready = ValueRow("ESP32 READY", "--")
        self.motor_failsafe = ValueRow("FAILSAFE", "--")
        self.motor_estop = ValueRow("E-STOP", "--")
        self.motor_sequence = ValueRow("SEQUENCE", "--")
        self.motor_ack = ValueRow("LAST ACK", "--")
        for row in (
            self.motor_provider,
            self.motor_port,
            self.motor_ready,
            self.motor_failsafe,
            self.motor_estop,
            self.motor_sequence,
            self.motor_ack,
        ):
            self.motor_panel.add_row(row)
        self.motor_rows = {}
        motor_values = QHBoxLayout()
        motor_values.setSpacing(6)
        for name in MOTOR_NAMES:
            row = ValueRow(name, "0")
            self.motor_rows[name] = row
            motor_values.addWidget(row)
        self.motor_panel.add_row(_wrap(motor_values))

        page.add(self.health_panel)
        page.add(perf)
        page.add(umb)
        page.add(self.motor_panel)

    def update(self):
        sm = self.ctx["service_manager"]
        if sm is None:
            return
        health = sm.health_check()
        names = set(list(self._health_pills) + list(health))
        for col, name in enumerate(sorted(names)):
            if name not in self._health_pills:
                pill = StatusPill(name, "info")
                self.health_grid.addWidget(pill, 0, col)
                self._health_pills[name] = pill
            ok = health.get(name)
            self._health_pills[name].set_status("ok" if ok else ("bad" if ok is False else "info"))
        self.fps.set_value(self.ctx.get("fps")())
        cpu, ram = self._load()
        self.cpu.set_value(cpu)
        self.ram.set_value(ram)
        self.uptime.set_value(self.ctx.get("uptime")())
        status = self._thruster_status()
        if not status:
            self.motor_provider.set_value("--")
            self.motor_port.set_value("--")
            self.motor_ready.set_value("--")
            self.motor_failsafe.set_value("--")
            self.motor_estop.set_value("--")
            self.motor_sequence.set_value("--")
            self.motor_ack.set_value("--")
            for row in self.motor_rows.values():
                row.set_value("0")
            return
        self.motor_provider.set_value(str(status.get("manager_provider", status.get("provider", "--"))).upper())
        port = status.get("port")
        self.motor_port.set_value(f"{port} @ {status.get('baud')}" if port else "SIMULATED")
        self.motor_ready.set_value("YES" if status.get("ready") else "NO")
        self.motor_failsafe.set_value("ACTIVE" if status.get("failsafe") else "CLEAR")
        self.motor_estop.set_value("LATCHED" if status.get("estop") else "CLEAR")
        self.motor_sequence.set_value(status.get("sequence", "--"))
        ack = status.get("last_ack") or {}
        self.motor_ack.set_value(ack.get("name", "SIMULATED") if isinstance(ack, dict) else str(ack))
        values = _motor_values(status)
        for name, row in self.motor_rows.items():
            row.set_value(f"{values[name]:+5d}")

    def _thruster_status(self):
        thrusters = self.ctx.get("thrusters")
        if thrusters is None:
            return None
        try:
            return thrusters.status()
        except Exception:
            return None

    @staticmethod
    def _load():
        try:
            import psutil

            return f"{psutil.cpu_percent():.0f}%", f"{psutil.virtual_memory().percent:.0f}%"
        except Exception:
            return "--", "--"


class LogsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        self.mission_path = os.path.join(ctx["log_dir"], "mission.log")
        self.system_path = os.path.join(ctx["log_dir"], "system.log")
        self.motor_path = os.path.join(ctx["log_dir"], "motor.log")

        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        mission = GlassPanel("MISSION LOG (TELEMETRY)")
        self.mission_box = QTextEdit()
        self.mission_box.setReadOnly(True)
        mission.add_row(self.mission_box)

        system = GlassPanel("SYSTEM LOG")
        self.system_box = QTextEdit()
        self.system_box.setReadOnly(True)
        system.add_row(self.system_box)

        motor = GlassPanel("MOTOR EVENTS")
        self.motor_box = QTextEdit()
        self.motor_box.setReadOnly(True)
        motor.add_row(self.motor_box)

        refresh = QPushButton("REFRESH LOGS")
        refresh.clicked.connect(self.update)
        page.add(mission)
        page.add(system)
        page.add(motor)
        page.add(refresh)

    @staticmethod
    def _tail(path, n=80):
        try:
            with open(path) as stream:
                return "".join(deque(stream, n))
        except FileNotFoundError:
            return "(no log file yet)\n"

    def update(self):
        self.mission_box.setPlainText(self._tail(self.mission_path))
        self.system_box.setPlainText(self._tail(self.system_path))
        self.motor_box.setPlainText(self._tail(self.motor_path))


class SettingsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        cm = ctx.get("config_manager")
        if cm is None:
            page.add(GlassPanel("CONFIG"))
            return
        self.cm = cm
        self._initial = deepcopy(cm.get_section("thrusters") or {})
        panel = GlassPanel("THRUSTER CONFIGURATION")
        self.provider = QComboBox()
        self.provider.addItems(["uart", "simulated"])
        self.provider.setCurrentText(str(self._initial.get("provider", "uart")))
        self.port = QLineEdit(str(self._initial.get("port", "/dev/ttyUSB0")))
        self.baud = QLineEdit(str(self._initial.get("baud", 115200)))
        self.ramp = QLineEdit(str(self._initial.get("ramp", 0.1)))
        self.boost = QLineEdit(str(self._initial.get("boost_scale", 1.0)))
        self.deadband_pos = QSpinBox()
        self.deadband_pos.setRange(0, 500)
        self.deadband_pos.setValue(int((self._initial.get("deadband") or {}).get("positive", 20)))
        self.deadband_neg = QSpinBox()
        self.deadband_neg.setRange(0, 500)
        self.deadband_neg.setValue(abs(int((self._initial.get("deadband") or {}).get("negative", 176))))
        self.fallback = QCheckBox("ALLOW SIMULATED FALLBACK")
        self.fallback.setChecked(bool(self._initial.get("fallback_to_simulated", True)))
        for label, widget in (
            ("PROVIDER", self.provider),
            ("SERIAL PORT", self.port),
            ("BAUD", self.baud),
            ("RAMP", self.ramp),
            ("BOOST SCALE", self.boost),
            ("POSITIVE DEADBAND", self.deadband_pos),
            ("NEGATIVE DEADBAND MAGNITUDE", self.deadband_neg),
        ):
            row = QHBoxLayout()
            row.setContentsMargins(8, 4, 8, 4)
            row.addWidget(caps_label(label))
            row.addStretch(1)
            row.addWidget(widget)
            panel.add_row(_wrap(row))
        panel.add_row(self.fallback)

        polarity = GlassPanel("MOTOR POLARITY")
        self.polarity = {}
        for name in MOTOR_NAMES:
            spin = QSpinBox()
            spin.setRange(-1, 1)
            spin.setSingleStep(2)
            spin.setValue(int((self._initial.get("polarity") or {}).get(name, 1)))
            self.polarity[name] = spin
            row = QHBoxLayout()
            row.setContentsMargins(8, 4, 8, 4)
            row.addWidget(caps_label(name))
            row.addStretch(1)
            row.addWidget(spin)
            polarity.add_row(_wrap(row))

        mixing = GlassPanel("MIXING")
        mixing.add_row(ValueRow("MIXING", str(self._initial.get("mixing", {}))))
        mixing.add_row(caps_label("Edit mixing in configs/thrusters.yaml"))

        self.status = QLabel("")
        self.status.setStyleSheet("color: #64ffda; padding: 4px 8px;")
        apply = QPushButton("APPLY THRUSTER SETTINGS")
        apply.setObjectName("Primary")
        apply.clicked.connect(self._save)
        page.add(panel)
        page.add(polarity)
        page.add(mixing)
        page.add(self.status)
        page.add(apply)

    def _save(self):
        try:
            baud = int(self.baud.text().strip())
            ramp = float(self.ramp.text().strip())
            boost = float(self.boost.text().strip())
            if baud <= 0:
                raise ValueError("baud must be positive")
            if not 0 <= ramp <= 1:
                raise ValueError("ramp must be between 0 and 1")
            if not 0 <= boost <= 1:
                raise ValueError("boost scale must be between 0 and 1")
            polarity = {name: spin.value() for name, spin in self.polarity.items()}
            if any(value not in (-1, 1) for value in polarity.values()):
                raise ValueError("polarity must be -1 or 1")
            data = dict(self._initial)
            data.update(
                {
                    "provider": self.provider.currentText(),
                    "port": self.port.text().strip(),
                    "baud": baud,
                    "ramp": ramp,
                    "boost_scale": boost,
                    "deadband": {
                        "positive": self.deadband_pos.value(),
                        "negative": self.deadband_neg.value(),
                    },
                    "polarity": polarity,
                    "fallback_to_simulated": self.fallback.isChecked(),
                }
            )
            self.cm.update_section("thrusters", data)
            self._initial = deepcopy(data)
            self.status.setText("SAVED - restart the application to apply the new thruster provider")
        except (TypeError, ValueError) as exc:
            self.status.setText(f"NOT SAVED: {exc}")
        except Exception as exc:
            self.status.setText(f"NOT SAVED: {exc}")


class OperationsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        self._held = set()
        self._direct = {name: 0.0 for name in MOTOR_NAMES}
        self._hold_timer = QTimer(self)
        self._hold_timer.setInterval(250)
        self._hold_timer.timeout.connect(self._refresh_hold)
        self._hold_timer.start()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        status = GlassPanel("MOTOR SYSTEM STATUS")
        self.mode = ValueRow("MODE", "DISARMED")
        self.link_pill_row = QWidget()
        row = QHBoxLayout(self.link_pill_row)
        row.setContentsMargins(0, 0, 0, 0)
        self.link_pill = StatusPill("MOTOR LINK", "info")
        self.armed_pill = StatusPill("DISARMED", "warn")
        self.ready_pill = StatusPill("ESP32 --", "info")
        self.failsafe_pill = StatusPill("FAILSAFE --", "info")
        self.estop_pill = StatusPill("E-STOP --", "info")
        for pill in (self.link_pill, self.armed_pill, self.ready_pill, self.failsafe_pill, self.estop_pill):
            row.addWidget(pill)
        row.addStretch(1)
        self.provider_row = ValueRow("PROVIDER", "--")
        self.sequence_row = ValueRow("SEQUENCE", "--")
        self.ack_row = ValueRow("LAST ACK", "--")
        status.add_row(self.mode)
        status.add_row(self.link_pill_row)
        status.add_row(self.provider_row)
        status.add_row(self.sequence_row)
        status.add_row(self.ack_row)

        cmd = GlassPanel("COMMAND CHANNEL")
        self.cmd_input = QTextEdit()
        self.cmd_input.setMaximumHeight(64)
        self.cmd_input.setPlaceholderText("e.g. FORWARD 0.5")
        send = QPushButton("SEND COMMAND")
        send.clicked.connect(self._send)
        cmd.add_row(self.cmd_input)
        cmd.add_row(send)

        control = GlassPanel("MAINTENANCE MOVEMENT")
        speed_row = QHBoxLayout()
        self.speed_label = QLabel("300")
        self.speed = QSlider(Qt.Orientation.Horizontal)
        self.speed.setRange(0, 1000)
        self.speed.setValue(300)
        self.speed.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.speed.valueChanged.connect(self._speed_changed)
        speed_row.addWidget(caps_label("SPEED"))
        speed_row.addWidget(self.speed, 1)
        speed_row.addWidget(self.speed_label)
        control.add_row(_wrap(speed_row))
        axis_grid = QGridLayout()
        axis_grid.setSpacing(5)
        self._axis_buttons = {}
        for index, (label, axis, direction) in enumerate(AXIS_LABELS):
            button = QPushButton(label)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            button.pressed.connect(lambda a=axis, d=direction: self._hold(a, d, True))
            button.released.connect(lambda a=axis, d=direction: self._hold(a, d, False))
            self._axis_buttons[(axis, direction)] = button
            axis_grid.addWidget(button, index // 3, index % 3)
        control.add_row(_wrap(axis_grid))
        direct_row = QHBoxLayout()
        self._direct_spins = {}
        for name in MOTOR_NAMES:
            column = QVBoxLayout()
            spin = QSpinBox()
            spin.setRange(-1000, 1000)
            spin.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            spin.setValue(0)
            spin.valueChanged.connect(lambda value, motor=name: self._direct_changed(motor, value))
            self._direct_spins[name] = spin
            column.addWidget(caps_label(name), 0, Qt.AlignHCenter)
            column.addWidget(spin)
            direct_row.addLayout(column)
        control.add_row(_wrap(direct_row))
        actions = QHBoxLayout()
        zero = QPushButton("ZERO")
        zero.clicked.connect(self._zero)
        self.estop_button = QPushButton("E-STOP")
        self.estop_button.setObjectName("Danger")
        self.estop_button.clicked.connect(self._estop)
        refresh = QPushButton("REFRESH STATUS")
        refresh.clicked.connect(self.update)
        actions.addWidget(zero)
        actions.addWidget(self.estop_button)
        actions.addWidget(refresh)
        control.add_row(_wrap(actions))
        warning = QLabel("Hold movement buttons to move. Release to zero. Test commands expire automatically.")
        warning.setStyleSheet("color: #ffd364; padding: 4px 8px;")
        control.add_row(warning)

        page.add(status)
        page.add(cmd)
        page.add(control)

    def _controller(self):
        return self.ctx.get("controller")

    def _armed(self):
        controller = self._controller()
        return bool(controller and getattr(controller, "armed", False))

    def _speed_changed(self, value):
        self.speed_label.setText(str(int(value)))
        self._refresh_hold()

    def _hold(self, axis, direction, pressed):
        key = (axis, direction)
        if pressed:
            if self._armed():
                self._held.add(key)
        else:
            self._held.discard(key)
        self._refresh_hold()

    def _refresh_hold(self):
        controller = self._controller()
        if controller is None or not self._armed():
            self._held.clear()
            return
        if not self._held:
            if not any(abs(value) > 1e-9 for value in self._direct.values()):
                try:
                    test_active = bool(controller.status().get("test_active"))
                except Exception:
                    test_active = True
                if test_active:
                    controller.clear_test()
            return
        motion = MotionState(boost=False)
        speed = self.speed.value() / 1000.0
        for axis in ("surge", "sway", "heave", "yaw", "pitch", "roll"):
            value = 0.0
            if (axis, 1.0) in self._held:
                value += speed
            if (axis, -1.0) in self._held:
                value -= speed
            setattr(motion, axis, max(-1.0, min(1.0, value)))
        controller.set_test_motion(motion, duration=0.6)

    def _direct_changed(self, name, value):
        self._direct[name] = value / 1000.0
        controller = self._controller()
        if controller is not None and self._armed():
            controller.set_test_motor_values(self._direct, duration=2.0)

    def _reset_direct_controls(self):
        self._direct = {name: 0.0 for name in MOTOR_NAMES}
        for spin in self._direct_spins.values():
            spin.blockSignals(True)
            spin.setValue(0)
            spin.blockSignals(False)

    def _zero(self):
        self._held.clear()
        self._reset_direct_controls()
        controller = self._controller()
        if controller is not None:
            controller.zero()

    def _send(self):
        text = self.cmd_input.toPlainText().strip()
        telemetry = self.ctx.get("telemetry")
        if text and telemetry is not None and telemetry.inject_command(text):
            self.cmd_input.clear()

    def _estop(self):
        answer = QMessageBox.question(
            self,
            "E-STOP",
            "Send E-STOP? It remains latched until the ESP32 is power-cycled.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer == QMessageBox.StandardButton.Yes:
            controller = self._controller()
            if controller is not None:
                controller.emergency_stop()
            self._zero()

    def _clear_override(self):
        self._held.clear()
        self._reset_direct_controls()
        controller = self._controller()
        if controller is not None:
            clear = getattr(controller, "clear_test", None)
            if callable(clear):
                clear()

    def clear_controls(self):
        self._clear_override()

    def hideEvent(self, event):
        self._clear_override()
        super().hideEvent(event)

    def update(self):
        armed = self._armed()
        if not armed:
            self._held.clear()
            if any(abs(value) > 1e-9 for value in self._direct.values()):
                self._reset_direct_controls()
        self.mode.set_value("MANUAL" if armed else "DISARMED")
        self.armed_pill.setText("ARMED" if armed else "DISARMED")
        self.armed_pill.set_status("ok" if armed else "warn")
        thrusters = self.ctx.get("thrusters")
        status = {}
        if thrusters is not None:
            try:
                status = thrusters.status()
            except Exception:
                status = {}
        fallback = bool(status.get("simulated_fallback"))
        link_ok = bool(status.get("responding"))
        if fallback:
            self.link_pill.setText("SIMULATED FALLBACK")
            self.link_pill.set_status("warn")
        else:
            self.link_pill.setText("MOTOR LINK OK" if link_ok else "MOTOR LINK DOWN")
            self.link_pill.set_status("ok" if link_ok else "bad")
        ready = bool(status.get("ready"))
        if fallback:
            self.ready_pill.setText("SIMULATION MODE")
            self.ready_pill.set_status("warn")
        else:
            self.ready_pill.setText("ESP32 READY" if ready else "ESP32 NOT READY")
            self.ready_pill.set_status("ok" if ready else "bad")
        failsafe = bool(status.get("failsafe"))
        self.failsafe_pill.setText("FAILSAFE" if failsafe else "FAILSAFE CLEAR")
        self.failsafe_pill.set_status("bad" if failsafe else "ok")
        estop = bool(status.get("estop"))
        self.estop_pill.setText("E-STOP LATCHED" if estop else "E-STOP CLEAR")
        self.estop_pill.set_status("bad" if estop else "ok")
        provider = status.get("manager_provider", status.get("provider", "--"))
        self.provider_row.set_value(str(provider).upper())
        self.sequence_row.set_value(status.get("sequence", "--"))
        ack = status.get("last_ack") or "--"
        self.ack_row.set_value(ack.get("name", "--") if isinstance(ack, dict) else str(ack))
        enabled = armed and thrusters is not None and ready
        for button in self._axis_buttons.values():
            button.setEnabled(enabled)
        self.speed.setEnabled(enabled)
        for spin in self._direct_spins.values():
            spin.setEnabled(enabled)


class NavigationPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        hold = GlassPanel("HOLD CONTROLS (FUTURE)")
        self.depth_hold = QCheckBox("DEPTH HOLD")
        self.depth_set = QSpinBox()
        self.depth_set.setRange(0, 500)
        self.depth_set.setValue(10)
        self.depth_set.setSuffix(" m")
        dh = QHBoxLayout()
        dh.addWidget(self.depth_hold)
        dh.addWidget(self.depth_set)
        dh.addStretch(1)
        self.heading_hold = QCheckBox("HEADING HOLD")
        self.hdg_set = QSpinBox()
        self.hdg_set.setRange(0, 359)
        self.hdg_set.setValue(0)
        self.hdg_set.setSuffix(" deg")
        hh = QHBoxLayout()
        hh.addWidget(self.heading_hold)
        hh.addWidget(self.hdg_set)
        hh.addStretch(1)
        hold.add_row(_wrap(dh))
        hold.add_row(_wrap(hh))
        page.add(hold)
        wp = GlassPanel("WAYPOINTS")
        wp.add_row(caps_label("No waypoints yet"))
        page.add(wp)


class ManipulatorPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        arm = GlassPanel("MANIPULATOR ARM (FUTURE)")
        btns = QHBoxLayout()
        open_b = QPushButton("CLAW OPEN")
        close_b = QPushButton("CLAW CLOSE")
        open_b.setEnabled(False)
        close_b.setEnabled(False)
        btns.addWidget(open_b)
        btns.addWidget(close_b)
        btns.addStretch(1)
        arm.add_row(_wrap(btns))
        arm.add_row(ValueRow("JAW POSITION", "--"))
        hyd = GlassPanel("HYDRAULICS")
        hyd.add_row(ValueRow("PRESSURE", "--"))
        hyd.add_row(ValueRow("TEMP", "--"))
        hyd.add_row(ValueRow("FLOW RATE", "--"))
        page.add(arm)
        page.add(hyd)


class AIVisionPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        det = GlassPanel("OBJECT DETECTION (FUTURE)")
        enable = QCheckBox("ENABLE DETECTION")
        enable.setEnabled(False)
        det.add_row(enable)
        det.add_row(ValueRow("MODEL STATUS", "NOT LOADED"))
        det.add_row(ValueRow("INFERENCE FPS", "--"))
        page.add(det)
        results = GlassPanel("DETECTIONS")
        results.add_row(ValueRow("PIPELINE JOINT", "92%"))
        results.add_row(ValueRow("L. PERTUSA", "88%"))
        results.add_row(ValueRow("UNKNOWN OBJ", "64%"))
        page.add(results)


class PlannerPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        page.add(GlassPanel("MISSION BLOCKS (FUTURE)"))
        seq = GlassPanel("SEQUENCE LOGIC")
        seq.add_row(caps_label("No mission loaded"))
        page.add(seq)
        val = GlassPanel("PLAN VALIDATION")
        val.add_row(ValueRow("STATUS", "--"))
        page.add(val)


def build(ctx):
    pages = {}
    for key, cls in (
        ("operations", OperationsPage),
        ("navigation", NavigationPage),
        ("sensors", SensorsPage),
        ("manipulator", ManipulatorPage),
        ("diagnostics", DiagnosticsPage),
        ("planner", PlannerPage),
        ("ai_vision", AIVisionPage),
        ("logs", LogsPage),
        ("settings", SettingsPage),
    ):
        pages[key] = cls(ctx)
    return pages
