import os
import shutil
import time

import cv2
import numpy as np
from PySide6.QtCore import QEvent, QSize, Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QSizePolicy,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from modules.controller.controller import MotionState
from ui.control_pad import ControlPad3D, FullMotionController
from ui.glass import GlassPanel, NavButton, StatusPill, caps_label
from ui.sections import build as build_sections
from ui.sonar import SonarPanel
from ui.theme import C, FONT_DATA

NAV_ITEMS = [
    ("dashboard", "Dashboard", "\u2302"),
    ("operations", "Operations", "\u25ce"),
    ("navigation", "Navigation", "\u2794"),
    ("sensors", "Sensors", "\u2733"),
    ("manipulator", "Manipulator", "\u2699"),
    ("diagnostics", "Diagnostics", "\u25a4"),
    ("planner", "Planner", "\u25ba"),
    ("ai_vision", "AI Vision", "\u2726"),
    ("logs", "Logs", "\u2630"),
    ("settings", "Settings", "\u25c8"),
]

_TYPING_WIDGETS = (QLineEdit, QTextEdit, QSpinBox, QComboBox, QSlider)


class MainWindow(QMainWindow):
    def __init__(self, camera_manager, overlay=None, hud_provider=None,
                 controller=None, keymap=None, service_manager=None,
                 config_manager=None, telemetry=None, thrusters=None,
                 log_dir="logs", gcs_config=None):
        super().__init__()
        self.camera = camera_manager
        self.overlay = overlay
        self.hud_provider = hud_provider
        self.controller = controller
        self.telemetry = telemetry
        self.thrusters = thrusters
        self.log_dir = log_dir
        self.gcs = gcs_config or {}
        self.key_speed = 1.0
        self._pressed_actions = set()
        self._clearing_motion_inputs = False
        self._key_actions = self._build_keymap(keymap or {})
        self._armed = False
        self._recording = False
        self._writer = None
        self._lights = False
        self._frames = 0
        self._fps = 0.0
        self._fps_t = time.time()
        self._boot_t = time.time()
        self._cam_size = (0, 0)
        self._cam_signal = None

        self.setWindowTitle("HYDRION SPECTRA - ROV Controller")
        self.resize(1440, 820)

        self._ctx = {
            "service_manager": service_manager,
            "config_manager": config_manager,
            "sensors": hud_provider,
            "controller": controller,
            "telemetry": telemetry,
            "thrusters": thrusters,
            "log_dir": log_dir,
            "fps": lambda: f"{self._fps:.0f}",
            "uptime": self._uptime_str,
            "armed": lambda: self._armed,
            "link_ok": lambda: self._link_ok(),
        }

        self._build_ui()
        self._build_sections()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_camera)
        self.timer.start(33)

        self.refresh = QTimer(self)
        self.refresh.timeout.connect(self._refresh_static)
        self.refresh.start(1000)

        QApplication.instance().installEventFilter(self)

    def _build_keymap(self, keymap):
        actions = {}
        for action, name in keymap.items():
            if name is None:
                continue
            key = getattr(Qt.Key, f"Key_{name}", None)
            if key is not None:
                actions[key] = action
        return actions

    def _link_ok(self):
        if self.telemetry is None:
            return False
        return getattr(self.telemetry, "link", None) is not None

    def _uptime_str(self):
        t = int(time.time() - self._boot_t)
        return f"{t // 3600:02d}:{t % 3600 // 60:02d}"

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addWidget(self._build_appbar())
        body = QWidget()
        body_lay = QHBoxLayout(body)
        body_lay.setContentsMargins(0, 0, 0, 0)
        body_lay.setSpacing(0)
        self.rail = self._build_rail()
        body_lay.addWidget(self.rail)
        self.stack = QStackedWidget()
        body_lay.addWidget(self.stack, 1)
        root.addWidget(body, 1)
        root.addWidget(self._build_footer())

        self.setCentralWidget(central)

    def _build_appbar(self):
        bar = QFrame()
        bar.setFixedHeight(46)
        bar.setStyleSheet(
            f"background: rgba(4, 19, 41, 0.85); border-bottom: 1px solid "
            f"rgba(100, 255, 218, 0.22);"
        )
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(20, 0, 16, 0)
        brand = QLabel("HYDRION SPECTRA")
        brand.setObjectName("Brand")
        lay.addWidget(brand)
        workspace = QLabel("WORKSPACE")
        workspace.setStyleSheet(
            f"color: {C['primary']}; font-weight: 700; font-size: 12px; "
            f"letter-spacing: 2px; border-bottom: 2px solid {C['primary']};"
        )
        lay.addSpacing(28)
        lay.addWidget(workspace)
        lay.addStretch(1)

        self.batt_pill = StatusPill("BATT --", "info")
        self.link_pill = StatusPill("LINK --", "info")
        self.uptime_pill = StatusPill("UP 00:00", "info")
        self.mode_pill = StatusPill("MANUAL", "ok")
        for pill in (self.batt_pill, self.link_pill, self.uptime_pill, self.mode_pill):
            lay.addWidget(pill)
            lay.addSpacing(6)
        lay.addSpacing(10)
        avatar = QLabel("P")
        avatar.setFixedSize(30, 30)
        avatar.setAlignment(Qt.AlignCenter)
        avatar.setStyleSheet(
            f"background: {C['surface_high']}; color: {C['primary']}; "
            f"border: 1px solid rgba(100, 255, 218, 0.3); border-radius: 15px;"
        )
        lay.addWidget(avatar)
        return bar

    def _build_rail(self):
        rail = _RailFrame()
        rail.expand = self.set_expanded
        self.rail = rail
        rail.setObjectName("Rail")
        rail.setStyleSheet(
            f"QFrame#Rail {{ background: rgba(17, 32, 54, 0.72); "
            f"border-right: 1px solid rgba(100, 255, 218, 0.22); }}"
        )
        rail.setMinimumWidth(56)
        rail.setMaximumWidth(208)
        self._rail_lay = QVBoxLayout(rail)
        self._rail_lay.setContentsMargins(8, 12, 8, 10)
        self._rail_lay.setSpacing(6)

        self.rail_logo = QLabel("\u25a6")  # system glyph
        self.rail_logo.setFixedHeight(34)
        self.rail_logo.setAlignment(Qt.AlignCenter)
        self.rail_logo.setStyleSheet(
            f"background: {C['surface_highest']}; color: {C['primary']}; "
            f"border: 1px solid rgba(100, 255, 218, 0.3); border-radius: 17px;"
        )
        self._rail_lay.addWidget(self.rail_logo, 0, Qt.AlignHCenter)

        self.rail_btns = {}
        for key, title, glyph in NAV_ITEMS:
            btn = NavButton(glyph, title)
            btn.clicked.connect(lambda _=False, k=key: self._set_section(k))
            self._rail_lay.addWidget(btn)
            self.rail_btns[key] = btn

        self._rail_lay.addStretch(1)

        self.launch_btn = QPushButton("\u25ba  LAUNCH MISSION")
        self.launch_btn.setObjectName("Danger")
        self.launch_btn.setCursor(Qt.PointingHandCursor)
        self.launch_btn.clicked.connect(self._launch_mission)
        self._rail_lay.addWidget(self.launch_btn)

        self.set_expanded(False)
        return rail

    def set_expanded(self, expanded):
        self._collapsed = not expanded
        for btn in self.rail_btns.values():
            btn.set_collapsed(not expanded)
        self.launch_btn.setText("\u25ba  LAUNCH MISSION" if expanded else "\u25ba")
        self.rail.set_expanded_state(expanded)

    def _build_footer(self):
        bar = QFrame()
        bar.setFixedHeight(30)
        bar.setStyleSheet(
            f"background: rgba(1, 14, 36, 0.9); border-top: 1px solid "
            f"rgba(100, 255, 218, 0.15);"
        )
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(16, 0, 16, 0)
        cr = QLabel("(c) 2024 HYDRION OFFSHORE SYSTEMS")
        cr.setStyleSheet(f"color: {C['secondary']}; font-size: 11px; letter-spacing: 1px;")
        lay.addWidget(cr)
        lay.addStretch(1)
        self.foot = {}
        for name in ("FPS", "CPU", "RAM", "STORAGE", "DEPTH", "LATENCY"):
            lab = QLabel(f"{name}: --")
            lab.setStyleSheet(
                f"color: {C['on_surface_variant']}; font-size: 11px; "
                f"font-family: {FONT_DATA};"
            )
            lay.addWidget(lab)
            self.foot[name] = lab
            lay.addSpacing(14)
        return bar

    def _build_sections(self):
        pages = build_sections(self._ctx)
        self._pages = {}
        for key, title, glyph in NAV_ITEMS:
            if key == "dashboard":
                widget = self._build_dashboard()
            else:
                widget = pages[key]
            self._pages[key] = widget
            self.stack.addWidget(widget)
        self.rail_btns["dashboard"].setChecked(True)

    def _build_dashboard(self):
        container = QWidget()
        container.setStyleSheet("background: #000000;")
        lay = QVBoxLayout(container)
        lay.setContentsMargins(0, 0, 0, 0)

        self.camera_view = QLabel("LIVE CAMERA FEED")
        self.camera_view.setAlignment(Qt.AlignCenter)
        self.camera_view.setMinimumSize(1, 1)
        self.camera_view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_view.setStyleSheet("background: #000; color: #556; font-weight: bold;")
        lay.addWidget(self.camera_view)
        self.camera_container = container

        self.live = QLabel()
        self.live.setText(" \u25cf LIVE: CH-1 MAIN CAM")
        self.live.setStyleSheet(
            f"background: rgba(17, 32, 54, 0.55); color: {C['on_surface']}; "
            f"border: 1px solid {C['outline_variant']}; border-radius: 8px; "
            f"padding: 3px 10px; font-family: {FONT_DATA}; font-size: 12px;"
        )
        self.live.setParent(container)
        self.live.adjustSize()
        self.live.show()

        self.sonar = SonarPanel(size=self.gcs.get("sonar", {}).get("size", 220),
                                range_m=self.gcs.get("sonar", {}).get("range_m", 6.0),
                                grid_lines=self.gcs.get("sonar", {}).get("grid_lines", 6))
        self.sonar.setParent(container)
        self.sonar.show()

        self.telemetry_panel = GlassPanel("SYSTEM TELEMETRY", parent=container)
        self.telemetry_panel.setFixedWidth(210)
        self.telemetry_panel._values = {}
        tcfg = self.gcs.get("telemetry", {})
        for label, value in (
            ("BUS VOLTAGE", f"{tcfg.get('bus_voltage', 48.2)} V"),
            ("TOTAL CURR", f"{tcfg.get('total_current', 12.4)} A"),
            ("INT TEMP", f"{tcfg.get('int_temp', 24.0)} C"),
        ):
            row = _ValueRowCompact(label, value)
            self.telemetry_panel._values[label] = row
            self.telemetry_panel.add_row(row)
        leak = _ValueRowCompact("LEAK SENSOR", tcfg.get("leak_status", "SAFE"), pill=True)
        self.telemetry_panel._values["LEAK SENSOR"] = leak
        self.telemetry_panel.add_row(leak)
        self.telemetry_panel.adjustSize()
        self.telemetry_panel.show()

        self.flyout = self._build_flyout(container)

        self.dock = self._build_dock(container)

        self.pad = ControlPad3D(container)
        if self.controller is not None:
            self.pad.motionChanged.connect(
                lambda motion: self._set_motion_input("pad", motion)
            )
        self.pad.show()

        self.full_motion = FullMotionController(container)
        if self.controller is not None:
            self.full_motion.motionChanged.connect(
                lambda motion: self._set_motion_input("full_motion", motion)
            )
            self.full_motion.zeroRequested.connect(self._zero_motion)
            self.full_motion.estopRequested.connect(self._confirm_estop)
        self.full_motion.show()

        self.motion_hint = QLabel("HOLD A MOVEMENT BUTTON · RELEASE TO ZERO")
        self.motion_hint.setStyleSheet(
            f"background: rgba(17, 32, 54, 0.55); color: {C['on_surface_variant']}; "
            f"border: 1px solid {C['outline_variant']}; border-radius: 8px; "
            f"padding: 2px 8px; font-size: 11px;"
        )
        self.motion_hint.setParent(container)
        self.motion_hint.adjustSize()
        self.motion_hint.show()

        self._refresh_armed()
        self._reposition_overlays()
        return container

    def _build_flyout(self, parent):
        tab = QPushButton("\u00bb")
        tab.setFixedSize(22, 60)
        tab.setObjectName("Ghost")
        tab.setParent(parent)
        panel = GlassPanel("MISSION LOG", parent=parent)
        panel.setFixedSize(230, 220)
        self.log_body = QLabel()
        self.log_body.setWordWrap(True)
        self.log_body.setTextFormat(Qt.PlainText)
        self.log_body.setStyleSheet(
            f"color: {C['on_surface_variant']}; font-family: {FONT_DATA}; "
            f"font-size: 10px;"
        )
        panel.add_row(self.log_body)
        panel.hide()

        def toggle():
            panel.setVisible(not panel.isVisible())
            panel.raise_()

        tab.clicked.connect(toggle)
        self._flyout_panel = panel
        self._flyout_tab = tab
        return tab

    def _build_dock(self, parent):
        dock = QWidget(parent)
        lay = QHBoxLayout(dock)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        self.armed_btn = _DockButton("\u25c9", "ARMED", danger=True)
        self.armed_btn.clicked.connect(self._toggle_armed)
        self.rec_btn = _DockButton("\u25cf", "REC")
        self.rec_btn.clicked.connect(self._toggle_rec)
        self.snap_btn = _DockButton("\u25a3", "SNAP")
        self.snap_btn.clicked.connect(self._snap)
        self.light_btn = _DockButton("\u2600", "LIGHTS")
        self.light_btn.clicked.connect(self._toggle_lights)
        for b in (self.armed_btn, self.rec_btn, self.snap_btn, self.light_btn):
            lay.addWidget(b)
        dock.adjustSize()
        dock.show()
        self._refresh_armed()
        return dock

    def _refresh_armed(self):
        was_armed = self._armed
        if self.controller is not None:
            self._armed = bool(getattr(self.controller, "armed", False))
        self.armed_btn.set_active(self._armed)
        self.armed_btn.set_label("ARMED" if self._armed else "DISARMED")
        self.mode_pill.setText("MANUAL" if self._armed else "DISARMED")
        self.mode_pill.set_status("ok" if self._armed else "bad")
        full_motion = getattr(self, "full_motion", None)
        if full_motion is not None:
            full_motion.set_controls_enabled(self._armed)
        pad = getattr(self, "pad", None)
        if pad is not None:
            pad.setEnabled(self._armed)
        if was_armed and not self._armed:
            self._clear_motion_inputs()

    def _toggle_armed(self):
        if self.controller is not None:
            if self._armed:
                self.controller.disarm("dashboard")
            else:
                self.controller.arm()
        else:
            self._armed = not self._armed
        self._armed = bool(getattr(self.controller, "armed", self._armed)) \
            if self.controller is not None else self._armed
        self._clear_motion_inputs()
        self.pad.set_yaw(0.0)
        self._refresh_armed()

    def _zero_motion(self):
        if self.controller is not None:
            self.controller.zero()
        self._clear_motion_inputs()
        self._refresh_armed()

    def _confirm_estop(self):
        answer = QMessageBox.question(
            self,
            "E-STOP",
            "Send E-STOP? It remains latched until the ESP32 is power-cycled.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        if self.controller is not None:
            self.controller.emergency_stop()
        self._clear_motion_inputs()
        self._refresh_armed()

    def _toggle_rec(self):
        self._recording = not self._recording
        self.rec_btn.set_active(self._recording)
        if self._recording:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._writer is not None:
            return
        try:
            path = os.path.join(self.log_dir, "recordings")
            os.makedirs(path, exist_ok=True)
            fname = time.strftime("rec_%Y%m%d_%H%M%S.avi")
            w, h = self._cam_size or (640, 480)
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self._writer = cv2.VideoWriter(
                os.path.join(path, fname), fourcc, 30.0, (w, h)
            )
            if not self._writer.isOpened():
                self._writer = None
        except Exception:
            self._writer = None

    def _stop_recording(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None

    def _toggle_lights(self):
        self._lights = not self._lights
        self.light_btn.set_active(self._lights)

    def _snap(self):
        path = os.path.join(self.log_dir, "snapshots")
        os.makedirs(path, exist_ok=True)
        pix = self.camera_view.pixmap()
        if pix is not None:
            fname = time.strftime("%Y%m%d_%H%M%S.png")
            pix.save(os.path.join(path, fname))

    def _launch_mission(self):
        self.status_message("MISSION LAUNCH - placeholder", 2000)

    def status_message(self, message, timeout=2000):
        self.mode_pill.setText(message)
        QTimer.singleShot(timeout, self._refresh_armed)

    def _set_motion_input(self, source, motion):
        if self.controller is not None:
            self.controller.set_input(source, motion if self._armed else MotionState())

    def _clear_motion_inputs(self):
        if self._clearing_motion_inputs:
            return
        self._clearing_motion_inputs = True
        try:
            self._pressed_actions.clear()
            self.pad.clear()
            self.full_motion.clear()
            if self.controller is not None:
                self.controller.set_input("pad", MotionState())
                self.controller.set_input("full_motion", MotionState())
                self.controller.set_input("keyboard", MotionState())
        finally:
            self._clearing_motion_inputs = False

    def _reposition_overlays(self):
        cw = self.camera_container.width()
        ch = self.camera_container.height()
        m = 10
        self.live.move(m, m)
        self.telemetry_panel.move(m, self.live.y() + self.live.height() + 8)
        self.sonar.move(cw - self.sonar.width() - m, m)
        self.dock.move(max(0, (cw - self.dock.sizeHint().width()) // 2),
                       ch - self.dock.sizeHint().height() - 12)
        self.full_motion.move(
            max(0, (cw - self.full_motion.width()) // 2),
            max(0, ch - self.dock.sizeHint().height() - self.full_motion.height() - 24),
        )
        self.motion_hint.move(
            max(0, (cw - self.motion_hint.width()) // 2),
            self.full_motion.y() + self.full_motion.height() + 3,
        )
        self.pad.move(cw - self.pad.width() - m, ch - self.pad.height() - m)
        self._flyout_tab.move(0, ch // 2 - 30)
        fy = self._flyout_tab.y() + 2
        self._flyout_panel.move(20, fy)
        for w in (self.live, self.sonar, self.telemetry_panel, self.dock,
                  self.full_motion, self.motion_hint, self.pad,
                  self._flyout_tab, self._flyout_panel):
            w.raise_()

    def _set_section(self, key):
        self.stack.setCurrentWidget(self._pages[key])
        for k, btn in self.rail_btns.items():
            btn.setChecked(k == key)
        page = self._pages[key]
        upd = getattr(page, "update", None)
        if callable(upd):
            upd()

    def _axis(self, action_pos, action_neg):
        return ((action_pos in self._pressed_actions) - (action_neg in self._pressed_actions)) \
            * self.key_speed

    def _keyboard_state(self):
        return MotionState(
            surge=self._axis("forward", "back"),
            sway=self._axis("right", "left"),
            heave=self._axis("up", "down"),
            yaw=self._axis("yaw_right", "yaw_left"),
            pitch=self._axis("pitch_up", "pitch_down"),
            roll=self._axis("roll_right", "roll_left"),
            boost="boost" in self._pressed_actions,
        )

    def _handle_key(self, key, pressed):
        action = self._key_actions.get(key)
        if action is None:
            return
        if action == "kill":
            if pressed:
                self._armed = False
                if self.controller is not None:
                    self.controller.emergency_stop()
                self._refresh_armed()
            return
        if pressed:
            self._pressed_actions.add(action)
        else:
            self._pressed_actions.discard(action)
        if self.controller is not None:
            state = self._keyboard_state() if self._armed else MotionState()
            self.controller.set_input("keyboard", state)
            self.pad.set_yaw(state.yaw)

    def eventFilter(self, obj, event):
        if obj is self.camera_container and event.type() == QEvent.Type.Resize:
            self._reposition_overlays()
        if event.type() in (QEvent.Type.WindowDeactivate, QEvent.Type.ApplicationDeactivate):
            self._clear_motion_inputs()
            operations = self._pages.get("operations")
            if operations is not None:
                operations.clear_controls()
        if event.type() in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            focus = QApplication.focusWidget()
            if focus is None or not isinstance(focus, _TYPING_WIDGETS):
                self._handle_key(event.key(), event.type() == QEvent.Type.KeyPress)
        return super().eventFilter(obj, event)

    def _set_camera_signal(self, has_signal):
        if self._cam_signal is has_signal:
            return
        self._cam_signal = has_signal
        if has_signal:
            self.camera_view.setText("")
            self.live.setText(" \u25cf LIVE: CH-1 MAIN CAM")
            self.live.setStyleSheet(
                f"background: rgba(17, 32, 54, 0.55); color: {C['on_surface']}; "
                f"border: 1px solid {C['outline_variant']}; border-radius: 8px; "
                f"padding: 3px 10px; font-family: {FONT_DATA}; font-size: 12px;"
            )
        else:
            self.camera_view.setPixmap(QPixmap())
            self.camera_view.setText("NO SIGNAL\ncamera not detected - retrying")
            self.live.setText(" \u25cf NO SIGNAL")
            self.live.setStyleSheet(
                f"background: rgba(17, 32, 54, 0.55); color: {C['error']}; "
                f"border: 1px solid {C['error']}; border-radius: 8px; "
                f"padding: 3px 10px; font-family: {FONT_DATA}; font-size: 12px;"
            )

    def _update_camera(self):
        frame = self.camera.read_frame() if self.camera is not None else None
        if frame is None:
            self._set_camera_signal(False)
            return
        self._set_camera_signal(True)
        if self.hud_provider is not None:
            # In-memory snapshot only: the provider is polled by the 100 ms
            # service timer, so this 33 ms frame loop never touches the I2C bus.
            state = self.hud_provider.get_state()
            self.sonar.set_state(state)
            if self.overlay is not None:
                self.overlay.update(state)
                frame = self.overlay.render(frame, state)
        if self._lights:
            frame = np.clip(frame.astype(np.int16) + 36, 0, 255).astype(np.uint8)

        if self._writer is not None:
            self._writer.write(frame)

        self._frames += 1
        now = time.time()
        if now - self._fps_t >= 1.0:
            self._fps = self._frames / (now - self._fps_t)
            self._frames = 0
            self._fps_t = now

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        self._cam_size = (w, h)
        image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(image)
        vs = self.camera_view.size()
        if vs.isValid() and not vs.isEmpty():
            pix = pix.scaled(vs, Qt.KeepAspectRatioByExpanding,
                             Qt.SmoothTransformation)
            if pix.width() > vs.width() or pix.height() > vs.height():
                x = (pix.width() - vs.width()) // 2
                y = (pix.height() - vs.height()) // 2
                pix = pix.copy(x, y, vs.width(), vs.height())
        self.camera_view.setPixmap(pix)

    def _refresh_static(self):
        self._refresh_armed()
        tcfg = self.gcs.get("telemetry", {})
        self.batt_pill.setText(f"BATT {tcfg.get('bus_voltage', 48.2)}V")
        self.uptime_pill.setText(f"UP {self._uptime_str()}")
        link_ok = self._link_ok()
        self.link_pill.setText("LINK OK" if link_ok else "LINK DOWN")
        self.link_pill.set_status("ok" if link_ok else "bad")
        self.foot["FPS"].setText(f"FPS: {self._fps:.0f}")
        cpu, ram = _load_percent()
        self.foot["CPU"].setText(f"CPU: {cpu}")
        self.foot["RAM"].setText(f"RAM: {ram}")
        disk = "--"
        try:
            usage = shutil.disk_usage(".")
            disk = f"{usage.used / usage.total * 100:.0f}%"
        except OSError:
            pass
        self.foot["STORAGE"].setText(f"STORAGE: {disk}")
        self.foot["DEPTH"].setText(self._depth_text())
        self.foot["LATENCY"].setText(
            f"LATENCY: {self.gcs.get('latency_ms', 12)}ms")

        if self._flyout_panel.isVisible():
            self.log_body.setText(_tail(self.log_dir, "mission.log", 8))

        page = self._pages[self._current_section_key()]
        upd = getattr(page, "update", None)
        if callable(upd):
            upd()

    def _depth_text(self):
        """Footer depth from the cached payload; '--' when nothing supplies it."""
        if self.hud_provider is None:
            return "DEPTH: --"
        data = self.hud_provider.get_sensor_data()
        if not data.get("depth_valid"):
            return "DEPTH: --"
        return f"DEPTH: {data['depth']:.1f}m"

    def _current_section_key(self):
        for key, widget in self._pages.items():
            if widget is self.stack.currentWidget():
                return key
        return "dashboard"

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_overlays()
        self._update_camera()

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, self._reposition_overlays)

    def closeEvent(self, event):
        self._stop_recording()
        super().closeEvent(event)


def _load_percent():
    try:
        import psutil

        return f"{psutil.cpu_percent():.0f}%", f"{psutil.virtual_memory().percent:.0f}%"
    except Exception:
        return "--", "--"


def _tail(log_dir, name, n=8):
    from collections import deque

    path = os.path.join(log_dir, name)
    try:
        with open(path) as f:
            return "".join(deque(f, n))
    except FileNotFoundError:
        return "(no log yet)"


class _RailFrame(QFrame):
    """Nav rail that expands on hover, collapses on leave.

    The width is applied synchronously via ``sizeHint``/``resize`` instead of
    ``setFixedWidth`` so the layout minimum stays constant (the window never
    grows past the screen when the rail expands).
    """

    expand = None

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rail_width = 56

    def set_expanded_state(self, expanded):
        self._rail_width = 208 if expanded else 56
        self.resize(self._rail_width, self.height())
        self.updateGeometry()

    def sizeHint(self):
        s = super().sizeHint()
        return QSize(self._rail_width, s.height())

    def minimumSizeHint(self):
        return QSize(56, 0)

    def enterEvent(self, event):
        if self.expand is not None:
            self.expand(True)
        super().enterEvent(event)

    def leaveEvent(self, event):
        if self.expand is not None:
            self.expand(False)
        super().leaveEvent(event)


class _ValueRowCompact(QWidget):
    def __init__(self, label, value, pill=False):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(8)
        lab = caps_label(label)
        self.val = QLabel(value)
        self.val.setStyleSheet(
            f"font-family: {FONT_DATA}; font-size: 12px; color: {C['primary']};"
        )
        lay.addWidget(lab)
        lay.addStretch(1)
        lay.addWidget(self.val)
        self.setStyleSheet(
            f"background: rgba(17, 32, 54, 0.6); border-top: 1px solid "
            f"rgba(100, 255, 218, 0.5); border-radius: 6px;"
        )

    def set_value(self, v):
        self.val.setText(str(v))


class _DockButton(QWidget):
    def __init__(self, glyph, label, danger=False):
        super().__init__()
        self._danger = danger
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self.btn = QPushButton(glyph)
        self.btn.setFixedSize(42, 42)
        self.btn.setCursor(Qt.PointingHandCursor)
        self.lbl = QLabel(label)
        self.lbl.setAlignment(Qt.AlignCenter)
        self.lbl.setStyleSheet(
            f"color: {C['on_surface_variant']}; font-size: 9px; font-weight: 700;"
            f"letter-spacing: 1px;"
        )
        lay.addWidget(self.btn, 0, Qt.AlignHCenter)
        lay.addWidget(self.lbl)
        self.clicked = self.btn.clicked

    def set_active(self, active):
        if self._danger:
            if active:
                bg = "rgba(255, 180, 171, 0.30)"
                border = C['error']
                color = C['error']
            else:
                bg = "rgba(255, 180, 171, 0.08)"
                border = "rgba(255, 180, 171, 0.4)"
                color = C['on_error_container']
        else:
            bg = f"rgba(100, 255, 218, 0.15)" if active else C['surface']
            border = f"{C['primary']}" if active else "rgba(100, 255, 218, 0.2)"
            color = C['primary'] if active else C['on_surface_variant']
        self.btn.setStyleSheet(
            f"background: {bg}; color: {color}; border: 1px solid {border}; "
            f"border-radius: 21px; font-size: 16px;"
        )

    def set_label(self, text):
        self.lbl.setText(text)
