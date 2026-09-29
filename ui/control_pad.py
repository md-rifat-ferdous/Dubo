"""Circular 3D motion controller (Hydrion Spectra style).

Stick = surge/sway. The dashed outer ring spins to indicate commanded yaw
(updated externally via `set_yaw`). Heave stays on the keyboard (R/F).
"""

import math

from PySide6.QtCore import QPointF, Qt, Signal, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from modules.controller.controller import MotionState


class ControlPad3D(QWidget):
    """Glass joystick disc with an animated yaw ring."""

    motionChanged = Signal(object)
    yawChanged = Signal(float)

    SIZE = 148
    OUTER = 66
    RADIUS = 46
    KNOB = 15

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(self.SIZE, self.SIZE)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._joy = QPointF()
        self._drag = False
        self._ring_angle = 0.0
        self._yaw_cmd = 0.0

        self._anim = QTimer(self)
        self._anim.timeout.connect(self._tick)
        self._anim.start(50)

    def clear(self):
        self._drag = False
        self._joy = QPointF()
        self._emit()
        self.update()

    def set_yaw(self, value):
        self._yaw_cmd = max(-1.0, min(1.0, value))

    def _tick(self):
        if self._yaw_cmd:
            self._ring_angle += 1.6 * self._yaw_cmd
            self.update()

    def _emit(self):
        self.motionChanged.emit(
            MotionState(surge=-self._joy.y(), sway=self._joy.x())
        )

    def _set_joy(self, pos):
        dx = (pos.x() - self.SIZE / 2) / self.RADIUS
        dy = (pos.y() - self.SIZE / 2) / self.RADIUS
        length = math.hypot(dx, dy)
        if length > 1.0:
            dx, dy = dx / length, dy / length
        self._joy = QPointF(dx, dy)
        self._emit()
        self.update()

    def mousePressEvent(self, event):
        c = QPointF(self.SIZE / 2, self.SIZE / 2)
        if (event.position() - c).manhattanLength() <= self.RADIUS + 12:
            self._drag = True
            self._set_joy(event.position())
        event.accept()

    def mouseMoveEvent(self, event):
        if self._drag:
            self._set_joy(event.position())
        event.accept()

    def mouseReleaseEvent(self, event):
        if self._drag:
            self._drag = False
            self._joy = QPointF()
            self._emit()
            self.update()
        event.accept()

    def leaveEvent(self, event):
        if self._drag:
            self._drag = False
            self._joy = QPointF()
            self._emit()
            self.update()
        super().leaveEvent(event)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx = cy = self.SIZE / 2

        p.setBrush(QColor(17, 34, 64, 185))
        p.setPen(QPen(QColor(100, 255, 218, 60), 1))
        p.drawEllipse(QPointF(cx, cy), self.OUTER, self.OUTER)
        p.setPen(QPen(QColor(100, 255, 218, 140), 1))
        p.drawEllipse(QPointF(cx, cy), self.OUTER - 3, self.OUTER - 3)

        p.save()
        p.translate(cx, cy)
        p.rotate(self._ring_angle)
        dash = QPen(QColor(100, 255, 218, 90), 1)
        dash.setDashPattern([4, 5])
        p.setPen(dash)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(0, 0), self.OUTER - 9, self.OUTER - 9)
        p.restore()

        p.setBrush(QColor(39, 53, 76, 220))
        p.setPen(QPen(QColor(100, 255, 218, 80), 1))
        p.drawEllipse(QPointF(cx, cy), self.RADIUS, self.RADIUS)

        p.setPen(QPen(QColor(100, 255, 218, 110), 1))
        p.drawLine(int(cx - self.RADIUS), int(cy), int(cx + self.RADIUS), int(cy))
        p.drawLine(int(cx), int(cy - self.RADIUS), int(cx), int(cy + self.RADIUS))

        knob = self._joy * (self.RADIUS - self.KNOB - 4)
        p.setBrush(QColor(100, 255, 218, 70))
        p.setPen(QPen(QColor(100, 255, 218, 220), 2))
        p.drawEllipse(QPointF(cx + knob.x(), cy + knob.y()), self.KNOB, self.KNOB)

        p.setPen(QPen(QColor(186, 202, 195, 200), 1))
        small = p.font()
        small.setPointSize(8)
        small.setBold(True)
        p.setFont(small)
        p.drawText(0, 10, self.SIZE, 14, Qt.AlignHCenter, "FWD")
        p.drawText(0, self.SIZE - 24, self.SIZE, 14, Qt.AlignHCenter, "REV")
        p.drawText(8, 0, 14, self.SIZE, Qt.AlignVCenter, "L")
        p.drawText(self.SIZE - 22, 0, 14, self.SIZE, Qt.AlignVCenter, "R")
        p.end()


class FullMotionController(QFrame):
    motionChanged = Signal(object)
    zeroRequested = Signal()
    estopRequested = Signal()

    AXES = (
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

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            "background: rgba(17, 32, 54, 0.82); border: 1px solid "
            "rgba(100, 255, 218, 0.35); border-radius: 10px;"
        )
        self.setFixedSize(292, 246)
        self._held = set()
        self._buttons = {}
        self._boost = QCheckBox("BOOST")
        self._boost.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._speed = QSlider(Qt.Orientation.Horizontal)
        self._speed.setRange(0, 1000)
        self._speed.setValue(300)
        self._speed.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._speed_label = QLabel("300")
        self._build()
        self._speed.valueChanged.connect(self._update_speed)
        self._boost.toggled.connect(self._emit)
        self._update_speed(self._speed.value())

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(6)
        title = QLabel("MOVEMENT CONTROL")
        title.setStyleSheet("color: #64ffda; font-weight: 700; letter-spacing: 1px;")
        root.addWidget(title)
        speed = QHBoxLayout()
        speed.setSpacing(6)
        speed.addWidget(QLabel("SPEED"))
        speed.addWidget(self._speed, 1)
        speed.addWidget(self._speed_label)
        root.addLayout(speed)
        grid = QGridLayout()
        grid.setSpacing(4)
        for index, (label, axis, direction) in enumerate(self.AXES):
            button = QPushButton(label)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            button.setMinimumHeight(28)
            key = (axis, direction)
            button.pressed.connect(lambda a=axis, d=direction: self._press(a, d))
            button.released.connect(lambda a=axis, d=direction: self._release(a, d))
            self._buttons[key] = button
            grid.addWidget(button, index // 3, index % 3)
        root.addLayout(grid, 1)
        actions = QHBoxLayout()
        actions.setSpacing(5)
        zero = QPushButton("ZERO")
        zero.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        zero.clicked.connect(self.zeroRequested)
        estop = QPushButton("E-STOP")
        estop.setObjectName("Danger")
        estop.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        estop.clicked.connect(self.estopRequested)
        actions.addWidget(zero)
        actions.addWidget(estop)
        actions.addWidget(self._boost)
        root.addLayout(actions)

    def _update_speed(self, value):
        self._speed_label.setText(str(int(value)))
        if self._held:
            self._emit()

    def _press(self, axis, direction):
        if not self.isEnabled():
            return
        self._held.add((axis, direction))
        self._emit()

    def _release(self, axis, direction):
        self._held.discard((axis, direction))
        self._emit()

    def _emit(self, *_):
        motion = MotionState(boost=self._boost.isChecked())
        speed = self._speed.value() / 1000.0
        for axis in ("surge", "sway", "heave", "yaw", "pitch", "roll"):
            value = 0.0
            if (axis, 1.0) in self._held:
                value += speed
            if (axis, -1.0) in self._held:
                value -= speed
            setattr(motion, axis, max(-1.0, min(1.0, value)))
        self.motionChanged.emit(motion)

    def clear(self):
        self._held.clear()
        self._emit()

    def set_controls_enabled(self, enabled):
        self.setEnabled(enabled)
        for button in self._buttons.values():
            button.setEnabled(enabled)
        self._speed.setEnabled(enabled)
        self._boost.setEnabled(enabled)

    def set_speed(self, value):
        self._speed.setValue(int(value))
