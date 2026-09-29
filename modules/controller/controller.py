import time
from dataclasses import dataclass

from core.base_module import BaseModule


@dataclass
class MotionState:
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    boost: bool = False


COMMANDS = {
    "FORWARD": ("surge", 1.0),
    "BACK": ("surge", -1.0),
    "LEFT": ("sway", -1.0),
    "RIGHT": ("sway", 1.0),
    "UP": ("heave", 1.0),
    "DOWN": ("heave", -1.0),
    "TURN_LEFT": ("yaw", -1.0),
    "TURN_RIGHT": ("yaw", 1.0),
    "PITCH_UP": ("pitch", 1.0),
    "PITCH_DOWN": ("pitch", -1.0),
    "ROLL_LEFT": ("roll", -1.0),
    "ROLL_RIGHT": ("roll", 1.0),
}


class ControllerModule(BaseModule):
    """Merge input sources and enforce arm, stop, and test safety states."""

    def __init__(self, config=None, telemetry=None, thrusters=None):
        super().__init__("Controller")
        self.config = config or {}
        self.telemetry = telemetry
        self.thrusters = thrusters
        self._inputs = {}
        self._armed = False
        self._estop = False
        self._estop_observed = False
        self._test_motion = None
        self._test_values = None
        self._test_until = 0.0
        self._test_gesture = 0
        self._last_motion = MotionState()
        self._last_failsafe = False
        self._last_ready = None
        self._failsafe_grace_until = 0.0

    @property
    def armed(self):
        return self._armed

    @property
    def estop_latched(self):
        return self._estop

    @property
    def last_motion(self):
        return self._last_motion

    def set_input(self, source, motion):
        if not self._armed:
            self._inputs[source] = MotionState()
            return
        self._inputs[source] = motion

    def clear_input(self, source):
        self._inputs[source] = MotionState()
        if not self._armed:
            self._inputs[source] = MotionState()

    def clear_all_inputs(self):
        self._inputs = {source: MotionState() for source in self._inputs}
        self._clear_test_state()

    def kill(self):
        self.disarm("kill")

    def disarm(self, reason="disarm"):
        self._armed = False
        self._clear_test_state(clear_provider=True)
        self._inputs = {source: MotionState() for source in self._inputs}
        if self.thrusters is not None:
            self.thrusters.set_motion(MotionState())
        self.logger.info("Motion disarmed (%s)", reason)

    def arm(self):
        if self._estop:
            self.logger.warning("Arm rejected: E-STOP is latched; power-cycle the ESP32")
            return False
        if self.thrusters is None or not self.thrusters.can_arm():
            self.logger.warning("Arm rejected: thrusters are not ready")
            return False
        self._armed = True
        self._failsafe_grace_until = time.monotonic() + 1.0
        self.logger.info("Motion armed")
        return True

    def toggle(self):
        if self._armed:
            self.disarm("toggle")
            return False
        return self.arm()

    def emergency_stop(self):
        self._armed = False
        self._estop = True
        self._estop_observed = False
        self._clear_test_state(clear_provider=True)
        self._inputs = {source: MotionState() for source in self._inputs}
        if self.thrusters is not None:
            self.thrusters.emergency_stop()
            self.thrusters.set_motion(MotionState())
        self.logger.warning("E-STOP dispatched; latch remains until ESP32 power-cycle")

    def recover(self):
        if self._estop:
            return False
        if self.thrusters is None or not self.thrusters.can_arm():
            self.logger.warning("Recover rejected: thrusters are not ready")
            return False
        if self.thrusters is not None:
            recovered = self.thrusters.recover()
            if recovered is False:
                return False
        self._armed = True
        self._failsafe_grace_until = time.monotonic() + 1.0
        self.logger.info("Motion re-enabled")
        return True

    def zero(self):
        self._clear_test_state()
        self._inputs = {source: MotionState() for source in self._inputs}
        if self.thrusters is not None:
            zero = getattr(self.thrusters, "zero", None)
            if callable(zero):
                zero()
            else:
                self.thrusters.set_motion(MotionState())
        self._last_motion = MotionState()
        self.logger.info("All motion zero")

    def set_test_motion(self, motion, duration=0.75):
        if not self._armed:
            return False
        self._test_gesture += 1
        self._test_motion = motion
        self._test_values = None
        if self.thrusters is not None:
            clear = getattr(self.thrusters, "clear_test_values", None)
            if callable(clear):
                clear()
            self.thrusters.set_motion(motion)
        self._test_until = time.monotonic() + max(0.1, float(duration))
        return True

    def set_test_motor_values(self, values, duration=2.0):
        if not self._armed:
            return False
        self._test_gesture += 1
        self._test_motion = None
        self._test_values = dict(values)
        if self.thrusters is not None:
            self.thrusters.set_motion(MotionState())
            setter = getattr(self.thrusters, "set_test_values", None)
            if callable(setter):
                setter(self._test_values)
        self._test_until = time.monotonic() + max(0.25, float(duration))
        return True

    def clear_test(self):
        self._clear_test_state()
        if self.thrusters is not None:
            clear = getattr(self.thrusters, "clear_test_values", None)
            if callable(clear):
                clear()
        if self._armed and self.thrusters is not None:
            self.thrusters.set_motion(MotionState())
        self.logger.info("Motor test override cleared")

    def _clear_test_state(self, clear_provider=False):
        self._test_motion = None
        self._test_values = None
        self._test_until = 0.0
        if clear_provider and self.thrusters is not None:
            clear = getattr(self.thrusters, "clear_test_values", None)
            if callable(clear):
                clear()

    def _merge_inputs(self):
        merged = MotionState()
        axes = ("surge", "sway", "heave", "yaw", "pitch", "roll")
        for axis in axes:
            best = 0.0
            for motion in self._inputs.values():
                value = float(getattr(motion, axis, 0.0))
                if abs(value) > abs(best):
                    best = value
            setattr(merged, axis, max(-1.0, min(1.0, best)))
        merged.boost = any(getattr(motion, "boost", False) for motion in self._inputs.values())
        return merged

    def _apply_command(self, command):
        tokens = str(command).upper().split()
        if not tokens:
            return
        if tokens[0] in ("ESTOP", "E_STOP", "EMERGENCY_STOP"):
            self.emergency_stop()
            return
        if tokens[0] in ("ZERO", "STOP"):
            self.zero()
            return
        if tokens[0] not in COMMANDS:
            return
        if not self._armed:
            self._inputs["link"] = MotionState()
            return
        magnitude = 1.0
        for token in tokens[1:]:
            try:
                magnitude = float(token)
                break
            except ValueError:
                continue
        magnitude = max(-1.0, min(1.0, magnitude))
        axis, direction = COMMANDS[tokens[0]]
        state = MotionState()
        setattr(state, axis, direction * magnitude)
        self._inputs["link"] = state
        self.logger.info("Command %s -> %s %+.2f", tokens[0], axis, direction * magnitude)

    def update(self):
        if not self.running:
            return
        if self.telemetry is not None:
            for command in self.telemetry.consume_commands():
                self._apply_command(command)
        if self._estop:
            self._armed = False
        if self.thrusters is not None:
            try:
                status = self.thrusters.status()
                if not isinstance(status, dict):
                    raise TypeError("thruster status must be a mapping")
            except Exception:
                self.logger.exception("Thruster status unavailable; disarming")
                self.disarm("thruster status unavailable")
                status = None
            if status is not None:
                device_estop = bool(status.get("estop"))
                if device_estop:
                    self._estop_observed = True
                if self._estop and self._estop_observed and not device_estop:
                    self._estop = False
                    self._estop_observed = False
                    self._armed = False
                    self.logger.warning("ESP32 E-STOP cleared after power-cycle; arm remains required")
                if device_estop and not self._estop:
                    self._estop = True
                    self._estop_observed = True
                    self.disarm("ESP32 E-STOP")
                ready = bool(status.get("ready"))
                if self._last_ready is not None and not ready and self._last_ready:
                    self.logger.warning("Thruster readiness lost; disarming controller")
                    self.disarm("thruster not ready")
                self._last_ready = ready
                failsafe = bool(status.get("failsafe"))
                if failsafe and self._armed and time.monotonic() >= self._failsafe_grace_until:
                    self.logger.error("ESP32 reports FAILSAFE; motion is disarmed")
                    self.disarm("ESP32 failsafe")
                self._last_failsafe = failsafe
        if self._test_motion is not None or self._test_values is not None:
            if time.monotonic() >= self._test_until:
                self.clear_test()
        if not self._armed or self._estop:
            motion = MotionState()
        elif self._test_motion is not None:
            motion = self._test_motion
        else:
            motion = self._merge_inputs()
        if self.thrusters is not None:
            self.thrusters.set_motion(motion)
        self._last_motion = motion

    def status(self):
        return {
            "armed": self._armed,
            "estop": self._estop,
            "test_active": self._test_motion is not None or self._test_values is not None,
            "last_motion": {
                "surge": round(self._last_motion.surge, 3),
                "sway": round(self._last_motion.sway, 3),
                "heave": round(self._last_motion.heave, 3),
                "yaw": round(self._last_motion.yaw, 3),
                "pitch": round(self._last_motion.pitch, 3),
                "roll": round(self._last_motion.roll, 3),
            },
        }

    def health_check(self):
        if self.thrusters is None:
            return False
        return self.thrusters.health_check()
