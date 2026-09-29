import os
import time

from core.base_module import BaseModule
from modules.controller.controller import MotionState
from modules.thrusters.mixer import (
    MOTOR_NAMES,
    mix_motion,
    normalize_mixing,
    normalize_motor_values,
    normalize_polarity,
)
from modules.thrusters.uart_provider import SERIAL_OK, UartThrusterProvider

RECONNECT_INTERVAL_S = 1.0


class SimulatedThrusterProvider:
    def __init__(self, config=None, sensors=None):
        self.config = config or {}
        self.sensors = sensors
        self.speed_mps = float(self.config.get("speed_mps", 0.8))
        self.turn_rate_degps = float(self.config.get("turn_rate_degps", 60.0))
        self.boost_scale = float(self.config.get("boost_scale", 1.0))
        deadband = self.config.get("deadband") or {}
        self.deadband_pos = max(0, int(deadband.get("positive", 20)))
        self.deadband_neg = -abs(int(deadband.get("negative", 176)))
        self.mixing = normalize_mixing(self.config.get("mixing"))
        self.polarity = normalize_polarity(self.config.get("polarity"))
        self._direct_values = None
        self._targets = {name: 0.0 for name in MOTOR_NAMES}
        self._applied = {name: 0.0 for name in MOTOR_NAMES}
        self._estop = False
        self.port = None
        self.baud = None
        self._open_error = None

    @property
    def ok(self):
        return True

    @property
    def responding(self):
        return True

    @property
    def ready(self):
        return not self._estop

    def open(self):
        return True

    def close(self):
        return None

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
        self._applied = {name: 0.0 for name in MOTOR_NAMES}
        self._direct_values = None

    def emergency_stop(self):
        self._estop = True
        self.zero()

    def recover(self):
        if self._estop:
            return False
        return True

    def shutdown(self):
        self.close()
        self.zero()

    def apply(self, motion, dt):
        if self._estop:
            self._applied = {name: 0.0 for name in MOTOR_NAMES}
            return
        if self._direct_values is not None:
            self._targets = dict(self._direct_values)
        else:
            self._targets = self.setpoints(motion)
        self._applied = {
            name: self._targets[name] for name in MOTOR_NAMES
        }
        if self.sensors is None:
            return
        sensor_motion = MotionState() if self._direct_values is not None else motion
        speed = self.speed_mps * (1.0 if self._direct_values is not None else (2.0 if motion.boost else 1.0))
        self.sensors.apply_motion(
            surge=sensor_motion.surge * speed,
            sway=sensor_motion.sway * speed,
            heave=sensor_motion.heave * speed,
            yaw_rate=sensor_motion.yaw * self.turn_rate_degps,
            dt=dt,
        )

    def telemetry(self):
        return {
            "provider": "simulated",
            "ok": True,
            "responding": True,
            "ready": self.ready,
            "port": None,
            "baud": None,
            "open_error": None,
            "estop": self._estop,
            "failsafe": False,
            "sequence": 0,
            "last_ack": {"name": "SIMULATED"},
            "status": None,
            "rx_age_s": 0.0,
            "write_age_s": 0.0,
            "targets": {name: round(value, 3) for name, value in self._targets.items()},
            "applied": {name: round(value, 3) for name, value in self._applied.items()},
            "deadband": [self.deadband_pos, self.deadband_neg],
            "mixing": self.mixing,
            "polarity": self.polarity,
        }


class ThrusterManager(BaseModule):
    def __init__(self, config=None, sensors=None, log_dir=None):
        super().__init__("Thrusters")
        self.config = config or {}
        self.sensors = sensors
        self.log_dir = log_dir or os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "logs",
        )
        self.event_log_path = os.path.join(self.log_dir, "motor.log")
        self.provider = None
        self.provider_name = None
        self._motion = MotionState()
        self._test_values = None
        self._last = None
        self._no_response_logged = False
        self._last_ready = None
        self._next_retry = 0.0
        self._retry_logged = False

    def initialize(self):
        super().initialize()
        name = str(self.config.get("provider", "simulated")).lower()
        if name not in ("simulated", "uart"):
            self.logger.warning("Unknown thruster provider '%s'; using simulated", name)
            name = "simulated"
        try:
            provider = UartThrusterProvider(self.config, sensors=self.sensors) if name == "uart" else SimulatedThrusterProvider(self.config, sensors=self.sensors)
        except Exception as exc:
            self.logger.exception("Could not create thruster provider '%s': %s", name, exc)
            provider = SimulatedThrusterProvider(self.config, sensors=self.sensors)
            name = "simulated"
        self.provider = provider
        self.provider_name = name
        if isinstance(provider, UartThrusterProvider):
            if SERIAL_OK and provider.open():
                self.logger.info("Thruster provider: uart (%s @ %d)", provider.port, provider.baud)
                self._record("connected", port=provider.port, baud=provider.baud)
            else:
                provider.close()
                if bool(self.config.get("fallback_to_simulated", True)):
                    self.logger.warning(
                        "UART provider failed to open %s: %s; falling back to simulated",
                        provider.port,
                        provider._open_error,
                    )
                    self.provider = SimulatedThrusterProvider(self.config, sensors=self.sensors)
                    self.provider_name = "simulated (fallback)"
                    self._record("fallback", reason=provider._open_error)
                else:
                    self.logger.error(
                        "UART provider unavailable and fallback is disabled: %s",
                        provider._open_error,
                    )
                    self._record("fault", reason=provider._open_error)
        self.logger.info("Thruster provider: %s", self.provider_name)
        return True

    @property
    def ready(self):
        ready = getattr(self.provider, "ready", None)
        if ready is not None:
            return bool(ready)
        return self.provider is not None

    def can_arm(self):
        can_arm = getattr(self.provider, "can_arm", None)
        if callable(can_arm):
            return bool(can_arm())
        return self.provider is not None

    def set_motion(self, motion):
        self._motion = motion

    def set_test_values(self, values):
        self._test_values = normalize_motor_values(values)
        setter = getattr(self.provider, "set_direct_values", None)
        if callable(setter):
            setter(self._test_values)
        self._record("test_override", values=dict(self._test_values))

    def clear_test_values(self):
        self._test_values = None
        clearer = getattr(self.provider, "clear_direct_values", None)
        if callable(clearer):
            clearer()

    def zero(self):
        self._motion = MotionState()
        self.clear_test_values()
        zero = getattr(self.provider, "zero", None)
        if callable(zero):
            zero()
        self._record("zero")

    def emergency_stop(self):
        self._motion = MotionState()
        self.clear_test_values()
        estop = getattr(self.provider, "emergency_stop", None)
        if callable(estop):
            estop()
            self._record("emergency_stop")
        else:
            self.logger.warning("E-STOP is not available for this provider")

    def recover(self):
        recover = getattr(self.provider, "recover", None)
        if not callable(recover):
            return True
        result = bool(recover())
        self._record("recover", result=result)
        return result

    def esp32_status(self):
        return self.status()

    def status(self):
        telemetry = getattr(self.provider, "telemetry", None)
        result = telemetry() if callable(telemetry) else {
            "provider": self.provider_name or "unknown",
            "ok": self.provider is not None,
            "responding": False,
            "ready": False,
        }
        result = dict(result)
        result["manager_provider"] = self.provider_name
        result["simulated_fallback"] = self.provider_name == "simulated (fallback)"
        result["test_active"] = self._test_values is not None
        result["motion"] = {
            "surge": round(self._motion.surge, 3),
            "sway": round(self._motion.sway, 3),
            "heave": round(self._motion.heave, 3),
            "yaw": round(self._motion.yaw, 3),
            "pitch": round(self._motion.pitch, 3),
            "roll": round(self._motion.roll, 3),
        }
        return result

    def _try_reconnect(self, now):
        """Retry the UART port so an ESP32 plugged in after startup is picked up."""
        provider = self.provider
        if not isinstance(provider, UartThrusterProvider) or provider.ok:
            self._next_retry = 0.0
            self._retry_logged = False
            return
        if now < self._next_retry:
            return
        self._next_retry = now + RECONNECT_INTERVAL_S
        if provider.open():
            self._retry_logged = False
            self._no_response_logged = False
            self._last_ready = False
            self.logger.info(
                "Thruster UART reconnected on %s @ %d", provider.port, provider.baud
            )
            self._record("connected", port=provider.port, baud=provider.baud)
        elif not self._retry_logged:
            self.logger.error(
                "Serial %s unavailable (%s); retrying every %.1fs",
                provider.port,
                provider._open_error,
                RECONNECT_INTERVAL_S,
            )
            self._retry_logged = True

    def update(self):
        if not self.running or self.provider is None:
            return
        now = time.monotonic()
        if self._last is None:
            self._last = now
            return
        dt = min(now - self._last, 0.25)
        self._last = now
        if self._test_values is not None:
            setter = getattr(self.provider, "set_direct_values", None)
            if callable(setter):
                setter(self._test_values)
        self._try_reconnect(now)
        self.provider.apply(self._motion, dt)
        current_ready = self.ready
        if current_ready and not self._last_ready:
            self._record("ready")
        self._last_ready = current_ready
        if isinstance(self.provider, UartThrusterProvider):
            if self.provider.responding:
                self._no_response_logged = False
            elif (
                self.provider.ok
                and self.provider.settled
                and not self._no_response_logged
            ):
                self.logger.error(
                    "Serial %s open but no ESP32 response; check wiring/baud (motors DO NOT run)",
                    self.provider.port,
                )
                self._record("fault", reason="no ESP32 response")
                self._no_response_logged = True

    def health_check(self):
        if self.provider is None:
            return False
        return self.ready

    def stop(self):
        self._motion = MotionState()
        self._test_values = None
        shutdown = getattr(self.provider, "shutdown", None)
        try:
            if callable(shutdown):
                shutdown()
            else:
                close = getattr(self.provider, "close", None)
                if callable(close):
                    close()
            self._record("shutdown")
        finally:
            super().stop()

    def _record(self, event, **fields):
        values = {"event": event, "ts": round(time.time(), 3)}
        values.update(fields)
        self.logger.info("Motor event: %s", values)
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            with open(self.event_log_path, "a") as stream:
                stream.write(str(values) + "\n")
        except OSError:
            self.logger.debug("Could not write motor event log", exc_info=True)
