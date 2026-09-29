import math
import threading
import time
from dataclasses import dataclass

from core.base_module import BaseModule

#: Published to the UI whenever no valid attitude is available, so a view never
#: has to guard against a missing payload.
FALLBACK_SENSOR_DATA = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "depth": 0.0,
}


@dataclass
class HudState:
    x: float = 0.0
    y: float = 0.0
    depth: float = 0.0
    alt: float = 5.2
    yaw_deg: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    #: False while the attitude source is unavailable or the last read was rejected.
    imu_valid: bool = False
    #: False when the active provider has no depth source, so the UI can show
    #: "--" instead of a convincing-looking constant 0.00 m.
    depth_valid: bool = False


class SimulatedSensorProvider:
    """Reports the vehicle state; motion is applied by the simulated thrusters."""

    #: The simulated seabed gives a real depth/altitude pair.
    provides_depth = True

    def __init__(self, config):
        sim = config.get("sim", {})
        self._seabed = sim.get("seabed_depth_m", 5.2)
        self.state = HudState(
            yaw_deg=sim.get("start_yaw_deg", 45.0),
            alt=self._seabed,
            imu_valid=True,
            depth_valid=True,
        )

    def get_state(self):
        return self.state

    def apply_motion(self, surge, sway, heave, yaw_rate, dt):
        rad = math.radians(self.state.yaw_deg)
        self.state.x += (surge * math.sin(rad) + sway * math.cos(rad)) * dt
        self.state.y += (surge * math.cos(rad) - sway * math.sin(rad)) * dt
        self.state.depth -= heave * dt
        self.state.alt = max(0.0, self._seabed - self.state.depth)
        self.state.yaw_deg = (self.state.yaw_deg + yaw_rate * dt) % 360.0


def _load_gy95t(config):
    """Import the GY-95T driver lazily so a missing smbus2 cannot break startup."""
    from modules.sensors.gy95t_provider import GY95T_OK, GY95TProvider

    return GY95TProvider, GY95T_OK


class SensorManager(BaseModule):
    """Reads IMU/depth/compass through a swappable provider, exposes a HudState.

    The provider is polled once per ``update_interval_ms`` from :meth:`update`,
    which the 100 ms Qt service timer drives.  Views then read the cached
    snapshot through :meth:`get_sensor_data` / :meth:`get_state`; those are pure
    in-memory reads, so a view can poll them every frame without ever touching
    the I2C bus.  This is what keeps the GUI thread free.
    """

    def __init__(self, config=None):
        super().__init__("Sensors")
        self.config = config or {}
        self.provider = None
        self.provider_name = None
        self._lock = threading.Lock()
        self._listeners = []
        self._sequence = 0
        self._last_poll = 0.0
        self.update_interval_s = max(
            0.01, float(self.config.get("update_interval_ms", 100)) / 1000.0
        )
        self._data = self._empty_payload()

    def _empty_payload(self):
        payload = dict(FALLBACK_SENSOR_DATA)
        payload.update(
            {
                "alt": 0.0,
                "x": 0.0,
                "y": 0.0,
                "imu_valid": False,
                "depth_valid": False,
                "source": "none",
                "sequence": 0,
                "age_ms": None,
            }
        )
        return payload

    def initialize(self):
        super().initialize()
        name = str(self.config.get("provider", "simulated")).lower()
        providers = {"simulated": SimulatedSensorProvider}
        if name == "gy95t":
            try:
                provider_cls, driver_ok = _load_gy95t(self.config)
            except ImportError as exc:
                self.logger.error("GY-95T driver unavailable: %s", exc)
                provider_cls, driver_ok = None, False
            if provider_cls is None or not driver_ok:
                self.logger.warning(
                    "smbus2 is not installed; install it or set provider: simulated"
                )
                name = "simulated"
            else:
                providers["gy95t"] = provider_cls
        provider_cls = providers.get(name)
        if provider_cls is None:
            self.logger.warning("Unknown sensor provider '%s'; using simulated", name)
            provider_cls = SimulatedSensorProvider
            name = "simulated"
        try:
            self.provider = provider_cls(self.config)
            self.provider_name = name
        except Exception as exc:
            self.logger.exception("Could not create sensor provider '%s': %s", name, exc)
            self.provider = SimulatedSensorProvider(self.config)
            self.provider_name = "simulated"
        if name == "gy95t" and self.provider_name == "gy95t":
            if not self.provider.connect():
                reason = getattr(self.provider, "_open_error", None) or "connect failed"
                if bool(self.config.get("fallback_to_simulated", True)):
                    self._fallback_to_simulated(reason)
                else:
                    self.logger.error("GY-95T unavailable and fallback is disabled: %s", reason)
        self.logger.info("Sensor provider: %s", self.provider_name)
        # Prime the cache so the first UI frame already has a payload.
        self._poll(force=True)
        return True

    def _fallback_to_simulated(self, reason):
        self.logger.warning("Falling back to simulated sensors: %s", reason)
        self.provider = SimulatedSensorProvider(self.config)
        self.provider_name = "simulated (fallback)"

    # ------------------------------------------------------------------ polling

    def update(self):
        """Service-timer hook: refresh the cache at most once per interval."""
        super().update()
        if not self.running or self.provider is None:
            return
        self._poll()

    def _poll(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_poll < self.update_interval_s:
            return
        self._last_poll = now
        try:
            state = self.provider.get_state()
        except Exception:
            # A provider blowing up must not kill the 100 ms service tick.
            self.logger.exception("Sensor provider read failed")
            return
        self._publish(state, now)

    def _publish(self, state, now):
        """Store a new snapshot and notify listeners.  Never raises."""
        self._sequence += 1
        provides_depth = bool(getattr(self.provider, "provides_depth", False))
        payload = {
            "roll": round(float(state.roll), 2),
            "pitch": round(float(state.pitch), 2),
            "yaw": round(float(state.yaw_deg) % 360.0, 2),
            "depth": round(float(state.depth), 2),
            "alt": round(float(state.alt), 2),
            "x": round(float(state.x), 2),
            "y": round(float(state.y), 2),
            "imu_valid": bool(state.imu_valid),
            "depth_valid": bool(state.depth_valid) and provides_depth,
            "source": self.provider_name or "unknown",
            "sequence": self._sequence,
            "age_ms": 0.0,
        }
        with self._lock:
            self._data = payload
            listeners = list(self._listeners)
        for listener in listeners:
            try:
                listener(dict(payload))
            except Exception:
                self.logger.debug("Sensor listener failed", exc_info=True)

    # ------------------------------------------------------------------ accessors

    def add_listener(self, callback):
        """Register a callable invoked with a fresh payload on every publish."""
        with self._lock:
            self._listeners.append(callback)
        return callback

    def remove_listener(self, callback):
        with self._lock:
            if callback in self._listeners:
                self._listeners.remove(callback)

    def get_sensor_data(self):
        """Thread-safe snapshot for the UI.  Cheap: pure in-memory read.

        Always returns the documented keys, defaulting to
        ``FALLBACK_SENSOR_DATA`` when no provider is running.
        """
        with self._lock:
            data = dict(self._data)
        if not data:
            return dict(FALLBACK_SENSOR_DATA)
        for key, value in FALLBACK_SENSOR_DATA.items():
            data.setdefault(key, value)
        return data

    def get_state(self):
        """HudState view of the cache.  Back-compatible, does no I/O."""
        data = self.get_sensor_data()
        return HudState(
            x=data["x"],
            y=data["y"],
            depth=data["depth"],
            alt=data["alt"],
            yaw_deg=data["yaw"],
            roll=data["roll"],
            pitch=data["pitch"],
            imu_valid=data["imu_valid"],
            depth_valid=data["depth_valid"],
        )

    def apply_motion(self, surge=0.0, sway=0.0, heave=0.0, yaw_rate=0.0, dt=0.1):
        """Forward simulated motion; hardware providers ignore it.

        Thrusters run after Sensors in the service order, so the cache is
        refreshed here for simulated motion to avoid a one-tick display lag.
        """
        if self.provider is None:
            return
        apply_motion = getattr(self.provider, "apply_motion", None)
        if callable(apply_motion):
            apply_motion(surge, sway, heave, yaw_rate, dt)
        if isinstance(self.provider, SimulatedSensorProvider):
            self._publish(self.provider.get_state(), time.monotonic())

    def telemetry(self):
        data = self.get_sensor_data()
        provider_telemetry = getattr(self.provider, "telemetry", None)
        result = provider_telemetry() if callable(provider_telemetry) else {
            "provider": self.provider_name or "unknown",
            "ok": self.provider is not None,
        }
        result = dict(result)
        result["manager_provider"] = self.provider_name
        result["simulated_fallback"] = self.provider_name == "simulated (fallback)"
        result["last_payload"] = data
        return result

    def health_check(self):
        return self.provider is not None

    def stop(self):
        try:
            if self.provider is not None:
                close = getattr(self.provider, "close", None)
                if callable(close):
                    close()
        finally:
            super().stop()
