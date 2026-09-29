import json
import os
import time
from collections import deque

from core.base_module import BaseModule


class Link:
    """Transport abstraction. SimulatedLink today; SocketLink/SerialLink later."""

    def open(self):
        raise NotImplementedError

    def send(self, frame):
        raise NotImplementedError

    def receive(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError


class SimulatedLink(Link):
    """Records outgoing frames to logs/mission.log; accepts injected commands."""

    def __init__(self, config=None, log_path=None):
        self.config = config or {}
        self.log_path = log_path
        self._file = None
        self._rx = deque()

    def open(self):
        if self.log_path:
            self._file = open(self.log_path, "a")

    def send(self, frame):
        if self._file is not None:
            self._file.write(frame + "\n")
            self._file.flush()

    def receive(self):
        commands = []
        while self._rx:
            commands.append(self._rx.popleft())
        return commands

    def inject_command(self, command):
        self._rx.append(command)

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None


class TelemetryManager(BaseModule):
    """Packs sensor state into JSON frames, sends them over a Link, emits heartbeats,
    and receives commands from the surface."""

    def __init__(self, config=None, sensors=None, camera=None, log_dir=None):
        super().__init__("Telemetry")
        self.config = config or {}
        self.sensors = sensors
        self.camera = camera
        self.link = None
        self.log_dir = log_dir or os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "logs",
        )
        self._last_frame = 0.0
        self._last_heartbeat = 0.0
        self._commands = deque()

    def initialize(self):
        super().initialize()
        name = self.config.get("link", "simulated")
        links = {"simulated": SimulatedLink}
        link_cls = links.get(name)
        if link_cls is None:
            self.logger.warning("Unknown link '%s'; using simulated", name)
            link_cls = SimulatedLink
        self.link = link_cls(
            self.config, log_path=os.path.join(self.log_dir, "mission.log")
        )
        self.link.open()
        self.logger.info("Link: %s", name)
        return True

    def _frame(self, kind, extra=None):
        payload = {"type": kind, "ts": round(time.time(), 3)}
        if extra:
            payload.update(extra)
        return json.dumps(payload)

    def _state_payload(self):
        payload = {"camera": bool(self.camera is not None and self.camera.capture is not None)}
        if self.sensors is not None:
            state = self.sensors.get_state()
            payload.update(
                depth=round(state.depth, 2),
                alt=round(state.alt, 2),
                yaw_deg=round(state.yaw_deg, 1),
                x=round(state.x, 2),
                y=round(state.y, 2),
            )
        return payload

    def update(self):
        if not self.running or self.link is None:
            return
        now = time.monotonic()
        frame_interval = self.config.get("telemetry_interval_s", 0.5)
        heartbeat_interval = self.config.get("heartbeat_interval_s", 1.0)

        if now - self._last_frame >= frame_interval:
            self._last_frame = now
            self.link.send(self._frame("telemetry", self._state_payload()))
        if now - self._last_heartbeat >= heartbeat_interval:
            self._last_heartbeat = now
            self.link.send(self._frame("heartbeat"))

        for command in self.link.receive():
            self._commands.append(command)
            self.logger.info("Command received: %s", command)

    def consume_commands(self):
        commands = list(self._commands)
        self._commands.clear()
        return commands

    def inject_command(self, command):
        if self.link is None:
            return False
        inject = getattr(self.link, "inject_command", None)
        if not callable(inject):
            self.logger.warning(
                "Link '%s' does not support command injection",
                self.config.get("link", "unknown"),
            )
            return False
        inject(command)
        return True

    def stop(self):
        super().stop()
        if self.link is not None:
            self.link.close()

    def health_check(self):
        return self.link is not None
