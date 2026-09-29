import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from core.service_manager import ServiceManager
from modules.camera.camera_manager import CameraManager
from modules.config.config_manager import ConfigManager
from modules.controller.controller import ControllerModule
from modules.logger.logger_manager import LoggerManager
from modules.sensors.sensor_manager import SensorManager
from modules.telemetry.telemetry_manager import TelemetryManager
from modules.thrusters.thruster_manager import ThrusterManager
from ui.hud import HudOverlay
from ui.main_window import MainWindow
from ui.theme import apply_theme


class Application:
    """Top-level bootstrap: wires config, logger, services, and the Qt GUI."""

    def __init__(self, argv=None, configs_dir=None, log_dir=None):
        self.argv = list(argv) if argv is not None else sys.argv
        self.config_manager = ConfigManager(configs_dir)
        self.logger_manager = LoggerManager(log_dir)
        self.service_manager = ServiceManager()
        self.qt_app = None
        self.main_window = None
        self.update_timer = None
        self.log = None

    def initialize(self):
        self.config_manager.initialize()
        self.logger_manager.initialize()
        self.log = self.logger_manager.get_logger("app")
        self._build_modules()
        self.service_manager.initialize_all()
        self.service_manager.start_all()

    def _build_modules(self):
        camera = None
        sensors = None
        if self.config_manager.get("camera", "enabled", True):
            camera = CameraManager(self.config_manager.get_section("camera"))
            self.service_manager.register(camera)
        if self.config_manager.get("sensors", "enabled", True):
            sensors = SensorManager(self.config_manager.get_section("sensors"))
            self.service_manager.register(sensors)

        telemetry = None
        if self.config_manager.get("network", "enabled", True):
            telemetry = TelemetryManager(
                self.config_manager.get_section("network"),
                sensors=sensors,
                camera=camera,
            )
            self.service_manager.register(telemetry)

        thrusters = None
        if self.config_manager.get("thrusters", "enabled", True):
            thrusters = ThrusterManager(
                self.config_manager.get_section("thrusters"),
                sensors=sensors,
                log_dir=self.logger_manager.log_dir,
            )
        if self.config_manager.get("controller", "enabled", True):
            self.service_manager.register(
                ControllerModule(
                    self.config_manager.get_section("controller"),
                    telemetry=telemetry,
                    thrusters=thrusters,
                )
            )
        if thrusters is not None:
            self.service_manager.register(thrusters)

    def create_window(self):
        hud_config = {"hud": self.config_manager.get_section("hud")}
        camera = self.service_manager.get("Camera")
        sensors = self.service_manager.get("Sensors")
        controller = self.service_manager.get("Controller")
        telemetry = self.service_manager.get("Telemetry")
        keymap = self.config_manager.get_section("controller").get("keymap", {})
        return MainWindow(
            camera,
            overlay=HudOverlay(hud_config),
            hud_provider=sensors,
            controller=controller,
            keymap=keymap,
            service_manager=self.service_manager,
            config_manager=self.config_manager,
            telemetry=telemetry,
            thrusters=self.service_manager.get("Thrusters"),
            log_dir=self.logger_manager.log_dir,
            gcs_config=self.config_manager.get_section("gcs") or {},
        )

    def run(self):
        self.initialize()
        self.qt_app = QApplication(self.argv)
        apply_theme(self.qt_app)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.service_manager.update_all)
        self.update_timer.start(100)
        self.main_window = self.create_window()
        self.main_window.show()
        self.log.info("Main window shown, entering event loop")
        try:
            exit_code = self.qt_app.exec()
        finally:
            self.shutdown()
        return exit_code

    def shutdown(self):
        if self.update_timer is not None:
            self.update_timer.stop()
        self.service_manager.stop_all()
        if self.log is not None:
            self.log.info("Application shutdown complete")
