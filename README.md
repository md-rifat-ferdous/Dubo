# Hydrion Spectra ROV Controller

A modular ground-control station for a Raspberry Pi-based underwater ROV. The application combines a PySide6 control interface, camera and HUD rendering, simulated telemetry, motion control, and an optional ESP32 DUBO UART link for five motor controllers.

The project follows a simulation-first design: the complete control path can run without physical sensors, a network link, or motors. Hardware providers are enabled through configuration and can be replaced without changing the UI or controller.

> **Safety notice:** This repository contains software for controlling an underwater vehicle. Keep propellers removed, isolate the motor power supply, and verify the failsafe and emergency-stop behavior before any powered test. The ESP32 E-STOP is latched by firmware and must be cleared by power-cycling the ESP32.

## Current Status

| Area | Status | Notes |
|------|--------|-------|
| PySide6 GCS shell | Implemented | Dark Ocean dashboard, navigation rail, footer, and workspace pages |
| Camera and HUD | Implemented | OpenCV camera capture, compass, crosshair, depth/altitude overlays, and sonar map |
| Motion controller | Implemented | Pad, keyboard, link commands, arm/disarm, hold-to-move controls, zero, and E-STOP |
| Thruster mixing | Implemented | Five-motor matrix, polarity, boost scaling, deadband, and Pi-side ramping |
| ESP32 UART integration | Implemented | DUBO binary frames, CRC16, ACK/STATUS readback, sequence synchronization, and failsafe handling |
| Sensors | Simulated | Provider interface exists; real IMU/depth/compass providers are not included |
| Telemetry/network | Simulated | JSON telemetry and heartbeats are written to `logs/mission.log`; no socket transport is included |
| Gamepad | Planned | A configuration codemap exists, but no polling provider is included |
| Navigation/manipulator/mission | Planned | UI pages and module placeholders exist |
| systemd startup | Planned | The service and start/stop scripts are not implemented yet |

## Features

- Modular lifecycle through `BaseModule` and `ServiceManager`.
- Central YAML configuration loader with in-app configuration updates.
- File and console logging through `LoggerManager`.
- Camera capture with a live HUD and simulated vehicle state.
- Sonar-style local position map showing the vehicle, surface vessel, and ping.
- Six-axis motion state: surge, sway, heave, yaw, pitch, and roll.
- Strongest-input merging across pad, keyboard, and link command sources.
- Disarmed-by-default motion with readiness checks and zero-on-disarm.
- Automatic test-motion expiry, focus-loss input clearing, and guarded E-STOP actions.
- Optional ESP32 link over USB serial or a configured UART device.
- DUBO frame construction and parsing with CRC16-CCITT-FALSE validation.
- ACK and STATUS display, reconnect sequence synchronization, ESC deadband handling, and motor event logging.
- Bench tools for serial diagnosis, terminal motor control, and a PySide6 motor console.
- Unit tests for configuration, controller safety, mixing, protocol framing, and UART provider behavior.

## Architecture

```text
app/main.py
    |
    v
Application
    |-- ConfigManager
    |-- LoggerManager
    |-- ServiceManager
    |   |-- CameraManager
    |   |-- SensorManager
    |   |-- TelemetryManager
    |   |-- ControllerModule
    |   `-- ThrusterManager
    |-- 100 ms service update timer
    `-- MainWindow
        |-- dashboard and HUD
        |-- motion pad and keyboard controls
        `-- workspace pages
```

The application registers modules in this order:

1. Camera
2. Sensors
3. Telemetry
4. Controller
5. Thrusters

The Qt update timer calls `ServiceManager.update_all()` every 100 ms. This drives telemetry, simulated motion, and the ESP32 command stream without blocking the GUI thread.

## Repository Layout

```text
.
├── app/
│   └── main.py                  Application entry point
├── configs/                     YAML configuration files
│   ├── camera.yaml
│   ├── controller.yaml
│   ├── gcs.yaml
│   ├── hud.yaml
│   ├── network.yaml
│   ├── sensors.yaml
│   └── thrusters.yaml
├── core/
│   ├── application.py            Bootstrap and dependency wiring
│   ├── base_module.py            Module lifecycle contract
│   └── service_manager.py        Module registration and lifecycle
├── modules/
│   ├── camera/                   OpenCV camera provider
│   ├── config/                   YAML configuration manager
│   ├── controller/               MotionState and safety controller
│   ├── logger/                   Logging manager
│   ├── sensors/                  Simulated sensor provider
│   ├── telemetry/                Telemetry and simulated link
│   └── thrusters/                Mixer, UART provider, and DUBO protocol
├── scripts/
│   ├── motor_control.py          Terminal bench console
│   ├── motor_gui.py              PySide6 bench console
│   └── serial_diag.py            Standalone UART diagnostic
├── tests/                        Unit tests
├── ui/                           GCS shell, HUD, controls, and pages
├── logs/                         Runtime logs and media output
├── systemd/                      Reserved for future service deployment
├── .gitignore
└── README.md
```

## Requirements

The current development environment is Python 3.13.5 and uses:

- PySide6 6.11.1
- OpenCV 5.0.0.93
- NumPy 2.5.1
- Pillow 12.3.0
- PyYAML 6.0.3
- pyserial 3.5

`psutil` is optional. If it is installed, the Diagnostics and footer views show CPU and memory usage; otherwise those values display as `--`.

A graphical Linux session is required for the GCS and motor GUI. The main application can still be used to exercise non-GUI module tests without a display.

## Installation

Clone the repository and create a virtual environment:

```bash
git clone https://github.com/md-rifat-ferdous/Hydrion-Spectra.git
cd Hydrion-Spectra
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

Install the runtime dependencies:

```bash
python -m pip install \
  PySide6==6.11.1 \
  opencv-python==5.0.0.93 \
  numpy==2.5.1 \
  pillow==12.3.0 \
  PyYAML==6.0.3 \
  pyserial==3.5
```

The repository does not currently include a lock file or a `requirements.txt`. The versions above describe the tested environment; adjust them for the target Python and Raspberry Pi platform when necessary.

## Running the Application

Start the GCS from the repository root:

```bash
source .venv/bin/activate
python app/main.py
```

On a headless machine, use a virtual display or X11 forwarding. The application expects a camera device at the configured index. If camera access fails, the application remains open and the camera view shows its placeholder text; the error is written to `logs/errors.log`.

### Simulation-first startup

The checked-in thruster configuration currently selects the UART provider and has `fallback_to_simulated: false`. A missing or unopenable UART is therefore reported as unavailable instead of silently enabling simulated motion. For software-only development, change this value in `configs/thrusters.yaml`:

```yaml
thrusters:
  provider: simulated
```

The simulated provider applies motion to the simulated sensor state, so the dashboard, telemetry, sonar, and controller remain useful without a serial device or motors.

To return to the ESP32 bench setup, set `provider` back to `uart` and configure the serial port for the connected board. If `fallback_to_simulated` is explicitly set to `true`, a UART failure intentionally substitutes the simulated provider; the GCS labels this state `SIMULATED FALLBACK`. Use that mode only with motor power disconnected.

## Controls

Motion is gated by the `ARMED` state. The application starts disarmed, and the controller refuses to arm until the thruster provider is ready.

### Keyboard

| Key | Action |
|-----|--------|
| `W` / `S` | Surge forward / back |
| `A` / `D` | Sway left / right |
| `R` / `F` | Heave up / down |
| `Q` / `E` | Yaw left / right |
| `I` / `K` | Pitch up / down |
| `J` / `L` | Roll left / right |
| `Shift` | Enable boost |
| `Backspace` | Send E-STOP and disarm |

Keyboard events are ignored while a text-entry widget has focus. When the window loses focus, motion inputs are cleared.

### Mouse and GCS controls

- Drag the circular motion pad for surge and sway. Releasing the pad returns it to neutral.
- Hold the six-axis Dashboard controls to move; release to request zero.
- Use `ARMED` to enable or disable normal motion.
- Use `ZERO` to clear the current command and test override.
- Use `SNAP` to save a frame under `logs/snapshots/`.
- Use `REC` to write an AVI under `logs/recordings/` when a camera frame is available.
- Use the Operations page for guarded maintenance movement, direct M1–M5 test values, status, and E-STOP.
- Send commands such as `FORWARD 0.5`, `LEFT`, `UP`, or `STOP` through the Operations command channel. Commands are accepted by the simulated link and consumed by the controller.

## Configuration

All YAML files live in `configs/`. The application loads them through `ConfigManager`, unwraps each file's top-level section name, and makes the resulting mappings available to the modules.

| File | Important settings |
|------|--------------------|
| `camera.yaml` | Camera enablement, device, width, height, and frame rate |
| `sensors.yaml` | Sensor enablement, provider, simulated starting yaw, and simulation values |
| `network.yaml` | Telemetry enablement, simulated link, intervals, and reserved IP/port values |
| `controller.yaml` | Keyboard map, boost key, E-STOP key, and gamepad codemap |
| `thrusters.yaml` | Provider, port, baud, ramp, boost scale, deadband, mixing, polarity, and fallback |
| `hud.yaml` | HUD enablement, panel opacity, overlay toggles, and RGB colors |
| `gcs.yaml` | GCS shell values, sonar tuning, and sample power/latency values |

Values under `gcs.telemetry` and `gcs.latency_ms` are presentation defaults, not live measurements. Replace them when real sensor or network providers are added.

The Settings page can update common thruster settings in `configs/thrusters.yaml`. Restart the application after saving because the active provider is created during startup.

## ESP32 Thruster Integration

### Protocol

The Pi-side implementation in `modules/thrusters/dubo_protocol.py` uses this frame format:

```text
AA 55 | VERSION 0x01 | TYPE | LENGTH | SEQUENCE u16 LE | PAYLOAD | CRC16 u16 LE
```

CRC16-CCITT-FALSE covers the complete frame except the CRC bytes, using polynomial `0x1021`, initial value `0xFFFF`, no reflection, and no final XOR.

The provider sends motor commands on the application update cycle, applies the configured ramp, removes values inside the ESC deadband, and reads ACK and STATUS frames. A command is considered ready only after a real response is received; a loopback or open serial port alone does not report a healthy ESP32.

### Bench wiring

The current bench setup uses USB:

```text
Raspberry Pi USB <-> ESP32 USB/UART bridge
ESP32 SMPS output <-> ESC motor power
```

The default configuration is:

```yaml
thrusters:
  provider: uart
  port: /dev/ttyUSB0
  baud: 115200
  ramp: 0.1
```

For a direct GPIO UART deployment, use crossed UART connections and a common ground:

```text
Pi GPIO14 (TX) -> ESP32 RX0 / GPIO3
Pi GPIO15 (RX) <- ESP32 TX0 / GPIO1
Pi GND         <-> ESP32 GND
```

Do not connect 5 V between the Pi and ESP32. Enable the Pi serial hardware, remove the serial console from the kernel command line when required, and add the operating user to the `dialout` group.

### Motor layout

The current five-motor mixing map is:

| Motor | Role in the current configuration |
|-------|------------------------------------|
| M1 | Front vertical: heave and pitch |
| M2 | Mid horizontal: surge and yaw |
| M3 | Mid horizontal: surge and yaw |
| M4 | Rear vertical: heave, pitch, and roll |
| M5 | Rear vertical: heave, pitch, and roll |

The physical layout has no dedicated sway actuator in this configuration. If a motor is mounted in the opposite direction, change its `polarity` value; if the mechanical layout changes, edit the `mixing` weights in `configs/thrusters.yaml`.

### Safe bench procedure

1. Remove propellers and disconnect motor power from the ESCs.
2. Power the Pi and ESP32 logic only.
3. Start one serial process; the GCS, `serial_diag.py`, `motor_control.py`, and `motor_gui.py` cannot own the same port at the same time.
4. Connect and verify that the ESP32 reports `READY`, `FAILSAFE CLEAR`, and `E-STOP CLEAR` with all motor values at zero.
5. Confirm the UART sequence and ACK feedback before enabling the SMPS.
6. Switch on the SMPS and test one motor at a low value, then verify mixing and direction.
7. Send `ZERO`, wait for neutral feedback, disconnect, and switch off the SMPS.

A serial disconnect is expected to trigger the ESP32 watchdog's neutral command within its configured timeout; verify the actual timing on the bench. An E-STOP remains latched until the ESP32 is power-cycled; re-arming from the Pi does not clear it.

## Bench Tools

Use the application entry point for the full GCS. The standalone tools are useful for isolating wiring and protocol problems.

### UART diagnostic

```bash
python scripts/serial_diag.py
```

Optional arguments are a port and baud rate:

```bash
python scripts/serial_diag.py /dev/ttyUSB0 115200
```

The diagnostic sends an initial neutral command and supports `m1`–`m5`, `zero`, `fwd`, `turn`, `estop`, `ack`, and `quit`.

### Terminal motor console

```bash
python scripts/motor_control.py
```

The terminal console supports commands such as:

```text
fwd 300
back 200
up 250
left 150
m1 200 m2 200
zero
status
estop
help
quit
```

### PySide6 motor console

```bash
python scripts/motor_gui.py
```

The GUI console provides port selection, live ACK/STATUS feedback, a speed slider, hold-to-move controls, direct M1–M5 values, `ZERO`, and a confirmation-guarded E-STOP.

## Logs and Generated Files

Runtime output is written under `logs/`:

| Path | Contents |
|------|----------|
| `logs/system.log` | Application and module events |
| `logs/errors.log` | Error-level events |
| `logs/mission.log` | JSON telemetry and heartbeat frames in the simulated link |
| `logs/motor.log` | Motor connection, test, ready, zero, and E-STOP events |
| `logs/snapshots/` | Images created by `SNAP` |
| `logs/recordings/` | AVI files created by `REC` |

Log files are runtime data and should not be committed. Review generated media before adding it to version control.

## Testing

Run the unit tests from the repository root:

```bash
source .venv/bin/activate
python -m unittest discover -s tests -v
```

Run a syntax check across the application packages:

```bash
python -m compileall -q app core modules scripts tests
```

The tests do not require a camera or an ESP32. They use fake serial objects and simulated providers, so they are safe to run on a development machine.

For a manual GUI smoke test, run the application in a normal desktop session with the thruster provider set to `simulated`. Do not use a real serial port for a basic UI test.

## Known Limitations

- Sensor data is simulated; there is no real IMU, depth, compass, or leak-sensor driver in this repository.
- The telemetry `ip` and `port` fields are reserved configuration. The implemented `SimulatedLink` writes to a log and accepts injected commands; it does not open a network socket.
- Camera health and the `No Camera` presentation are not yet a complete hardware-health model.
- Gamepad mappings are defined in YAML but no gamepad provider is implemented.
- Navigation, manipulator, planner, and AI Vision pages are presentational placeholders.
- `systemd/rov-controller.service` and the shell start/stop scripts are not implemented.
- The ESP32 firmware is not included in this repository. Motor direction, watchdog timing, and target-hardware E-STOP behavior must be validated on the bench with props removed.
- Power, latency, and leak values shown in the GCS shell are sample values until real providers are connected.

## Roadmap

1. Implement and test the systemd service and safe start/stop scripts.
2. Add real sensor providers behind the existing `SensorManager` interface.
3. Add a real `Link` transport for network telemetry and command delivery.
4. Add a gamepad provider using the existing controller codemap.
5. Implement watchdog, navigation, mission, and manipulator modules.
6. Complete open-water mixing-sign and failsafe validation with the final thruster layout.

## Contributing

Keep hardware changes simulation-first, preserve the provider interfaces, and add or update unit tests for protocol, mixing, and safety changes. Do not commit credentials, serial captures, generated recordings, or real vehicle secrets.
