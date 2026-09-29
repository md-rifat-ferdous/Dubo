#!/usr/bin/env python3
"""Verify that SensorManager is publishing live attitude, without starting Qt.

Run this first when the UI shows stale or missing attitude.  It exercises the
exact path the UI uses (``update()`` -> ``get_sensor_data()``) so you can tell a
provider/driver problem apart from a UI wiring problem.

Usage:
    .venv/bin/python scripts/sensor_probe.py            # 5 s, default config
    .venv/bin/python scripts/sensor_probe.py --seconds 20
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.config.config_manager import ConfigManager  # noqa: E402
from modules.sensors.sensor_manager import SensorManager  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--interval-ms", type=float, default=100.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    configs = ConfigManager()
    configs.initialize()
    sensors = SensorManager(configs.get_section("sensors"))
    sensors.initialize()
    sensors.start()

    print(f"\nprovider : {sensors.provider_name}")
    print(f"interval : {sensors.update_interval_s * 1000:.0f} ms")
    print(f"telemetry: {sensors.telemetry().get('open_error') or 'no open error'}\n")
    print(f"{'seq':>5} {'roll':>8} {'pitch':>8} {'yaw':>8} {'depth':>7}  flags")
    print("-" * 56)

    seen_sequences = set()
    changes = 0
    previous = None
    deadline = time.monotonic() + args.seconds
    # Mirror the app: the 100 ms Qt timer drives update_all(), which calls
    # SensorManager.update().  Here we call update() on the same cadence.
    while time.monotonic() < deadline:
        sensors.update()
        data = sensors.get_sensor_data()
        seen_sequences.add(data["sequence"])
        signature = (data["roll"], data["pitch"], data["yaw"])
        if previous is not None and signature != previous:
            changes += 1
        previous = signature
        flags = []
        if data["imu_valid"]:
            flags.append("imu-ok")
        else:
            flags.append("IMU-INVALID")
        if data["depth_valid"]:
            flags.append("depth-ok")
        else:
            flags.append("depth-none")
        print(
            f"{data['sequence']:>5} {data['roll']:>8.2f} {data['pitch']:>8.2f} "
            f"{data['yaw']:>8.2f} {data['depth']:>7.2f}  {' '.join(flags)}"
        )
        time.sleep(args.interval_ms / 1000.0)

    sensors.stop()

    print("-" * 56)
    print(f"publishes : {len(seen_sequences)} distinct sequence numbers")
    print(f"value changes: {changes}")
    if len(seen_sequences) <= 1:
        print("\nFAIL: cache never updated. Check SensorManager.update() is being")
        print("      called by ServiceManager.update_all() (application.py timer).")
        return 1
    final = sensors.get_sensor_data()
    if not final["imu_valid"]:
        print("\nWARN: payload published but imu_valid is False - the driver is")
        print("      rejecting reads. Check the Sensors log for decode errors and")
        print("      confirm the layout offsets with scripts/gy95t_calibrate.py.")
        return 2
    if changes == 0:
        print("\nNOTE: values never changed. Expected if the vehicle is stationary;")
        print("      tilt the IMU to confirm the payload tracks.")
    else:
        print("\nOK: attitude is live and changing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
