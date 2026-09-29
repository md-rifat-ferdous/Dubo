#!/usr/bin/env python3
"""Live attitude watcher: tilt the module and watch roll/pitch/yaw respond.

Answers "is roll/pitch actually coming through?" and, at the same time, settles
which payload byte offsets are roll/pitch/yaw.  Tilt the module while this runs.

How to read it:
  * Hold the module FLAT, note the baseline.
  * Tilt 90 deg about the module's long axis  -> the axis that hits ~90 is ROLL.
  * Tilt 90 deg the other way                -> the axis that hits ~90 is PITCH.
  * Spin it about the vertical axis          -> the axis that sweeps 0..360 is YAW.

At the end it prints the observed range per axis.  An axis that never leaves its
baseline means its offset in sensors.yaml is pointing at the wrong byte.

Usage:
    .venv/bin/python scripts/tilt_watch.py
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
    parser.add_argument("--interval-ms", type=float, default=100.0)
    args = parser.parse_args()

    logging.basicConfig(level=logging.ERROR, format="%(levelname)s %(name)s: %(message)s")
    configs = ConfigManager()
    configs.initialize()
    sensors = SensorManager(configs.get_section("sensors"))
    sensors.initialize()
    sensors.start()

    print(f"provider: {sensors.provider_name}")
    print("Tilt the module now.  Ctrl-C to stop and print the observed range.\n")
    print(f"{'roll':>8} {'pitch':>8} {'yaw':>8}   note")
    print("-" * 46)

    ranges = {"roll": [], "pitch": [], "yaw": []}
    last = None
    tick = 0
    try:
        while True:
            sensors.update()
            data = sensors.get_sensor_data()
            for axis in ranges:
                ranges[axis].append(data[axis])
            now = (data["roll"], data["pitch"], data["yaw"])
            # Only print on change (or every 2s), so the output stays readable
            # when piped to a file or another program.
            if now != last or tick % 20 == 0:
                print(
                    f"{data['roll']:>8.2f} {data['pitch']:>8.2f} {data['yaw']:>8.2f}"
                    + ("" if data["imu_valid"] else "   (read rejected)"),
                    flush=True,
                )
                last = now
            tick += 1
            time.sleep(args.interval_ms / 1000.0)
    except KeyboardInterrupt:
        pass
    finally:
        sensors.stop()

    print("\n" + "-" * 46)
    print(f"{'axis':>8} {'min':>9} {'max':>9} {'span':>9}")
    stale = []
    for axis, values in ranges.items():
        if not values:
            continue
        low, high = min(values), max(values)
        span = high - low
        print(f"{axis:>8} {low:>9.2f} {high:>9.2f} {span:>9.2f}")
        if span < 2.0:
            stale.append(axis)
    if stale:
        print(f"\nNEVER MOVED: {', '.join(stale)}")
        print("Tilt harder, or the offset for that axis in sensors.yaml (layout:) is")
        print("pointing at the wrong byte.  Re-run and tilt each axis in turn.")
    else:
        print("\nAll three axes responded.  Match the spans above against the way you")
        print("tilted the module to confirm the layout offsets in sensors.yaml.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
