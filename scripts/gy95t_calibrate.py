#!/usr/bin/env python3
"""Bench calibration for the GY-95T I2C payload layout.

Prints the live 45-byte stream and every candidate roll/pitch/yaw offset so the
values in ``sensors.yaml`` (``layout.roll_offset`` / ``pitch_offset`` /
``yaw_offset``) can be pinned down from real hardware.

The GY-95T does not publish a byte map for its packed I2C buffer, so the offsets
are established empirically:

1. Leave the module flat on the bench and run this script.  Roll and pitch should
   read near 0.00 deg for every candidate triple.
2. Tilt the module 90 deg about its X axis and re-run.  The offset that moves to
   +-90.00 deg is roll.  Repeat for Y to get pitch.
3. Rotate the module about Z and re-run.  The offset that sweeps 0..360 is yaw.
4. Copy the confirmed offsets into ``configs/sensors.yaml``.

Usage:
    .venv/bin/python scripts/gy95t_calibrate.py [--bus 1] [--address 0x52]
"""

from __future__ import annotations

import argparse
import struct
import sys

sys.path.insert(0, ".")

from modules.sensors.gy95t_provider import (  # noqa: E402
    ANGLE_SCALE_DEG,
    STREAM_LENGTH,
    STREAM_PREAMBLE,
)

try:
    import smbus2
except ImportError:
    print("smbus2 not installed: .venv/bin/pip install smbus2", file=sys.stderr)
    raise SystemExit(1)


def read_stream(bus: int, address: int, chunk: int = 32) -> bytes:
    handle = smbus2.SMBus(bus)
    try:
        head = bytes(handle.read_i2c_block_data(address, 0, chunk))
        if STREAM_LENGTH <= chunk:
            return head
        tail = bytes(handle.read_i2c_block_data(address, chunk, STREAM_LENGTH - chunk))
        return head + tail
    finally:
        handle.close()


def dump(buffer: bytes) -> None:
    print(f"buffer ({len(buffer)} bytes): {buffer.hex(' ')}")
    preamble_ok = buffer[: len(STREAM_PREAMBLE)] == STREAM_PREAMBLE
    print(f"preamble {STREAM_PREAMBLE.hex(' ')}: {'match' if preamble_ok else 'MISMATCH'}")
    for row in range(0, len(buffer), 16):
        print(f"  0x{row:02X}  " + " ".join(f"{b:02X}" for b in buffer[row : row + 16]))


def candidates(buffer: bytes, scale: float) -> None:
    print("\nint16 LE at each offset, scaled by %.10f (180/32768):" % scale)
    print("  off   raw          deg")
    for off in range(0, len(buffer) - 1):
        raw = struct.unpack_from("<h", buffer, off)[0]
        deg = raw * scale
        marker = ""
        if -180.0 <= deg <= 180.0:
            marker = "  <- in range"
        if abs(deg) <= 1.0:
            marker += "  <- near zero (flat on bench?)"
        print(f"  0x{off:02X}  {raw:7d}  {deg:9.2f}{marker}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda v: int(v, 0), default=0x52)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--scale", type=float, default=ANGLE_SCALE_DEG)
    args = parser.parse_args()

    print(f"GY-95T calibration  bus={args.bus}  address=0x{args.address:02X}\n")
    for n in range(1, args.samples + 1):
        try:
            buffer = read_stream(args.bus, args.address)
        except Exception as exc:
            print(f"sample {n}: read failed: {type(exc).__name__}: {exc}")
            continue
        print(f"--- sample {n} ---")
        dump(buffer)
        if n == 1:
            candidates(buffer, args.scale)
        print()
    print("Now tilt/rotate the module and re-run to identify which offsets move.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
