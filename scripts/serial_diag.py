#!/usr/bin/env python3
"""Standalone ESP32 UART diagnostic for the DUBO thruster protocol.

Verifies wiring + protocol against the ESP32 before running the app.

Usage:
    .venv/bin/python scripts/serial_diag.py [/dev/ttyUSB0] [115200]

Interactive commands:
    m1 <v> m2 <v> m3 <v> m4 <v> m5 <v>   send motor command (-1000..1000)
    zero                                   all motors to 0
    fwd <v>                                M2=M3=<v> (surge forward)
    turn <v>                               M2=<v>, M3=-<v> (yaw)
    estop                                  send latched E-STOP
    ack                                    show last ACK
    quit
"""
import sys
import time

sys.path.insert(0, ".")
from modules.thrusters import dubo_protocol as proto
from modules.thrusters.mixer import load_thruster_config
from modules.thrusters.uart_provider import apply_deadband

try:
    import serial
except ImportError:
    print("pyserial not installed: pip install pyserial")
    sys.exit(1)


def main():
    config = load_thruster_config()
    port = sys.argv[1] if len(sys.argv) > 1 else config.get("port", "/dev/ttyUSB0")
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else int(config.get("baud", 115200))
    ser = serial.Serial(port, baud, timeout=0.2, write_timeout=1)
    print(f"Open {port} @ {baud}")

    sequence = 1
    deadband = config.get("deadband") or {}
    deadband_pos = int(deadband.get("positive", 20))
    deadband_neg = -abs(int(deadband.get("negative", 176)))
    rx = bytearray()
    last_ack = None
    last_status = None
    estop_latched = False
    estop_observed = False

    def sync_sequence():
        nonlocal sequence
        if last_status is not None:
            esp_sequence = int(last_status.get("sequence", 0))
            difference = (sequence - esp_sequence) & 0xFFFF
            if difference == 0 or difference >= 0x8000:
                sequence = (esp_sequence + 1) & 0xFFFF or 1

    def send(frame):
        ser.write(frame)

    def send_motor(values):
        nonlocal sequence
        if estop_latched:
            print("E-STOP is latched; power-cycle the ESP32 before sending motor commands")
            return
        values = [
            apply_deadband(max(-1000, min(1000, int(value))), deadband_pos, deadband_neg)
            for value in values
        ]
        sync_sequence()
        frame = proto.build_motor_command(sequence, *values)
        send(frame)
        sequence = (sequence + 1) & 0xFFFF or 1

    def send_estop():
        nonlocal sequence
        sync_sequence()
        send(proto.build_estop(sequence))
        sequence = (sequence + 1) & 0xFFFF or 1

    def drain(last_status_print):
        nonlocal last_ack, last_status, estop_latched, estop_observed
        data = ser.read(ser.in_waiting or 1)
        rx.extend(data)
        while True:
            idx = rx.find(bytes([proto.HEADER_1, proto.HEADER_2]))
            if idx > 0:
                del rx[:idx]
                continue
            if idx == -1:
                rx.clear()
                break
            if len(rx) < 5:
                break
            frame_len = 5 + 2 + rx[4] + 2
            if len(rx) < frame_len:
                break
            parsed = proto.parse_frame(bytes(rx[:frame_len]))
            del rx[:frame_len]
            if parsed is None:
                continue
            mtype, seq, payload = parsed
            if mtype == proto.MSG_ACK:
                ack = proto.parse_ack(payload)
                if ack is None:
                    continue
                code, e_seq = ack
                last_ack = (code, proto.ACK_NAMES.get(code, "?"), e_seq)
                print(f"[ACK] seq={e_seq} status={last_ack[1]}")
                if code == proto.ACK_ESTOP_ACTIVE:
                    estop_latched = True
                    estop_observed = True
                    print("  ! ESP32 E-STOP latched -- power-cycle the ESP32 to clear")
            elif mtype == proto.MSG_STATUS:
                last_status = proto.parse_status(payload)
                if last_status and last_status["estop"]:
                    estop_latched = True
                    estop_observed = True
                elif estop_observed:
                    estop_latched = False
                    estop_observed = False
                    print("  ESP32 E-STOP cleared after power-cycle")
                now = time.monotonic()
                if last_status and now - last_status_print >= 1.0:
                    last_status_print = now
                    print(f"[STATUS] fs={int(last_status['failsafe'])} "
                          f"estop={int(last_status['estop'])} motors={last_status['motors']}")
        return last_status_print

    time.sleep(0.2)
    last_status_print = drain(0.0)
    send_motor([0, 0, 0, 0, 0])
    print("Ready. Type 'help' for commands; motors remain at neutral until commanded.")

    while True:
        last_status_print = drain(last_status_print)
        try:
            cmd = input("> ").strip()
        except EOFError:
            break
        if not cmd:
            continue
        parts = cmd.split()
        if parts[0] == "quit":
            break
        elif parts[0] == "zero":
            send_motor([0, 0, 0, 0, 0])
            print("sent zero")
        elif parts[0] == "m1":
            vals = [0] * 5
            for i in range(5):
                key = f"m{i+1}"
                if key in parts and parts.index(key) + 1 < len(parts):
                    vals[i] = int(parts[parts.index(key) + 1])
            send_motor(vals)
            print(f"sent {vals}")
        elif parts[0] == "fwd":
            v = int(parts[1]) if len(parts) > 1 else 300
            send_motor([0, v, v, 0, 0])
            print(f"sent surge M2=M3={v}")
        elif parts[0] == "turn":
            v = int(parts[1]) if len(parts) > 1 else 300
            send_motor([0, v, -v, 0, 0])
            print(f"sent yaw M2={v} M3={-v}")
        elif parts[0] == "estop":
            send_estop()
            estop_latched = True
            print("sent E-STOP")
        elif parts[0] == "ack":
            print("last_ack:", last_ack)
        else:
            print("unknown command")

    ser.close()


if __name__ == "__main__":
    main()
