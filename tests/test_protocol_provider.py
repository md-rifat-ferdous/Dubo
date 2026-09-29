import time
import unittest

from modules.controller.controller import MotionState
from modules.thrusters import dubo_protocol as proto
from modules.thrusters.thruster_manager import SimulatedThrusterProvider
from modules.thrusters.uart_provider import UartThrusterProvider


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.in_waiting = 0
        self.closed = False

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def read(self, size=1):
        return b""

    def close(self):
        self.closed = True


class FakeSensors:
    def __init__(self):
        self.calls = []

    def apply_motion(self, **values):
        self.calls.append(values)


class ProtocolProviderTests(unittest.TestCase):
    def test_crc_matches_ccitt_reference_vector(self):
        self.assertEqual(proto.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_motor_command_frame_round_trip(self):
        frame = proto.build_motor_command(12, 100, -100, 0, 250, -250)
        parsed = proto.parse_frame(frame)
        self.assertIsNotNone(parsed)
        message_type, sequence, payload = parsed
        self.assertEqual(message_type, proto.MSG_MOTOR_COMMAND)
        self.assertEqual(sequence, 12)
        self.assertEqual(int.from_bytes(payload[0:2], "little", signed=True), 100)
        self.assertEqual(int.from_bytes(payload[8:10], "little", signed=True), -250)

    def make_provider(self):
        provider = UartThrusterProvider(
            {
                "ramp": 1.0,
                "deadband": {"positive": 0, "negative": 0},
            }
        )
        serial = FakeSerial()
        provider._ser = serial
        provider.last_status = {
            "sequence": 40,
            "failsafe": False,
            "estop": False,
            "motors": {name: 0 for name in ("M1", "M2", "M3", "M4", "M5")},
        }
        provider._last_rx_time = time.monotonic()
        return provider, serial

    def test_reconnect_sequence_is_synchronized(self):
        provider, serial = self.make_provider()
        provider.apply(MotionState(surge=0.5), 0.1)
        frame = proto.parse_frame(serial.writes[-1])
        self.assertEqual(frame[1], 41)
        payload = frame[2]
        self.assertEqual(int.from_bytes(payload[2:4], "little", signed=True), 500)

    def test_estop_writes_latched_command_and_zeroes_state(self):
        provider, serial = self.make_provider()
        provider.apply(MotionState(surge=1.0), 0.1)
        provider.emergency_stop()
        frame = proto.parse_frame(serial.writes[-1])
        self.assertEqual(frame[0], proto.MSG_ESTOP)
        self.assertTrue(provider.telemetry()["estop"])
        self.assertFalse(provider.recover())
        self.assertTrue(all(value == 0 for value in provider.telemetry()["applied"].values()))

    def test_estop_uses_synchronized_sequence(self):
        provider, serial = self.make_provider()
        provider.emergency_stop()
        frame = proto.parse_frame(serial.writes[-1])
        self.assertEqual(frame[0], proto.MSG_ESTOP)
        self.assertEqual(frame[1], 41)

    def test_status_power_cycle_clears_local_estop_latch(self):
        provider, _serial = self.make_provider()
        provider.emergency_stop()
        self.assertTrue(provider.telemetry()["estop"])

        def status_frame(sequence, estop):
            payload = bytearray(14)
            payload[0:2] = sequence.to_bytes(2, "little")
            payload[3] = int(estop)
            return proto.build_frame(proto.MSG_STATUS, sequence, payload)

        provider._handle_frame(status_frame(41, False))
        self.assertTrue(provider.telemetry()["estop"])
        provider._handle_frame(status_frame(42, True))
        provider._handle_frame(status_frame(43, False))
        self.assertFalse(provider.telemetry()["estop"])

    def test_simulated_estop_stays_latched_and_zeroes_output(self):
        provider = SimulatedThrusterProvider()
        provider.apply(MotionState(surge=1.0), 0.1)
        provider.emergency_stop()
        provider.apply(MotionState(surge=1.0), 0.1)
        self.assertFalse(provider.ready)
        self.assertTrue(all(value == 0 for value in provider.telemetry()["applied"].values()))
        self.assertFalse(provider.recover())

    def test_direct_motor_values_override_motion_targets(self):
        provider, serial = self.make_provider()
        provider.set_direct_values({"M1": 0.5})
        provider.apply(MotionState(surge=1.0), 0.1)
        payload = proto.parse_frame(serial.writes[-1])[2]
        self.assertEqual(int.from_bytes(payload[0:2], "little", signed=True), 500)
        self.assertEqual(int.from_bytes(payload[2:4], "little", signed=True), 0)

    def test_simulated_direct_override_does_not_move_sensors(self):
        sensors = FakeSensors()
        provider = SimulatedThrusterProvider(sensors=sensors)
        provider.set_direct_values({"M1": 0.5})
        provider.apply(MotionState(surge=1.0), 0.1)
        self.assertTrue(sensors.calls)
        motion = {key: sensors.calls[-1][key] for key in ("surge", "sway", "heave", "yaw_rate")}
        self.assertTrue(all(value == 0.0 for value in motion.values()))

    def test_deadband_removes_small_commands(self):
        provider = UartThrusterProvider(
            {
                "ramp": 1.0,
                "deadband": {"positive": 20, "negative": 176},
            }
        )
        serial = FakeSerial()
        provider._ser = serial
        provider.last_status = {
            "sequence": 1,
            "failsafe": False,
            "estop": False,
            "motors": {name: 0 for name in ("M1", "M2", "M3", "M4", "M5")},
        }
        provider._last_rx_time = time.monotonic()
        provider.apply(MotionState(surge=0.01), 0.1)
        payload = proto.parse_frame(serial.writes[-1])[2]
        self.assertEqual(int.from_bytes(payload[2:4], "little", signed=True), 0)


if __name__ == "__main__":
    unittest.main()
